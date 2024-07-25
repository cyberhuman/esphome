#include "esp_adf_speaker.h"

#ifdef USE_ESP_IDF

#include <inttypes.h>

#include <driver/i2s.h>

#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <audio_hal.h>
#include <filter_resample.h>
#include <i2s_stream.h>
#include <raw_stream.h>

namespace esphome {
namespace esp_adf {

static const size_t BUFFER_COUNT = 50;

static const char *const TAG = "esp_adf.speaker";

void ESPADFSpeaker::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ESP ADF Speaker...");

  ExternalRAMAllocator<uint8_t> allocator(ExternalRAMAllocator<uint8_t>::ALLOW_FAILURE);

  this->buffer_queue_.storage = allocator.allocate(sizeof(StaticQueue_t) + (BUFFER_COUNT * sizeof(DataEvent)));
  if (this->buffer_queue_.storage == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate buffer queue!");
    this->mark_failed();
    return;
  }

  this->buffer_queue_.handle =
      xQueueCreateStatic(BUFFER_COUNT, sizeof(DataEvent), this->buffer_queue_.storage + sizeof(StaticQueue_t),
                         (StaticQueue_t *) (this->buffer_queue_.storage));

  this->event_queue_ = xQueueCreate(20, sizeof(TaskEvent));
  if (this->event_queue_ == nullptr) {
    ESP_LOGW(TAG, "Could not allocate event queue.");
    this->mark_failed();
    return;
  }
}

void ESPADFSpeaker::start() { this->state_ = speaker::STATE_STARTING; }
void ESPADFSpeaker::start_() {
  if (!this->parent_->try_lock()) {
    return;  // Waiting for another i2s component to return lock
  }

  ESP_LOGI(TAG, "ESPADFSpeaker::start_ %p now_playing_data %p length %zd", this, this->now_playing_data, this->now_playing_length);
  xTaskCreate(ESPADFSpeaker::player_task, "speaker_task", 8192, (void *) this, 2, &this->player_task_handle_);
}

void ESPADFSpeaker::player_task(void *params) {
  ESPADFSpeaker *this_speaker = (ESPADFSpeaker *) params;

  TaskEvent event;
  event.type = TaskEventType::STARTING;
  xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);

  i2s_driver_config_t i2s_config = {
      .mode = (i2s_mode_t) (I2S_MODE_MASTER | I2S_MODE_TX),
      .sample_rate = 16000,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
      .communication_format = I2S_COMM_FORMAT_STAND_I2S,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL2 | ESP_INTR_FLAG_IRAM,
      .dma_buf_count = 8,
      .dma_buf_len = 1024,
      .use_apll = false,
      .tx_desc_auto_clear = true,
      .fixed_mclk = 0,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
      .bits_per_chan = I2S_BITS_PER_CHAN_DEFAULT,
  };

  audio_pipeline_cfg_t pipeline_cfg = {
      .rb_size = 8 * 1024,
  };
  audio_pipeline_handle_t pipeline = audio_pipeline_init(&pipeline_cfg);

  i2s_stream_cfg_t i2s_cfg = {
      .type = AUDIO_STREAM_WRITER,
      .i2s_config = i2s_config,
      .i2s_port = I2S_NUM_0,
      .use_alc = false,
      .volume = 0,
      .out_rb_size = I2S_STREAM_RINGBUFFER_SIZE,
      .task_stack = I2S_STREAM_TASK_STACK,
      .task_core = I2S_STREAM_TASK_CORE,
      .task_prio = I2S_STREAM_TASK_PRIO,
      .stack_in_ext = false,
      .multi_out_num = 0,
      .uninstall_drv = true,
      .need_expand = false,
      .expand_src_bits = I2S_BITS_PER_SAMPLE_16BIT,
  };
  audio_element_handle_t i2s_stream_writer = i2s_stream_init(&i2s_cfg);

  rsp_filter_cfg_t rsp_cfg = {
      .src_rate = 16000,
      .src_ch = 1,
      .dest_rate = 16000,
      .dest_bits = 16,
      .dest_ch = 2,
      .src_bits = 16,
      .mode = RESAMPLE_DECODE_MODE,
      .max_indata_bytes = RSP_FILTER_BUFFER_BYTE,
      .out_len_bytes = RSP_FILTER_BUFFER_BYTE,
      .type = ESP_RESAMPLE_TYPE_AUTO,
      .complexity = 2,
      .down_ch_idx = 0,
      .prefer_flag = ESP_RSP_PREFER_TYPE_SPEED,
      .out_rb_size = RSP_FILTER_RINGBUFFER_SIZE,
      .task_stack = RSP_FILTER_TASK_STACK,
      .task_core = RSP_FILTER_TASK_CORE,
      .task_prio = RSP_FILTER_TASK_PRIO,
      .stack_in_ext = true,
  };
  audio_element_handle_t filter = rsp_filter_init(&rsp_cfg);

  raw_stream_cfg_t raw_cfg = {
      .type = AUDIO_STREAM_WRITER,
      .out_rb_size = 8 * 1024,
  };
  audio_element_handle_t raw_write = raw_stream_init(&raw_cfg);

  audio_pipeline_register(pipeline, raw_write, "raw");
  audio_pipeline_register(pipeline, filter, "filter");
  audio_pipeline_register(pipeline, i2s_stream_writer, "i2s");

  const char *link_tag[3] = {
      "raw",
      // "filter",
      "i2s",
  };
  audio_pipeline_link(pipeline, &link_tag[0], 2);

  audio_pipeline_run(pipeline);

  DataEvent data_event;

  this_speaker->is_player_task_active = true;

  event.type = TaskEventType::STARTED;
  xQueueSend(this_speaker->event_queue_, &event, 0);

  TickType_t last_received = xTaskGetTickCount();
  //uint32_t last_received = millis();

  while (true) {
    TickType_t now = xTaskGetTickCount();
    int32_t wait_timeout = 500l / portTICK_PERIOD_MS - (now - last_received);
    if (wait_timeout < 0) {
      wait_timeout = 0;
    }
    //uint32_t now = millis();
    //ESP_LOGI(TAG, "ESPADFSpeaker::player_task %p now %" PRIu32 " last_received %" PRIu32 " | now_playing_data %p length %zd",
    //  this_speaker, now, last_received, this_speaker->now_playing_data, this_speaker->now_playing_length);
    if (xQueueReceive(this_speaker->buffer_queue_.handle, &data_event, wait_timeout) != pdTRUE) {
      // No audio for 500ms, stop
      break;
    }
    if (data_event.stop) {
      // Stop signal from main thread
      while (xQueueReceive(this_speaker->buffer_queue_.handle, &data_event, 0) == pdTRUE) {
        // Flush queue
      }
      break;
    }

    size_t remaining = data_event.len;
    size_t current = 0;

    if (remaining > 0)
      last_received = now;
    while (remaining > 0) {
      int bytes_written = raw_stream_write(raw_write, (char *) data_event.data + current, remaining);
      if (bytes_written == ESP_FAIL) {
        event = {.type = TaskEventType::WARNING, .err = ESP_FAIL};
        xQueueSend(this_speaker->event_queue_, &event, 0);
        continue;
      }

      remaining -= bytes_written;
      current += bytes_written;
    }

    event.type = TaskEventType::RUNNING;
    xQueueSend(this_speaker->event_queue_, &event, 0);
  }

  this_speaker->is_player_task_active = false;

  ESP_LOGI(TAG, "ESPADFSpeaker::player_task %p stopping 1 | now_playing_data %p length %zd", this_speaker, this_speaker->now_playing_data, this_speaker->now_playing_length);
  audio_pipeline_stop(pipeline);
  audio_pipeline_wait_for_stop(pipeline);
  audio_pipeline_terminate(pipeline);

  ESP_LOGI(TAG, "ESPADFSpeaker::player_task %p stopping 2 | now_playing_data %p length %zd", this_speaker, this_speaker->now_playing_data, this_speaker->now_playing_length);
  event.type = TaskEventType::STOPPING;
  xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);

  ESP_LOGI(TAG, "ESPADFSpeaker::player_task %p stopping 3 | now_playing_data %p length %zd", this_speaker, this_speaker->now_playing_data, this_speaker->now_playing_length);
  audio_pipeline_unregister(pipeline, i2s_stream_writer);
  audio_pipeline_unregister(pipeline, filter);
  audio_pipeline_unregister(pipeline, raw_write);

  ESP_LOGI(TAG, "ESPADFSpeaker::player_task %p stopping 4 | now_playing_data %p length %zd", this_speaker, this_speaker->now_playing_data, this_speaker->now_playing_length);
  audio_pipeline_deinit(pipeline);
  audio_element_deinit(i2s_stream_writer);
  audio_element_deinit(filter);
  audio_element_deinit(raw_write);

  ESP_LOGI(TAG, "ESPADFSpeaker::player_task %p stopping 5 | now_playing_data %p length %zd", this_speaker, this_speaker->now_playing_data, this_speaker->now_playing_length);
  event.type = TaskEventType::STOPPED;
  xQueueSend(this_speaker->event_queue_, &event, portMAX_DELAY);

  //ESP_LOGI(TAG, "ESPADFSpeaker::player_task %p stopping 6 | now_playing_data %p length %zd", this_speaker, this_speaker->now_playing_data, this_speaker->now_playing_length);
  while (true) {
    vTaskDelay(portMAX_DELAY);
  }
}

void ESPADFSpeaker::stop() {
  if (this->state_ == speaker::STATE_STOPPED)
    return;
  if (this->state_ == speaker::STATE_STARTING) {
    this->state_ = speaker::STATE_STOPPED;
    return;
  }
  this->state_ = speaker::STATE_STOPPING;
  DataEvent data;
  data.stop = true;
  xQueueSendToFront(this->buffer_queue_.handle, &data, portMAX_DELAY);
}

void ESPADFSpeaker::watch_() {
  TaskEvent event;
  if (xQueueReceive(this->event_queue_, &event, 0) == pdTRUE) {
    switch (event.type) {
      case TaskEventType::STARTING:
        ESP_LOGI(TAG, "ESPADFSpeaker::watch_ %p starting | now_playing_data %p length %zd", this, this->now_playing_data, this->now_playing_length);
        break;
      case TaskEventType::STOPPING:
        ESP_LOGI(TAG, "ESPADFSpeaker::watch_ %p stopping | now_playing_data %p length %zd", this, this->now_playing_data, this->now_playing_length);
        break;
      case TaskEventType::STARTED:
        ESP_LOGI(TAG, "ESPADFSpeaker::watch_ %p started | now_playing_data %p length %zd", this, this->now_playing_data, this->now_playing_length);
        this->state_ = speaker::STATE_RUNNING;
        break;
      case TaskEventType::RUNNING:
        this->status_clear_warning();
        break;
      case TaskEventType::STOPPED:
        this->parent_->unlock();
        this->state_ = speaker::STATE_STOPPED;
        vTaskDelete(this->player_task_handle_);
        this->player_task_handle_ = nullptr;
        ESP_LOGI(TAG, "ESPADFSpeaker::watch_ %p stopped | now_playing_data %p length %zd", this, this->now_playing_data, this->now_playing_length);
        this->keep_playing_();
        break;
      case TaskEventType::WARNING:
        ESP_LOGW(TAG, "Error writing to pipeline: %s", esp_err_to_name(event.err));
        this->status_set_warning();
        break;
    }
  }
}

void ESPADFSpeaker::loop() {
  this->watch_();
  switch (this->state_) {
    case speaker::STATE_STARTING:
      this->start_();
      break;
    case speaker::STATE_RUNNING:
      this->keep_playing_();
      break;
    case speaker::STATE_STOPPING:
    case speaker::STATE_STOPPED:
      break;
  }
}

size_t ESPADFSpeaker::play(const uint8_t *data, size_t length) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Failed to play audio, speaker is in failed state.");
    return 0;
  }
  ESP_LOGI(TAG,
    "Will be sending data %p bytes %zd | prev now_playing_data %p length %zd buffered chunks %d",
    data, length, this->now_playing_data, this->now_playing_length, uxQueueMessagesWaiting(this->buffer_queue_.handle));
  now_playing_data = data;
  now_playing_length = length;
  return this->keep_playing_();
}

size_t ESPADFSpeaker::keep_playing_() {
  if (!this->now_playing_length) {
    return 0;
  }
  if (this->state_ == speaker::STATE_STOPPED) {
    this->start();
  }
  if (!this->is_player_task_active) {
    return 0;
  }
  size_t sent_length = 0;
  while (now_playing_length > 0) {
    DataEvent event;
    event.stop = false;
    size_t to_send_length = std::min(now_playing_length, BUFFER_SIZE);
    event.len = to_send_length;
    memcpy(event.data, now_playing_data, to_send_length);
    if (xQueueSend(this->buffer_queue_.handle, &event, 0) != pdTRUE) {
      ESP_LOGI(TAG, "Buffer is full, sent bytes %zd, data %p remaining bytes %zd", sent_length, now_playing_data, now_playing_length);
      return sent_length;  // Queue full
    }
    now_playing_data += to_send_length;
    now_playing_length -= to_send_length;
    sent_length += to_send_length;
  }
  ESP_LOGI(TAG, "Finished sending bytes %zd data %p", sent_length, now_playing_data);
  return sent_length;
}

bool ESPADFSpeaker::has_buffered_data() const { return uxQueueMessagesWaiting(this->buffer_queue_.handle) > 0; }

}  // namespace esp_adf
}  // namespace esphome

#endif  // USE_ESP_IDF
