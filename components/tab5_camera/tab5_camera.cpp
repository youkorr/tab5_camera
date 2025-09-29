#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#ifdef HAS_ESP32_P4_CAMERA

#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_cache.h"
#include "esp_cam_sensor.h"

#include "tab5_camera_defaults.h"   // <-- fallback macro definitions
#include "sc202cs.h"                // <-- sensor driver prototype

static const char *const TAG = "tab5_camera";

/*------------------------------------------------------------------*/
/*                         PUBLIC API                               */
/*------------------------------------------------------------------*/

Tab5Camera::~Tab5Camera() {
  this->deinit_camera_();
}

/*-------------------- Component life‑cycle ------------------------*/
void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera (M5Stack‑style)…");

  /*--- Synchronisation primitives --------------------------------*/
  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  if (!this->frame_ready_semaphore_) {
    ESP_LOGE(TAG, "Failed to create frame‑ready semaphore");
    this->mark_failed();
    return;
  }

  this->frame_queue_ = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(FrameData));
  if (!this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create frame queue");
    this->mark_failed();
    return;
  }

  /*--- Initialise the hardware chain (M5Stack reference) --------*/
  if (!this->init_i2c_bus_())            { ESP_LOGE(TAG, "I2C init failed");            this->mark_failed(); return; }
  if (!this->init_sccb_())               { ESP_LOGE(TAG, "SCCB init failed");           this->mark_failed(); return; }
  if (!this->detect_camera_sensor_())    { ESP_LOGE(TAG, "Sensor detection failed");    this->mark_failed(); return; }
  if (!this->init_camera_sensor_())      { ESP_LOGE(TAG, "Sensor init failed");          this->mark_failed(); return; }

  ESP_LOGCONFIG(TAG, "Tab5 Camera setup OK");
  this->camera_initialized_ = true;
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Platform: ESP32‑P4 (MIPI‑CSI)");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Pixel format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  SCCB address: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  SCCB pins: SCL GPIO%u  SDA GPIO%u", this->sccb_scl_pin_, this->sccb_sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCCB frequency: %u Hz", this->sccb_frequency_);
  if (this->external_clock_pin_ != GPIO_NUM_NC) {
    ESP_LOGCONFIG(TAG, "  External clock: GPIO%u @ %u Hz", this->external_clock_pin_, this->external_clock_frequency_);
  }
  if (this->reset_pin_) LOG_PIN("  Reset pin: ", this->reset_pin_);
  if (this->is_failed())
    ESP_LOGCONFIG(TAG, "  Setup FAILED – %s", this->last_error_.c_str());
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

bool Tab5Camera::is_ready() const {
  return this->camera_initialized_ && this->sensor_initialized_;
}

/*------------------------------------------------------------------*/
/*                         SNAPSHOT API                             */
/*------------------------------------------------------------------*/

bool Tab5Camera::take_snapshot() {
  if (!this->is_ready()) {
    ESP_LOGE(TAG, "Camera not ready for snapshot");
    return false;
  }

  ESP_LOGINFO(TAG, "Taking snapshot");

  esp_cam_ctlr_trans_t trans = {};
  trans.buffer = this->frame_buffer_;
  trans.buflen = this->frame_buffer_size_;

  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 5000 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Snapshot failed: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGDEBUG(TAG, "Snapshot received %zu bytes", trans.received_size);
  esp_cache_msync(this->frame_buffer_, trans.received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  this->process_frame_(static_cast<uint8_t *>(this->frame_buffer_), trans.received_size);
  return true;
}

/*------------------------------------------------------------------*/
/*                         STREAMING API                             */
/*------------------------------------------------------------------*/

bool Tab5Camera::start_streaming() {
  if (!this->is_ready()) {
    ESP_LOGE(TAG, "Camera not ready for streaming");
    return false;
  }
  if (this->streaming_active_) {
    ESP_LOGW(TAG, "Streaming already active");
    return true;
  }

  ESP_LOGINFO(TAG, "Starting streaming");
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;

  /*--- start the CSI controller (continuous mode) ---------------*/
  esp_err_t ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_cam_ctlr_start() failed: %s", esp_err_to_name(ret));
    this->streaming_active_ = false;
    return false;
  }

  /*--- launch the FreeRTOS task that pulls frames from the queue ---*/
  BaseType_t r = xTaskCreate(
      Tab5Camera::streaming_task,
      "tab5_stream",
      STREAMING_TASK_STACK_SIZE,
      this,
      STREAMING_TASK_PRIORITY,
      &this->streaming_task_handle_);

  if (r != pdPASS) {
    ESP_LOGE(TAG, "Failed to create streaming task");
    esp_cam_ctlr_stop(this->cam_handle_);
    this->streaming_active_ = false;
    return false;
  }

  ESP_LOGINFO(TAG, "Streaming started");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_)
    return true;   // already stopped

  ESP_LOGINFO(TAG, "Stopping streaming");
  this->streaming_should_stop_ = true;

  /* Wake the task if it is blocked on the semaphore */
  if (this->streaming_task_handle_) {
    xSemaphoreGive(this->frame_ready_semaphore_);
    /* Wait a short while for the task to exit gracefully */
    uint32_t timeout = 0;
    while (this->streaming_active_ && timeout < 50) {
      vTaskDelay(pdMS_TO_TICKS(100));
      ++timeout;
    }
    if (this->streaming_active_) {
      ESP_LOGW(TAG, "Force‑deleting streaming task");
      vTaskDelete(this->streaming_task_handle_);
      this->streaming_active_ = false;
    }
    this->streaming_task_handle_ = nullptr;
  }

  /* Stop the CSI controller */
  esp_cam_ctlr_stop(this->cam_handle_);
  ESP_LOGINFO(TAG, "Streaming stopped");
  return true;
}

/*------------------------------------------------------------------*/
/*                     LOW‑LEVEL HARDWARE SETUP                     */
/*------------------------------------------------------------------*/

/*-------------------- I2C bus ------------------------------------*/
bool Tab5Camera::init_i2c_bus_() {
  ESP_LOGDEBUG(TAG, "Creating I2C master bus (port %d)", SCCB0_PORT_NUM);

  i2c_master_bus_config_t cfg = {};
  cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  cfg.i2c_port   = SCCB0_PORT_NUM;
  cfg.scl_io_num = this->sccb_scl_pin_;   // gpio_num_t already
  cfg.sda_io_num = this->sccb_sda_pin_;
  cfg.flags.enable_internal_pullup = true;

  esp_err_t ret = i2c_new_master_bus(&cfg, &this->i2c_bus_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "i2c_new_master_bus() failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGDEBUG(TAG, "I2C master bus created");
  return true;
}

/*-------------------- SCCB --------------------------------------*/
bool Tab5Camera::init_sccb_() {
  ESP_LOGDEBUG(TAG, "Creating SCCB interface (addr 0x%02X)", this->sensor_address_);

  esp_sccb_i2c_config_t cfg = {};
  cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  cfg.device_address  = this->sensor_address_;
  cfg.scl_speed_hz    = this->sccb_frequency_;

  esp_err_t ret = esp_sccb_new_i2c_io(this->i2c_bus_handle_, &cfg, &this->sccb_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_sccb_new_i2c_io() failed: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGDEBUG(TAG, "SCCB interface ready");
  return true;
}

/*-------------------- Sensor detection --------------------------*/
bool Tab5Camera::detect_camera_sensor_() {
  ESP_LOGINFO(TAG, "Detecting camera sensor (SC202CS)…");

  esp_cam_sensor_config_t cam_cfg = {};
  cam_cfg.sccb_handle = this->sccb_handle_;
  cam_cfg.reset_pin   = (this->reset_pin_) ? this->reset_pin_->get_pin() : -1;
  cam_cfg.pwdn_pin    = -1;
  cam_cfg.xclk_pin    = -1;   // external clock handled separately
  cam_cfg.xclk_freq_hz = 0;
  cam_cfg.sensor_port = ESP_CAM_SENSOR_MIPI_CSI;

  this->cam_sensor_ = sc202cs_detect(&cam_cfg);
  if (!this->cam_sensor_) {
    ESP_LOGE(TAG, "SC202CS sensor not found");
    return false;
  }

  ESP_LOGI(TAG, "SC202CS detected – PID=0x%04X", this->cam_sensor_->id.pid);
  this->sensor_initialized_ = true;
  return true;
}

/*-------------------- Camera sensor init -----------------------*/
bool Tab5Camera::init_camera_sensor_() {
  ESP_LOGINFO(TAG, "Initialising camera sensor");

  if (!this->cam_sensor_) {
    ESP_LOGE(TAG, "No sensor handle – cannot initialise");
    return false;
  }

  /*--- external clock (if configured) --------------------------*/
  if (!this->setup_external_clock_()) {
    ESP_LOGE(TAG, "External‑clock setup failed");
    return false;
  }

  /*--- power‑rail for the MIPI PHY ---------------------------*/
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "MIPI LDO acquisition failed");
    return false;
  }

  /*--- CSI controller ----------------------------------------*/
  if (!this->init_csi_controller_()) {
    ESP_LOGE(TAG, "CSI controller init failed");
    return false;
  }

  /*--- ISP processor ------------------------------------------*/
  if (!this->init_isp_processor_()) {
    ESP_LOGE(TAG, "ISP processor init failed");
    return false;
  }

  /*--- allocate frame buffers ---------------------------------*/
  if (!this->allocate_frame_buffers_()) {
    ESP_LOGE(TAG, "Frame‑buffer allocation failed");
    return false;
  }

  ESP_LOGINFO(TAG, "Camera sensor fully initialised");
  return true;
}

/*-------------------- External clock (LEDC PWM) ----------------*/
bool Tab5Camera::setup_external_clock_() {
  if (this->external_clock_pin_ == GPIO_NUM_NC) {
    ESP_LOGDEBUG(TAG, "No external clock pin configured – skipping");
    return true;
  }

  ESP_LOGINFO(TAG, "Configuring external clock: GPIO%u @ %u Hz",
              this->external_clock_pin_, this->external_clock_frequency_);

  ledc_timer_config_t tcfg = {};
  tcfg.duty_resolution = LEDC_TIMER_1_BIT;   // 50 % duty
  tcfg.freq_hz         = this->external_clock_frequency_;
  tcfg.speed_mode      = LEDC_LOW_SPEED_MODE;
  tcfg.timer_num       = LEDC_TIMER_0;
  tcfg.clk_cfg         = LEDC_AUTO_CLK;

  esp_err_t ret = ledc_timer_config(&tcfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ledc_timer_config() failed: %s", esp_err_to_name(ret));
    return false;
  }

  ledc_channel_config_t ccfg = {};
  ccfg.gpio_num   = this->external_clock_pin_;
  ccfg.speed_mode = LEDC_LOW_SPEED_MODE;
  ccfg.channel    = LEDC_CHANNEL_0;
  ccfg.timer_sel  = LEDC_TIMER_0;
  ccfg.duty       = 1;          // 1‑bit resolution → 50 %
  ccfg.hpoint     = 0;
  ccfg.intr_type  = LEDC_INTR_DISABLE;
  ccfg.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;

  ret = ledc_channel_config(&ccfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ledc_channel_config() failed: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGDEBUG(TAG, "External clock PWM configured");
  return true;
}

/*-------------------- MIPI LDO -----------------------------------*/
bool Tab5Camera::init_ldo_() {
  ESP_LOGDEBUG(TAG, "Acquiring MIPI‑PHY LDO (2.5 V)");

  esp_ldo_channel_config_t cfg = {
      .chan_id   = 3,      // channel 3 is the one wired to the MIPI PHY on ESP32‑P4
      .voltage_mv = 2500,
  };

  esp_err_t ret = esp_ldo_acquire_channel(&cfg, &this->ldo_mipi_phy_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_ldo_acquire_channel() failed: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGDEBUG(TAG, "MIPI LDO acquired");
  return true;
}

/*-------------------- CSI controller ----------------------------*/
bool Tab5Camera::init_csi_controller_() {
  ESP_LOGDEBUG(TAG, "Creating CSI controller");

  esp_cam_ctlr_csi_config_t cfg = {};
  cfg.ctlr_id               = 0;
  cfg.h_res                 = this->frame_width_;
  cfg.v_res                 = this->frame_height_;
  cfg.lane_bit_rate_mbps    = 200;               // matches TAB5_MIPI_CSI_LANE_BITRATE_MBPS
  cfg.input_data_color_type = CAM_CTLR_COLOR_RAW8;   // sensor outputs RAW8 in our default format
  cfg.output_data_color_type= CAM_CTLR_COLOR_RGB565; // we want RGB565 after ISP
  cfg.data_lane_num         = 2;                  // typical for Tab5 board
  cfg.byte_swap_en          = false;
  cfg.queue_items           = 4;

  esp_err_t ret = esp_cam_new_csi_ctlr(&cfg, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_cam_new_csi_ctlr() failed: %s", esp_err_to_name(ret));
    return false;
  }

  /* Register the callback that is invoked when a frame finishes */
  esp_cam_ctlr_evt_cbs_t cbs = {};
  cbs.on_get_new_trans   = nullptr;
  cbs.on_trans_finished  = Tab5Camera::camera_get_finished_trans_callback;

  ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "register_event_callbacks() failed: %s", esp_err_to_name(ret));
    return false;
  }

  ret = esp_cam_ctlr_enable(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_cam_ctlr_enable() failed: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGDEBUG(TAG, "CSI controller ready");
  return true;
}

bool Tab5Camera::init_isp_processor_() {
  ESP_LOGI(TAG, "Initializing ISP processor");

  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_hz = TAB5_ISP_CLOCK_HZ;
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.h_res = this->frame_width_;
  isp_config.v_res = this->frame_height_;

  esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create ISP processor: %s", esp_err_to_name(ret));
    return false;
  }

  ret = esp_isp_enable(this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "ISP processor initialized successfully");
  return true;
}

bool Tab5Camera::allocate_frame_buffers_() {
  ESP_LOGI(TAG, "Allocating frame buffers");

  this->frame_buffer_size_ = this->frame_width_ * this->frame_height_ * 2; // RGB565
  this->frame_buffer_size_ = (this->frame_buffer_size_ + 63) & ~63; // 64-byte alignment

  ESP_LOGI(TAG, "Frame buffer size: %zu bytes", this->frame_buffer_size_);

  // Allocate main frame buffer
  this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_,
                                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!this->frame_buffer_) {
    ESP_LOGE(TAG, "Failed to allocate main frame buffer in PSRAM");
    this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_,
                                                  MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!this->frame_buffer_) {
      ESP_LOGE(TAG, "Failed to allocate main frame buffer in DMA RAM");
      return false;
    }
  }

  ESP_LOGI(TAG, "Frame buffer allocated at %p", this->frame_buffer_);
  memset(this->frame_buffer_, 0, this->frame_buffer_size_);

  return true;
}

bool Tab5Camera::take_snapshot() {
  if (!this->is_ready()) {
    ESP_LOGE(TAG, "Camera not ready for snapshot");
    return false;
  }

  ESP_LOGI(TAG, "Taking snapshot");

  esp_cam_ctlr_trans_t trans = {};
  trans.buffer = this->frame_buffer_;
  trans.buflen = this->frame_buffer_size_;

  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 5000 / portTICK_PERIOD_MS);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to capture frame: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "Snapshot captured: %zu bytes", trans.received_size);

  // Sync cache
  esp_cache_msync(this->frame_buffer_, trans.received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

  // Process frame
  this->process_frame_(static_cast<uint8_t*>(this->frame_buffer_), trans.received_size);

  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->is_ready()) {
    ESP_LOGE(TAG, "Camera not ready for streaming");
    return false;
  }

  if (this->streaming_active_) {
    ESP_LOGW(TAG, "Streaming already active");
    return true;
  }

  ESP_LOGI(TAG, "Starting camera streaming");

  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;

  // Start camera controller
  esp_err_t ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
    this->streaming_active_ = false;
    return false;
  }

  // Create streaming task
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task,
    "tab5_streaming",
    STREAMING_TASK_STACK_SIZE,
    this,
    STREAMING_TASK_PRIORITY,
    &this->streaming_task_handle_
  );

  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create streaming task");
    esp_cam_ctlr_stop(this->cam_handle_);
    this->streaming_active_ = false;
    return false;
  }

  ESP_LOGI(TAG, "Camera streaming started successfully");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return true;
  }

  ESP_LOGI(TAG, "Stopping camera streaming");

  this->streaming_should_stop_ = true;

  if (this->streaming_task_handle_) {
    // Signal semaphore to wake up task
    xSemaphoreGive(this->frame_ready_semaphore_);

    // Wait for task to finish
    uint32_t timeout = 0;
    while (this->streaming_active_ && timeout < 50) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      timeout++;
    }

    if (this->streaming_active_) {
      ESP_LOGW(TAG, "Force stopping streaming task");
      vTaskDelete(this->streaming_task_handle_);
      this->streaming_active_ = false;
    }

    this->streaming_task_handle_ = nullptr;
  }

  // Stop camera controller
  esp_cam_ctlr_stop(this->cam_handle_);

  ESP_LOGI(TAG, "Camera streaming stopped");
  return true;
}

void Tab5Camera::streaming_task(void *parameter) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(parameter);
  camera->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGI(TAG, "Streaming loop started");

  // Start first capture
  esp_cam_ctlr_trans_t trans = {};
  trans.buffer = this->frame_buffer_;
  trans.buflen = this->frame_buffer_size_;
  esp_cam_ctlr_receive(this->cam_handle_, &trans, 0);

  while (!this->streaming_should_stop_) {
    // Wait for frame ready signal
    if (xSemaphoreTake(this->frame_ready_semaphore_, 100 / portTICK_PERIOD_MS) == pdTRUE) {
      FrameData frame;
      if (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE) {
        if (frame.valid) {
          ESP_LOGD(TAG, "Processing frame: %zu bytes", frame.size);
          // Frame processing is done in callback
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  this->streaming_active_ = false;
  ESP_LOGI(TAG, "Streaming loop ended");
  vTaskDelete(nullptr);
}

bool Tab5Camera::camera_get_finished_trans_callback(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data) {

  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !trans->buffer) {
    return false;
  }

  static uint32_t frame_count = 0;
  frame_count++;

  if (trans->received_size > 0) {
    // Sync cache
    esp_cache_msync(trans->buffer, trans->received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);

    // Process frame
    camera->process_frame_(static_cast<uint8_t*>(trans->buffer), trans->received_size);

    // Queue frame data
    FrameData frame_data = {};
    frame_data.buffer = trans->buffer;
    frame_data.size = trans->received_size;
    frame_data.timestamp = esp_timer_get_time();
    frame_data.valid = true;

    BaseType_t ret = xQueueSendFromISR(camera->frame_queue_, &frame_data, NULL);
    if (ret == pdTRUE) {
      xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, NULL);
    }
  }

  // Start next capture
  esp_cam_ctlr_trans_t new_trans = {};
  new_trans.buffer = trans->buffer;
  new_trans.buflen = camera->frame_buffer_size_;
  esp_cam_ctlr_receive(handle, &new_trans, 0);

  return false;
}

void Tab5Camera::process_frame_(uint8_t *data, size_t len) {
  this->frame_count_++;

  ESP_LOGD(TAG, "Frame #%u: %zu bytes", this->frame_count_, len);

  // Trigger callbacks
  this->trigger_on_frame_callbacks_(data, len);
}

void Tab5Camera::trigger_on_frame_callbacks_(uint8_t *data, size_t len) {
  for (auto *trigger : this->on_frame_triggers_) {
    trigger->trigger(data, len);
  }
}

void Tab5Camera::deinit_camera_() {
  if (this->streaming_active_) {
    this->stop_streaming();
  }

  ESP_LOGI(TAG, "Deinitializing camera");

  // Delete camera sensor
  if (this->cam_sensor_) {
    esp_cam_sensor_del_dev(this->cam_sensor_);
    this->cam_sensor_ = nullptr;
  }

  // Clean up camera controller
  if (this->cam_handle_) {
    esp_cam_ctlr_stop(this->cam_handle_);
    esp_cam_ctlr_disable(this->cam_handle_);
    esp_cam_ctlr_del(this->cam_handle_);
    this->cam_handle_ = nullptr;
  }

  // Clean up ISP processor
  if (this->isp_proc_) {
    esp_isp_disable(this->isp_proc_);
    esp_isp_del_processor(this->isp_proc_);
    this->isp_proc_ = nullptr;
  }

  // Clean up SCCB
  if (this->sccb_handle_) {
    esp_sccb_del_i2c_io(this->sccb_handle_);
    this->sccb_handle_ = nullptr;
  }

  // Clean up I2C bus
  if (this->i2c_bus_handle_) {
    i2c_del_master_bus(this->i2c_bus_handle_);
    this->i2c_bus_handle_ = nullptr;
  }

  // Release LDO
  if (this->ldo_mipi_phy_) {
    esp_ldo_release_channel(this->ldo_mipi_phy_);
    this->ldo_mipi_phy_ = nullptr;
  }

  // Free frame buffers
  if (this->frame_buffer_) {
    heap_caps_free(this->frame_buffer_);
    this->frame_buffer_ = nullptr;
  }

  for (size_t i = 0; i < NUM_FRAME_BUFFERS; i++) {
    if (this->frame_buffers_[i]) {
      heap_caps_free(this->frame_buffers_[i]);
      this->frame_buffers_[i] = nullptr;
    }
  }

  // Clean up synchronization objects
  if (this->frame_ready_semaphore_) {
    vSemaphoreDelete(this->frame_ready_semaphore_);
    this->frame_ready_semaphore_ = nullptr;
  }

  if (this->frame_queue_) {
    vQueueDelete(this->frame_queue_);
    this->frame_queue_ = nullptr;
  }

  // Reset state
  this->camera_initialized_ = false;
  this->sensor_initialized_ = false;
  this->streaming_active_ = false;

  ESP_LOGI(TAG, "Camera deinitialization completed");
}

void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  ESP_LOGE(TAG, "Camera error: %s", error.c_str());
}

PixelFormat Tab5Camera::parse_pixel_format_(const std::string &format) const {
  if (format == "RAW8") return PixelFormat::RAW8;
  if (format == "RAW10") return PixelFormat::RAW10;
  if (format == "YUV422") return PixelFormat::YUV422;
  if (format == "RGB565") return PixelFormat::RGB565;
  if (format == "JPEG") return PixelFormat::JPEG;
  return PixelFormat::RGB565;
}

size_t Tab5Camera::calculate_frame_size_() const {
  uint16_t bytes_per_pixel = 2; // RGB565 default

  switch (this->parse_pixel_format_(this->pixel_format_)) {
    case PixelFormat::RAW8:
      bytes_per_pixel = 1;
      break;
    case PixelFormat::RAW10:
      bytes_per_pixel = 2;
      break;
    case PixelFormat::YUV422:
      bytes_per_pixel = 2;
      break;
    case PixelFormat::RGB565:
      bytes_per_pixel = 2;
      break;
    case PixelFormat::JPEG:
      bytes_per_pixel = 1; // Variable, approximation
      break;
  }

  return this->frame_width_ * this->frame_height_ * bytes_per_pixel;
}

}  // namespace tab5_camera
}  // namespace esphome

#endif  // HAS_ESP32_P4_CAMERA
#endif  // USE_ESP32
















