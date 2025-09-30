#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32
#ifdef HAS_ESP32_P4_CAMERA

#include "esp_timer.h"
#include "driver/ledc.h"
#include "esp_cache.h"

// External sensor detection functions
extern "C" {
  esp_cam_sensor_device_t *sc2336_detect(esp_cam_sensor_config_t *config);
  esp_cam_sensor_device_t *ov5645_detect(esp_cam_sensor_config_t *config);
  esp_cam_sensor_device_t *sc202cs_detect(esp_cam_sensor_config_t *config); // Added for SC202CS support
}

static const char *const TAG = "tab5_camera";

// Configuration constants based on M5Stack demo
#define SCCB0_PORT_NUM I2C_NUM_0
#define TAB5_CAMERA_H_RES 640
#define TAB5_CAMERA_V_RES 480
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 200
#define TAB5_ISP_CLOCK_HZ 50000000

namespace esphome {
namespace tab5_camera {

Tab5Camera::~Tab5Camera() {
  this->deinit_camera_();
}

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera with M5Stack configuration...");

  // Create synchronization objects
  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  if (!this->frame_ready_semaphore_) {
    ESP_LOGE(TAG, "Failed to create frame ready semaphore");
    this->mark_failed();
    return;
  }

  this->frame_queue_ = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(FrameData));
  if (!this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create frame queue");
    this->mark_failed();
    return;
  }

  // Initialize camera following M5Stack sequence
  if (!this->init_i2c_bus_()) {
    ESP_LOGE(TAG, "Failed to initialize I2C bus");
    this->mark_failed();
    return;
  }

  if (!this->init_sccb_()) {
    ESP_LOGE(TAG, "Failed to initialize SCCB");
    this->mark_failed();
    return;
  }

  if (!this->detect_camera_sensor_()) {
    ESP_LOGE(TAG, "Failed to detect camera sensor");
    this->mark_failed();
    return;
  }

  if (!this->init_camera_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize camera sensor");
    this->mark_failed();
    return;
  }

  ESP_LOGCONFIG(TAG, "Tab5 Camera setup completed successfully");
  this->camera_initialized_ = true;
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Platform: ESP32-P4 MIPI-CSI (M5Stack)");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  SCCB Address: 0x%02X", this->sensor_address_);
  ESP_LOGCONFIG(TAG, "  SCCB SCL: GPIO%u", this->sccb_scl_pin_);
  ESP_LOGCONFIG(TAG, "  SCCB SDA: GPIO%u", this->sccb_sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCCB Frequency: %u Hz", this->sccb_frequency_);
  if (this->external_clock_pin_ > 0) {
    ESP_LOGCONFIG(TAG, "  External Clock Pin: GPIO%u", this->external_clock_pin_);
    ESP_LOGCONFIG(TAG, "  External Clock Frequency: %u Hz", this->external_clock_frequency_);
  }
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup Failed");
  }
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

bool Tab5Camera::is_ready() const {
  return this->camera_initialized_ && this->sensor_initialized_;
}

// Initialize I2C bus following M5Stack pattern
bool Tab5Camera::init_i2c_bus_() {
  ESP_LOGI(TAG, "Initializing I2C master bus");
  
  i2c_master_bus_config_t i2c_bus_config = {};
  i2c_bus_config.i2c_port = SCCB0_PORT_NUM;
  i2c_bus_config.scl_io_num = static_cast<gpio_num_t>(this->sccb_scl_pin_); // Cast explicite
  i2c_bus_config.sda_io_num = static_cast<gpio_num_t>(this->sccb_sda_pin_); // Cast explicite
  i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
  i2c_bus_config.glitch_ignore_cnt = 7;
  i2c_bus_config.intr_priority = 0;
  i2c_bus_config.trans_queue_depth = 0;
  i2c_bus_config.flags.enable_internal_pullup = true;

  esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &this->i2c_bus_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "I2C master bus initialized successfully");
  return true;
}



// Detect camera sensor following M5Stack pattern
bool Tab5Camera::detect_camera_sensor_() {
  ESP_LOGI(TAG, "Detecting camera sensor");
  
  esp_cam_sensor_config_t cam_config = {};
  cam_config.sccb_handle = this->sccb_handle_;

  // Récupérez le numéro de broche pour reset_pin
 




  cam_config.pwdn_pin = GPIO_NUM_NC;
  cam_config.xclk_pin = GPIO_NUM_NC;  // We handle external clock separately
  cam_config.sensor_port = ESP_CAM_SENSOR_MIPI_CSI;

#ifdef CONFIG_CAMERA_SC2336
  this->cam_sensor_ = sc2336_detect(&cam_config);
  if (this->cam_sensor_) {
    ESP_LOGI(TAG, "SC2336 camera sensor detected successfully");
  }
#elif CONFIG_CAMERA_OV5645
  this->cam_sensor_ = ov5645_detect(&cam_config);
  if (this->cam_sensor_) {
    ESP_LOGI(TAG, "OV5645 camera sensor detected successfully");
  }
#elif CONFIG_CAMERA_SC202CS
  this->cam_sensor_ = sc202cs_detect(&cam_config);
  if (this->cam_sensor_) {
    ESP_LOGI(TAG, "SC202CS camera sensor detected successfully");
  }
#else
  ESP_LOGW(TAG, "No specific camera sensor configured, using generic detection");
  // Try SC2336 as default
  this->cam_sensor_ = sc2336_detect(&cam_config);
  if (this->cam_sensor_) {
    ESP_LOGI(TAG, "Camera sensor detected (generic SC2336)");
  }
#endif

  if (!this->cam_sensor_) {
    ESP_LOGE(TAG, "Failed to detect camera sensor");
    return false;
  }

  this->sensor_initialized_ = true;
  return true;
}


// Initialize camera sensor
bool Tab5Camera::init_camera_sensor_() {
  ESP_LOGI(TAG, "Initializing camera sensor");
  
  if (!this->cam_sensor_) {
    ESP_LOGE(TAG, "Camera sensor not detected");
    return false;
  }

  // Setup external clock if configured
  if (!this->setup_external_clock_()) {
    ESP_LOGE(TAG, "Failed to setup external clock");
    return false;
  }

  // Initialize LDO for MIPI PHY
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "Failed to initialize MIPI LDO");
    return false;
  }

  // Initialize CSI controller
  if (!this->init_csi_controller_()) {
    ESP_LOGE(TAG, "Failed to initialize CSI controller");
    return false;
  }

  // Initialize ISP processor
  if (!this->init_isp_processor_()) {
    ESP_LOGE(TAG, "Failed to initialize ISP processor");
    return false;
  }

  // Allocate frame buffers
  if (!this->allocate_frame_buffers_()) {
    ESP_LOGE(TAG, "Failed to allocate frame buffers");
    return false;
  }

  ESP_LOGI(TAG, "Camera sensor initialized successfully");
  return true;
}

bool Tab5Camera::setup_external_clock_() {
  if (this->external_clock_pin_ == 0) {
    ESP_LOGI(TAG, "No external clock pin configured");
    return true;
  }

  ESP_LOGI(TAG, "Setting up external clock on GPIO%u at %u Hz", 
           this->external_clock_pin_, this->external_clock_frequency_);

  ledc_timer_config_t timer_conf = {};
  timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
  timer_conf.freq_hz = this->external_clock_frequency_;
  timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_conf.deconfigure = false;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  timer_conf.timer_num = LEDC_TIMER_0;
  
  esp_err_t ret = ledc_timer_config(&timer_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
    return false;
  }

  ledc_channel_config_t ch_conf = {};
  ch_conf.gpio_num = this->external_clock_pin_;
  ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  ch_conf.channel = LEDC_CHANNEL_0;
  ch_conf.intr_type = LEDC_INTR_DISABLE;
  ch_conf.timer_sel = LEDC_TIMER_0;
  ch_conf.duty = 1;  // 50% duty cycle
  ch_conf.hpoint = 0;
  ch_conf.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;
  
  ret = ledc_channel_config(&ch_conf);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "External clock setup completed");
  return true;
}

bool Tab5Camera::init_ldo_() {
  ESP_LOGI(TAG, "Initializing MIPI LDO regulator");
  
  esp_ldo_channel_config_t ldo_cfg = {};
  ldo_cfg.chan_id = 3;
  ldo_cfg.voltage_mv = 2500;
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_cfg, &this->ldo_mipi_phy_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to acquire MIPI LDO channel: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "MIPI LDO regulator initialized");
  return true;
}

bool Tab5Camera::init_csi_controller_() {
  ESP_LOGI(TAG, "Initializing CSI controller");
  
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = this->frame_width_;
  csi_config.v_res = this->frame_height_;
  csi_config.lane_bit_rate_mbps = TAB5_MIPI_CSI_LANE_BITRATE_MBPS;
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 2;
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 4;

  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create CSI controller: %s", esp_err_to_name(ret));
    return false;
  }

  // Register callbacks
  esp_cam_ctlr_evt_cbs_t cbs = {};
  cbs.on_get_new_trans = nullptr;
  cbs.on_trans_finished = Tab5Camera::camera_get_finished_trans_callback;
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register camera callbacks: %s", esp_err_to_name(ret));
    return false;
  }

  ret = esp_cam_ctlr_enable(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable CSI controller: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "CSI controller initialized successfully");
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
    FrameData frame_data;
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

}  // namespace tab5_camera
}  // namespace esphome

#endif  // HAS_ESP32_P4_CAMERA
#endif  // USE_ESP32


























