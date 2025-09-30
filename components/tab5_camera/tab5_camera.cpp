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
}

static const char *const TAG = "tab5_camera";

// Configuration constants
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
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  SCCB Address: 0x%02X", this->sensor_address_);
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

// -------- Correction principale : utilisation du bus ESPHome -----------
bool Tab5Camera::init_i2c_bus_() {
  ESP_LOGI(TAG, "Attaching to ESPHome I2C bus");

  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "No I2C parent bus found, check i2c_id in YAML");
    return false;
  }

  this->i2c_bus_handle_ = this->parent_->get_bus_handle();
  if (!this->i2c_bus_handle_) {
    ESP_LOGE(TAG, "Failed to get I2C bus handle from parent");
    return false;
  }

  ESP_LOGI(TAG, "Using ESPHome I2C bus successfully");
  return true;
}

// Initialize SCCB
bool Tab5Camera::init_sccb_() {
  ESP_LOGI(TAG, "Initializing SCCB interface");

  sccb_i2c_config_t sccb_config = {};
  sccb_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  sccb_config.device_address = this->sensor_address_;
  sccb_config.scl_speed_hz = this->sccb_frequency_;

  esp_err_t ret = sccb_new_i2c_io(this->i2c_bus_handle_, &sccb_config, &this->sccb_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create SCCB interface: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "SCCB interface initialized successfully");
  return true;
}

// Detect camera sensor
bool Tab5Camera::detect_camera_sensor_() {
  ESP_LOGI(TAG, "Detecting camera sensor");

  esp_cam_sensor_config_t cam_config = {};
  cam_config.sccb_handle = this->sccb_handle_;
  cam_config.reset_pin = (this->reset_pin_) ? static_cast<gpio_num_t>(this->reset_pin_->get_pin()) : GPIO_NUM_NC;
  cam_config.pwdn_pin = GPIO_NUM_NC;
  cam_config.xclk_pin = GPIO_NUM_NC;
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
#else
  ESP_LOGW(TAG, "No specific camera sensor configured, trying SC2336 by default");
  this->cam_sensor_ = sc2336_detect(&cam_config);
  if (this->cam_sensor_) {
    ESP_LOGI(TAG, "Camera sensor detected (SC2336)");
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

  if (!this->setup_external_clock_()) {
    ESP_LOGE(TAG, "Failed to setup external clock");
    return false;
  }

  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "Failed to initialize MIPI LDO");
    return false;
  }

  if (!this->init_csi_controller_()) {
    ESP_LOGE(TAG, "Failed to initialize CSI controller");
    return false;
  }

  if (!this->init_isp_processor_()) {
    ESP_LOGE(TAG, "Failed to initialize ISP processor");
    return false;
  }

  if (!this->allocate_frame_buffers_()) {
    ESP_LOGE(TAG, "Failed to allocate frame buffers");
    return false;
  }

  ESP_LOGI(TAG, "Camera sensor initialized successfully");
  return true;
}

// ----------------- les fonctions suivantes inchangées -----------------
bool Tab5Camera::setup_external_clock_() { ... }
bool Tab5Camera::init_ldo_() { ... }
bool Tab5Camera::init_csi_controller_() { ... }
bool Tab5Camera::init_isp_processor_() { ... }
bool Tab5Camera::allocate_frame_buffers_() { ... }
bool Tab5Camera::take_snapshot() { ... }
bool Tab5Camera::start_streaming() { ... }
bool Tab5Camera::stop_streaming() { ... }
void Tab5Camera::streaming_task(void *parameter) { ... }
void Tab5Camera::streaming_loop_() { ... }
bool Tab5Camera::camera_get_finished_trans_callback(
    esp_cam_ctlr_handle_t handle,
    esp_cam_ctlr_trans_t *trans,
    void *user_data) { ... }
void Tab5Camera::process_frame_(uint8_t *data, size_t len) { ... }
void Tab5Camera::trigger_on_frame_callbacks_(uint8_t *data, size_t len) { ... }
void Tab5Camera::deinit_camera_() { ... }
void Tab5Camera::set_error_(const std::string &error) { ... }

}  // namespace tab5_camera
}  // namespace esphome

#endif  // HAS_ESP32_P4_CAMERA
#endif  // USE_ESP32

























