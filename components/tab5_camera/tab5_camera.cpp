#include "tab5_camera.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

Tab5Camera::~Tab5Camera() {
  this->deinit_camera_();
}

void Tab5Camera::setup() {
#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGI(TAG, "Initializing Tab5Camera...");

  if (!this->init_i2c_bus_()) {
    this->set_error_("Failed to init I2C bus");
    return;
  }

  if (!this->detect_camera_sensor_()) {
    this->set_error_("No camera sensor detected");
    return;
  }
  if (!this->init_camera_sensor_()) {
    this->set_error_("Failed to init camera sensor");
    return;
  }
  if (!this->init_ldo_()) {
    this->set_error_("Failed to init LDO");
    return;
  }
  if (!this->setup_external_clock_()) {
    this->set_error_("Failed to setup external clock");
    return;
  }
  if (!this->init_csi_controller_()) {
    this->set_error_("Failed to init CSI controller");
    return;
  }
  if (!this->init_isp_processor_()) {
    this->set_error_("Failed to init ISP processor");
    return;
  }
  if (!this->allocate_frame_buffers_()) {
    this->set_error_("Failed to allocate frame buffers");
    return;
  }
  if (!this->start_camera_controller_()) {
    this->set_error_("Failed to start camera controller");
    return;
  }

  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Tab5Camera initialized successfully.");
#else
  ESP_LOGE(TAG, "This platform does not support ESP32-P4 camera");
#endif
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Pixel format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  Ready: %s", this->is_ready() ? "YES" : "NO");
}

float Tab5Camera::get_setup_priority() const { return setup_priority::HARDWARE; }

bool Tab5Camera::is_ready() const {
  return this->camera_initialized_ && this->sensor_initialized_;
}

// ======================================================
// I2C / SCCB
// ======================================================

bool Tab5Camera::init_i2c_bus_() {
  ESP_LOGI(TAG, "Initializing I2C master bus");

  i2c_master_bus_config_t i2c_bus_config = {};
  i2c_bus_config.clk_source        = I2C_CLK_SRC_DEFAULT;
  i2c_bus_config.i2c_port          = SCCB0_PORT_NUM;
  i2c_bus_config.scl_io_num        = (gpio_num_t) this->sccb_scl_pin_;
  i2c_bus_config.sda_io_num        = (gpio_num_t) this->sccb_sda_pin_;
  i2c_bus_config.glitch_ignore_cnt = 7;
  i2c_bus_config.intr_priority     = 0;
  i2c_bus_config.trans_queue_depth = 0;
  i2c_bus_config.flags.enable_internal_pullup = true;

  esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &this->i2c_bus_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
    return false;
  }
  return true;
}



bool Tab5Camera::detect_camera_sensor_() {
  ESP_LOGI(TAG, "Detecting camera sensor...");

  esp_cam_sensor_config_t cam_config = {};
 
  cam_config.reset_pin   = (this->reset_pin_) ? this->reset_pin_->get_pin() : -1;

#ifdef CONFIG_CAMERA_SC2356
  this->cam_sensor_ = sc202cs_detect(&cam_config);
#elif CONFIG_CAMERA_OV5645
  this->cam_sensor_ = ov5645_detect(&cam_config);
#else
  ESP_LOGW(TAG, "No known camera sensor selected at compile time.");
  this->cam_sensor_ = nullptr;
#endif

  if (!this->cam_sensor_) {
    ESP_LOGE(TAG, "Camera sensor detection failed");
    return false;
  }

  this->sensor_initialized_ = true;
  ESP_LOGI(TAG, "Camera sensor detected successfully.");
  return true;
}

// ======================================================
// Helpers
// ======================================================

PixelFormat Tab5Camera::parse_pixel_format_(const std::string &format) const {
  if (format == "RAW8") return PixelFormat::RAW8;
  if (format == "RAW10") return PixelFormat::RAW10;
  if (format == "YUV422") return PixelFormat::YUV422;
  if (format == "RGB565") return PixelFormat::RGB565;
  if (format == "JPEG") return PixelFormat::JPEG;
  return PixelFormat::RGB565;
}

size_t Tab5Camera::calculate_frame_size_() const {
  PixelFormat fmt = this->parse_pixel_format_(this->pixel_format_);
  switch (fmt) {
    case PixelFormat::RAW8:
      return this->frame_width_ * this->frame_height_;
    case PixelFormat::RAW10:
      return this->frame_width_ * this->frame_height_ * 10 / 8;
    case PixelFormat::YUV422:
    case PixelFormat::RGB565:
      return this->frame_width_ * this->frame_height_ * 2;
    case PixelFormat::JPEG:
      return this->frame_width_ * this->frame_height_ / 4;  // estimation
    default:
      return this->frame_width_ * this->frame_height_ * 2;
  }
}

void Tab5Camera::set_error_(const std::string &error) {
  ESP_LOGE(TAG, "%s", error.c_str());
  this->last_error_ = error;
  this->error_state_ = true;
}

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32
















