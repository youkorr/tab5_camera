#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/i2c/i2c.h"
#include <vector>

#ifdef USE_ESP32
#include "esp_err.h"
#include "esp_log.h"
#include "driver/i2c.h"

// Utiliser la nouvelle API ESP-IDF 5.x pour ESP32-P4
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // Nouveau système de caméra pour ESP-IDF 5.x (ESP32-P4)
  // Inclure les headers ESP-IDF directement
  #include "esp_cam_sensor.h"
  #include "esp_sccb_intf.h"
  
  // Définir les types si nécessaire - utiliser les valeurs correctes de l'enum
  #ifndef ESP_CAM_SENSOR_PIXFORMAT_RGB565
    #define ESP_CAM_SENSOR_PIXFORMAT_RGB565 ((esp_cam_sensor_output_format_t)1)
    #define ESP_CAM_SENSOR_PIXFORMAT_YUV422 ((esp_cam_sensor_output_format_t)2)
    #define ESP_CAM_SENSOR_PIXFORMAT_RAW8 ((esp_cam_sensor_output_format_t)8)
    #define ESP_CAM_SENSOR_PIXFORMAT_JPEG ((esp_cam_sensor_output_format_t)12)
  #endif
#else
  // Ancienne API esp_camera (non supportée sur P4)
  #include "esp_camera.h"
#endif

#endif

namespace esphome {
namespace tab5_camera {

enum CameraResolution {
  CAMERA_1080P = 0,
  CAMERA_720P = 1,
  CAMERA_VGA = 2,
  CAMERA_QVGA = 3,
};

enum CameraPixelFormat {
  CAMERA_RGB565 = 0,
  CAMERA_YUV422 = 1,
  CAMERA_RAW8 = 2,
  CAMERA_JPEG = 3,
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Configuration
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(GPIOPin *pin) { this->ext_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->ext_clock_freq_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_sensor_address(uint8_t addr) { this->sensor_address_ = addr; }
  
  // Enum setters
  void set_resolution(CameraResolution res) { this->resolution_ = res; }
  void set_pixel_format(CameraPixelFormat fmt) { this->pixel_format_ = fmt; }
  
  // Integer setters for YAML compatibility
  void set_resolution(int res) { 
    this->resolution_ = static_cast<CameraResolution>(res); 
  }
  void set_pixel_format(int fmt) { 
    this->pixel_format_ = static_cast<CameraPixelFormat>(fmt); 
  }
  
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_framerate(uint8_t fps) { this->framerate_ = fps; }

  // Méthodes de contrôle
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();
  
  // Récupération d'image
  bool get_frame(std::vector<uint8_t> &buffer);

 protected:
  std::string name_;
  GPIOPin *ext_clock_pin_{nullptr};
  GPIOPin *reset_pin_{nullptr};
  uint32_t ext_clock_freq_{24000000};
  uint8_t sensor_address_{0x36};
  
  CameraResolution resolution_{CAMERA_VGA};
  CameraPixelFormat pixel_format_{CAMERA_RGB565};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{30};
  
  bool streaming_{false};
  bool initialized_{false};

  // Méthodes internes
  bool init_camera_();
  bool init_sc202cs_sensor_();
  bool configure_csi_interface_();
  bool write_sensor_reg_(uint16_t reg, uint8_t value);
  bool read_sensor_reg_(uint16_t reg, uint8_t &value);
  void reset_camera_();
  
#ifdef USE_ESP32
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // Nouvelle API ESP-IDF 5.x pour ESP32-P4
  esp_cam_sensor_device_t *cam_device_{nullptr};
  esp_sccb_io_handle_t sccb_handle_{nullptr};
#else
  // Ancienne API pour compatibilité (non supportée sur P4)
  camera_config_t camera_config_;
#endif
#endif
};

}  // namespace tab5_camera
}  // namespace esphome














