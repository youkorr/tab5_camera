#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/automation.h"

#ifdef USE_ESP32

// ESP32-P4 specific includes
#if defined(CONFIG_IDF_TARGET_ESP32P4)
#define HAS_ESP32_P4_CAMERA 1

#include "driver/i2c_master.h"
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h" 
#include "esp_cam_sensor.h"
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Camera sensor includes based on configuration
#ifdef CONFIG_CAMERA_SC2336
#include "sc2336.h"
#define CAM_DEVICE_ADDR SC2336_SCCB_ADDR
#elif CONFIG_CAMERA_OV5645
#include "ov5645.h"
#define CAM_DEVICE_ADDR OV5645_SCCB_ADDR
#else
#define CAM_DEVICE_ADDR 0x30  // Default SC2336 address
#endif

#endif // ESP32-P4

namespace esphome {
namespace tab5_camera {

// Énumérations
enum class PixelFormat { RAW8, RAW10, YUV422, RGB565, JPEG };

class Tab5Camera;

class OnFrameTrigger : public Trigger<uint8_t *, size_t> {
 public:
  explicit OnFrameTrigger(Tab5Camera *parent) : parent_(parent) {}
 protected:
  Tab5Camera *parent_;
};

class Tab5Camera : public Component {
 public:
  Tab5Camera() = default;
  ~Tab5Camera();

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Configuration methods
  void set_sensor_address(uint8_t address) { this->sensor_address_ = address; }
  void set_sccb_scl_pin(uint8_t pin) { this->sccb_scl_pin_ = pin; }
  void set_sccb_sda_pin(uint8_t pin) { this->sccb_sda_pin_ = pin; }
  void set_sccb_frequency(uint32_t freq) { this->sccb_frequency_ = freq; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  void set_resolution(uint16_t width, uint16_t height) {
    this->frame_width_ = width;
    this->frame_height_ = height;
  }
  void set_pixel_format(const std::string &format) { this->pixel_format_ = format; }
  void set_framerate(uint8_t framerate) { this->framerate_ = framerate; }

  // Public operations
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();
  bool is_ready() const;

  void add_on_frame_trigger(OnFrameTrigger *trigger) {
    this->on_frame_triggers_.push_back(trigger);
  }

#ifdef HAS_ESP32_P4_CAMERA
  bool is_streaming() const { return this->streaming_active_; }
  uint8_t *get_frame_buffer() const { return static_cast<uint8_t *>(this->frame_buffer_); }
  size_t get_frame_buffer_size() const { return this->frame_buffer_size_; }
#else
  bool is_streaming() const { return false; }
  uint8_t *get_frame_buffer() const { return nullptr; }
  size_t get_frame_buffer_size() const { return 0; }
#endif

 protected:
#ifdef HAS_ESP32_P4_CAMERA
  struct FrameData {
    void *buffer;
    size_t size;
    uint64_t timestamp;
    bool valid;
  };

  // Core initialization methods - following M5Stack pattern
  bool init_i2c_bus_();
  bool init_sccb_();
  bool detect_camera_sensor_();
  bool init_camera_sensor_();
  bool init_ldo_();
  bool setup_external_clock_();
  bool init_csi_controller_();
  bool init_isp_processor_();
  
  // Camera operations
  bool allocate_frame_buffers_();
  bool start_camera_controller_();
  void deinit_camera_();
  
  // Frame handling
  void process_frame_(uint8_t *data, size_t len);
  void trigger_on_frame_callbacks_(uint8_t *data, size_t len);
  
  // Callback functions for ESP-CAM driver
  static bool IRAM_ATTR camera_get_finished_trans_callback(
      esp_cam_ctlr_handle_t handle,
      esp_cam_ctlr_trans_t *trans,
      void *user_data);

  // Streaming task
  static void streaming_task(void *parameter);
  void streaming_loop_();

  // Hardware handles - following M5Stack structure
  i2c_master_bus_handle_t i2c_bus_handle_{nullptr};
  esp_sccb_io_handle_t sccb_handle_{nullptr};
  esp_cam_sensor_device_t *cam_sensor_{nullptr};
  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  esp_ldo_channel_handle_t ldo_mipi_phy_{nullptr};

  // Frame buffers
  void *frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};
  static constexpr size_t NUM_FRAME_BUFFERS = 3;
  void *frame_buffers_[NUM_FRAME_BUFFERS]{nullptr};

  // State tracking
  bool camera_initialized_{false};
  bool sensor_initialized_{false};
  bool streaming_active_{false};
  bool streaming_should_stop_{false};

  // FreeRTOS objects
  TaskHandle_t streaming_task_handle_{nullptr};
  SemaphoreHandle_t frame_ready_semaphore_{nullptr};
  QueueHandle_t frame_queue_{nullptr};

  // Constants
  static constexpr size_t STREAMING_TASK_STACK_SIZE = 4096;
  static constexpr UBaseType_t STREAMING_TASK_PRIORITY = 5;
  static constexpr size_t FRAME_QUEUE_SIZE = 3;
#endif // HAS_ESP32_P4_CAMERA

 private:
  // Configuration parameters
  uint8_t sensor_address_{CAM_DEVICE_ADDR};
  uint8_t sccb_scl_pin_{21};  // CONFIG_SCCB0_SCL
  uint8_t sccb_sda_pin_{22};  // CONFIG_SCCB0_SDA
  uint32_t sccb_frequency_{400000};  // CONFIG_SCCB0_FREQUENCY
  uint8_t external_clock_pin_{25};
  uint32_t external_clock_frequency_{24000000};
  GPIOPin *reset_pin_{nullptr};

  uint16_t frame_width_{640};
  uint16_t frame_height_{480};
  std::string pixel_format_{"RGB565"};
  uint8_t framerate_{15};

  // State
  std::vector<OnFrameTrigger *> on_frame_triggers_;
  uint32_t frame_count_{0};

  // Helper methods
  void set_error_(const std::string &error);
  std::string last_error_;
  bool error_state_{false};
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32









