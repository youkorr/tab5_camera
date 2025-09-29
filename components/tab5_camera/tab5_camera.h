#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/preferences.h"
#include "esphome/core/automation.h"
#include "esphome/components/i2c/i2c.h"

#ifdef USE_ESP32

// Vérification spécifique pour ESP32-P4
#if defined(CONFIG_IDF_TARGET_ESP32P4) || (defined(CONFIG_IDF_TARGET) && defined(CONFIG_IDF_TARGET_ESP32P4))
#define HAS_ESP32_P4_CAMERA 1

// Includes ESP32-P4 spécifiques
#include "esp_cam_ctlr_csi.h"
#include "esp_cam_ctlr.h"
#include "driver/isp.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

// Vérification des versions IDF
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)
#define ESP_P4_CAMERA_SUPPORTED 1
#else
#warning "ESP32-P4 camera support requires IDF 5.1 or later"
#undef HAS_ESP32_P4_CAMERA
#endif

#endif // ESP32-P4 check

namespace esphome {
namespace tab5_camera {

// Forward declarations
class Tab5Camera;

// Trigger pour les callbacks de frames
class OnFrameTrigger : public Trigger<uint8_t *, size_t> {
 public:
  explicit OnFrameTrigger(Tab5Camera *parent) : parent_(parent) {}

 protected:
  Tab5Camera *parent_;
};

// Énumérations pour les formats et résolutions
enum class PixelFormat {
  RAW8,
  RAW10,
  YUV422,
  RGB565,
  JPEG
};

enum class CameraResolution {
  QVGA_320x240,
  VGA_640x480,
  HD_720p,
  FHD_1080p
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  Tab5Camera() = default;
  ~Tab5Camera();

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Configuration I2C (héritée de I2CDevice mais avec alias pour clarté)
  void set_sensor_address(uint8_t address) { 
    this->sensor_address_ = address; 
    this->set_i2c_address(address); 
  }

  // Configuration générale
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }

  // Nouveaux paramètres de caméra
  void set_resolution(uint16_t width, uint16_t height) { 
    this->frame_width_ = width; 
    this->frame_height_ = height; 
  }
  void set_pixel_format(const std::string &format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { 
    this->jpeg_quality_ = std::max(static_cast<uint8_t>(1), std::min(quality, static_cast<uint8_t>(63))); 
  }
  void set_framerate(uint8_t framerate) { 
    this->framerate_ = std::max(static_cast<uint8_t>(1), std::min(framerate, static_cast<uint8_t>(60))); 
  }

  // Getters
  const std::string &get_name() const { return this->name_; }
  uint16_t get_frame_width() const { return this->frame_width_; }
  uint16_t get_frame_height() const { return this->frame_height_; }
  const std::string &get_pixel_format() const { return this->pixel_format_; }
  uint8_t get_jpeg_quality() const { return this->jpeg_quality_; }
  uint8_t get_framerate() const { return this->framerate_; }
  uint8_t get_sensor_address() const { return this->sensor_address_; }

  // Fonctions de capture
  bool take_snapshot();
  
  // Fonctions de streaming
  bool start_streaming();
  bool stop_streaming();

  // État et diagnostics
  bool is_ready() const;
  bool has_error() const { return this->error_state_; }
  const std::string &get_last_error() const { return this->last_error_; }

  // Triggers pour l'automation ESPHome
  void add_on_frame_trigger(OnFrameTrigger *trigger) { 
    this->on_frame_triggers_.push_back(trigger); 
  }

#ifdef HAS_ESP32_P4_CAMERA
  bool is_streaming() const { return this->streaming_active_; }
  uint8_t* get_frame_buffer() const { return static_cast<uint8_t*>(this->frame_buffer_); }
  size_t get_frame_buffer_size() const { return this->frame_buffer_size_; }
  
  // Fonctions spécifiques ESP32-P4
  bool init_camera_controller();
  bool init_isp_processor();
  void cleanup_resources();
#else
  bool is_streaming() const { return false; }
  uint8_t* get_frame_buffer() const { return nullptr; }
  size_t get_frame_buffer_size() const { return 0; }
  
  // Stubs pour compatibilité
  bool init_camera_controller() { return false; }
  bool init_isp_processor() { return false; }
  void cleanup_resources() {}
#endif

  // Callbacks pour le streaming
  void add_on_frame_callback(std::function<void(uint8_t*, size_t)> &&callback) {
    this->on_frame_callbacks_.add(std::move(callback));
  }

 protected:
#ifdef HAS_ESP32_P4_CAMERA
  // Structure pour les frames en queue
  struct FrameData {
    void* buffer;
    size_t size;
    uint32_t timestamp;
    bool valid;
  };

  // Fonctions principales d'initialisation
  bool init_camera_();
  bool init_sensor_();
  bool init_ldo_();
  void deinit_camera_();

  // Séquence de reset améliorée
  bool reset_sensor_();
  bool setup_external_clock_();

  // Configuration du capteur SC202CS - fonctions principales
  bool identify_sc202cs_sensor_();
  bool configure_sc202cs_sensor_();
  bool init_sc202cs_sensor_();
  
  // Fonctions de diagnostic I2C pour SC202CS
  bool test_sc202cs_register_access_msb_first();
  bool test_sc202cs_register_access_lsb_first();
  bool configure_sc202cs_sensor_lsb_first();
  bool read_register_16_msb_first(uint16_t reg, uint8_t *val);
  bool read_register_16_lsb_first(uint16_t reg, uint8_t *val);
  
  // Tests et diagnostics
  bool test_manual_capture_();
  bool start_continuous_capture_();
  
  // Debug et diagnostic
  void debug_camera_status_();
  void verify_external_clock_();
  
  // Communication I2C avec le capteur
  bool read_sensor_register_(uint16_t reg, uint8_t *value);
  bool write_sensor_register_(uint16_t reg, uint8_t value);
  
  // Callbacks statiques pour le contrôleur de caméra
  static bool IRAM_ATTR camera_get_new_vb_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  static bool IRAM_ATTR camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
  
  // Tâche de streaming
  static void streaming_task(void *parameter);
  void streaming_loop_();
  
  // Traitement des frames reçues
  void process_frame_(uint8_t* data, size_t len);
  void trigger_on_frame_callbacks_(uint8_t* data, size_t len);
  
  // Handles ESP32-P4 spécifiques
  esp_cam_ctlr_handle_t cam_handle_{nullptr};
  isp_proc_handle_t isp_proc_{nullptr};
  esp_ldo_channel_handle_t ldo_mipi_phy_{nullptr};
  void *frame_buffer_{nullptr};
  size_t frame_buffer_size_{0};
  
  // Buffers multiples pour capture continue
  static constexpr size_t NUM_FRAME_BUFFERS = 3;
  void *frame_buffers_[NUM_FRAME_BUFFERS]{nullptr};
  size_t current_buffer_index_{0};
  
  // États d'initialisation
  bool camera_initialized_{false};
  bool sensor_initialized_{false};
  bool ldo_initialized_{false};
  
  // Variables de streaming
  TaskHandle_t streaming_task_handle_{nullptr};
  SemaphoreHandle_t frame_ready_semaphore_{nullptr};
  QueueHandle_t frame_queue_{nullptr};
  bool streaming_active_{false};
  bool streaming_should_stop_{false};
  bool continuous_capture_active_{false};
  
  // Configuration des buffers et tâches
  static constexpr size_t FRAME_QUEUE_SIZE = 3;
  static constexpr uint32_t STREAMING_TASK_STACK_SIZE = 4096;
  static constexpr UBaseType_t STREAMING_TASK_PRIORITY = 5;
#endif

 private:
  // Configuration générale ESP32-P4CAM
  std::string name_{"Tab5 Camera ESP32-P4CAM"};
  uint8_t external_clock_pin_{36};  // GPIO36 = CAM_MCLK sur ESP32-P4CAM
  uint32_t external_clock_frequency_{24000000};  // 24MHz pour SC202CS
  uint8_t sensor_address_{0x36};  // Adresse SCCB du capteur SC202CS
  GPIOPin *reset_pin_{nullptr};

  // Paramètres de caméra pour SC202CS (720p par défaut)
  uint16_t frame_width_{1280};   // SC202CS 720p
  uint16_t frame_height_{720};   // SC202CS 720p
  std::string pixel_format_{"RGB565"};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{30};  // 30fps optimal pour SC202CS

  // Méthodes pour registres 16-bit (SC202CS)
  bool write_register_16(uint16_t reg, uint8_t val);
  bool read_register_16(uint16_t reg, uint8_t *val);

  // Gestion des erreurs
  bool error_state_{false};
  std::string last_error_{""};
  
  // Callbacks et triggers
  CallbackManager<void(uint8_t*, size_t)> on_frame_callbacks_;
  std::vector<OnFrameTrigger *> on_frame_triggers_;
  
  // Méthodes utilitaires privées
  void set_error_(const std::string &error);
  void clear_error_();
  PixelFormat parse_pixel_format_(const std::string &format) const;
  size_t calculate_frame_size_() const;
  
  // Statistiques pour diagnostics
  uint32_t frame_count_{0};
  uint32_t error_count_{0};
  uint32_t last_frame_timestamp_{0};
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32




