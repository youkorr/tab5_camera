#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

// Inclure le driver SC202CS si disponible
#ifdef CONFIG_CAMERA_SC202CS
  #ifdef __cplusplus
  extern "C" {
  #endif
    #include "sc202cs.h"  // Driver SC202CS de M5Stack
  #ifdef __cplusplus
  }
  #endif
#endif

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

// Registres SC202CS (SC2356)
#define SC202CS_CHIP_ID_H 0x3107
#define SC202CS_CHIP_ID_L 0x3108
#define SC202CS_CHIP_ID_VALUE 0x2311  // ID chip SC2356

#define SC202CS_REG_RESET 0x0103
#define SC202CS_REG_MODE_SELECT 0x0100
#define SC202CS_REG_SOFTWARE_STANDBY 0x0100

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera (SC202CS) for ESP32-P4...");
  
  // Configuration de la pin MCLK (GPIO36)
  if (this->ext_clock_pin_ != nullptr) {
    this->ext_clock_pin_->setup();
    this->ext_clock_pin_->digital_write(false);
  }
  
  // Configuration de la pin RESET si disponible
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);
    delay(10);
  }
  
  // Initialisation de la caméra
  if (!this->init_camera_()) {
    ESP_LOGE(TAG, "Failed to initialize camera");
    this->mark_failed();
    return;
  }
  
  this->initialized_ = true;
  ESP_LOGI(TAG, "Tab5 Camera initialized successfully");
}

void Tab5Camera::loop() {
  if (!this->initialized_ || !this->streaming_) {
    return;
  }
  
  // Gestion du streaming continu si nécessaire
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Sensor: SC202CS (SC2356)");
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->sensor_address_);
  
  if (this->ext_clock_pin_ != nullptr) {
    LOG_PIN("  MCLK Pin: ", this->ext_clock_pin_);
  }
  ESP_LOGCONFIG(TAG, "  MCLK Frequency: %d Hz", this->ext_clock_freq_);
  
  const char *resolution_str[] = {"1080P", "720P", "VGA", "QVGA"};
  ESP_LOGCONFIG(TAG, "  Resolution: %s", resolution_str[this->resolution_]);
  
  const char *format_str[] = {"RGB565", "YUV422", "RAW8", "JPEG"};
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", format_str[this->pixel_format_]);
  
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %d", this->jpeg_quality_);
  
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  ESP_LOGCONFIG(TAG, "  Using ESP-IDF 5.x Camera API (CSI Interface)");
#else
  ESP_LOGCONFIG(TAG, "  Using Legacy ESP Camera API");
#endif
  
  if (this->is_failed()) {
    ESP_LOGE(TAG, "  Setup Failed!");
  }
}

bool Tab5Camera::init_camera_() {
#ifdef USE_ESP32
  ESP_LOGI(TAG, "Initializing ESP32-P4 Camera with new API...");
  
  // Reset du capteur
  this->reset_camera_();
  
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // ========================================
  // Nouvelle API ESP-IDF 5.x pour ESP32-P4
  // ========================================
  
  // 1. Configuration SCCB (I2C pour le capteur)
  // Note: Nous utilisons l'interface I2C d'ESPHome directement
  // et configurons le SCCB manuellement
  
  // 2. Initialisation du capteur SC202CS via I2C
  if (!this->init_sc202cs_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize SC202CS sensor");
    return false;
  }
  
  // 3. Configuration du capteur camera
  esp_cam_sensor_config_t cam_config = {
    .sccb_handle = nullptr,  // À configurer plus tard
    .reset_pin = -1,  // Géré manuellement
    .pwdn_pin = -1,
    .xclk_pin = -1,  // Géré par ESPHome
    .xclk_freq_hz = static_cast<int32_t>(this->ext_clock_freq_),  // Fix narrowing conversion
    .sensor_port = ESP_CAM_SENSOR_MIPI_CSI,
  };
  
  ESP_LOGI(TAG, "Creating SC202CS camera sensor device...");
  
#ifdef CONFIG_CAMERA_SC202CS
  // Utiliser le driver SC202CS de M5Stack
  this->cam_device_ = sc202cs_detect(&cam_config);
  if (this->cam_device_ != nullptr) {
    ESP_LOGI(TAG, "✓ Successfully created SC202CS sensor device");
  } else {
    ESP_LOGE(TAG, "✗ Failed to create SC202CS sensor device");
    return false;
  }
#else
  // Pas de driver SC202CS disponible
  ESP_LOGE(TAG, "");
  ESP_LOGE(TAG, "╔════════════════════════════════════════════════════════════════╗");
  ESP_LOGE(TAG, "║  ERREUR: Driver SC202CS non disponible                        ║");
  ESP_LOGE(TAG, "╠════════════════════════════════════════════════════════════════╣");
  ESP_LOGE(TAG, "║  Pour installer le driver SC202CS:                            ║");
  ESP_LOGE(TAG, "║                                                                ║");
  ESP_LOGE(TAG, "║  Linux/Mac:                                                    ║");
  ESP_LOGE(TAG, "║    cd custom_components/tab5_camera                            ║");
  ESP_LOGE(TAG, "║    chmod +x download_sc202cs_driver.sh                         ║");
  ESP_LOGE(TAG, "║    ./download_sc202cs_driver.sh                                ║");
  ESP_LOGE(TAG, "║                                                                ║");
  ESP_LOGE(TAG, "║  Windows (PowerShell):                                         ║");
  ESP_LOGE(TAG, "║    cd custom_components\\tab5_camera                           ║");
  ESP_LOGE(TAG, "║    .\\download_sc202cs_driver.ps1                              ║");
  ESP_LOGE(TAG, "║                                                                ║");
  ESP_LOGE(TAG, "║  Ou téléchargez manuellement depuis:                          ║");
  ESP_LOGE(TAG, "║  https://github.com/m5stack/M5Tab5-UserDemo                   ║");
  ESP_LOGE(TAG, "║  platforms/tab5/components/esp_cam_sensor/sensors/sc202cs/    ║");
  ESP_LOGE(TAG, "╚════════════════════════════════════════════════════════════════╝");
  ESP_LOGE(TAG, "");
  return false;
#endif
  
  // 4. Configuration du format et de la résolution
  esp_cam_sensor_format_t sensor_format = {};
  
  // Format pixel - utiliser les valeurs correctes de l'enum avec cast explicite
  switch (this->pixel_format_) {
    case CAMERA_RGB565:
      sensor_format.format = static_cast<esp_cam_sensor_output_format_t>(ESP_CAM_SENSOR_PIXFORMAT_RGB565);
      break;
    case CAMERA_YUV422:
      sensor_format.format = static_cast<esp_cam_sensor_output_format_t>(ESP_CAM_SENSOR_PIXFORMAT_YUV422);
      break;
    case CAMERA_RAW8:
      sensor_format.format = static_cast<esp_cam_sensor_output_format_t>(ESP_CAM_SENSOR_PIXFORMAT_RAW8);
      break;
    case CAMERA_JPEG:
      sensor_format.format = static_cast<esp_cam_sensor_output_format_t>(ESP_CAM_SENSOR_PIXFORMAT_JPEG);
      break;
    default:
      sensor_format.format = static_cast<esp_cam_sensor_output_format_t>(ESP_CAM_SENSOR_PIXFORMAT_RGB565);
  }
  
  // Résolution
  switch (this->resolution_) {
    case CAMERA_1080P:
      sensor_format.width = 1920;
      sensor_format.height = 1080;
      break;
    case CAMERA_720P:
      sensor_format.width = 1280;
      sensor_format.height = 720;
      break;
    case CAMERA_VGA:
      sensor_format.width = 640;
      sensor_format.height = 480;
      break;
    case CAMERA_QVGA:
      sensor_format.width = 320;
      sensor_format.height = 240;
      break;
  }
  
  ESP_LOGI(TAG, "Setting camera format: %dx%d", sensor_format.width, sensor_format.height);
  esp_err_t ret = esp_cam_sensor_set_format(this->cam_device_, &sensor_format);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to set camera format: %s", esp_err_to_name(ret));
    return false;
  }
  
  // 5. Démarrage du capteur
  ESP_LOGI(TAG, "Starting camera sensor...");
  int stream_enable = 1;
  ret = esp_cam_sensor_ioctl(this->cam_device_, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_enable);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start camera sensor: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "Camera initialized successfully with new API");
  return true;
  
#else
  // ========================================
  // Ancienne API pour compatibilité
  // ========================================
  ESP_LOGW(TAG, "ESP-IDF < 5.0 detected, using legacy camera API");
  ESP_LOGE(TAG, "Legacy API not supported for ESP32-P4. Please use ESP-IDF 5.x");
  return false;
#endif

#else
  ESP_LOGE(TAG, "Camera only supported on ESP32");
  return false;
#endif
}

bool Tab5Camera::configure_csi_interface_() {
  ESP_LOGI(TAG, "CSI interface is configured automatically by ESP-IDF driver");
  return true;
}

bool Tab5Camera::init_sc202cs_sensor_() {
  ESP_LOGI(TAG, "Initializing SC202CS sensor at address 0x%02X", this->sensor_address_);
  
  delay(50);  // Attendre que le capteur soit prêt
  
  // Lecture du Chip ID pour vérifier la communication
  uint8_t chip_id_h = 0, chip_id_l = 0;
  
  if (!this->read_sensor_reg_(SC202CS_CHIP_ID_H, chip_id_h)) {
    ESP_LOGE(TAG, "Failed to read chip ID high byte");
    return false;
  }
  
  if (!this->read_sensor_reg_(SC202CS_CHIP_ID_L, chip_id_l)) {
    ESP_LOGE(TAG, "Failed to read chip ID low byte");
    return false;
  }
  
  uint16_t chip_id = (chip_id_h << 8) | chip_id_l;
  ESP_LOGI(TAG, "SC202CS Chip ID: 0x%04X (expected: 0x%04X)", chip_id, SC202CS_CHIP_ID_VALUE);
  
  if (chip_id != SC202CS_CHIP_ID_VALUE) {
    ESP_LOGW(TAG, "Chip ID mismatch! Expected 0x%04X but got 0x%04X", SC202CS_CHIP_ID_VALUE, chip_id);
    ESP_LOGW(TAG, "Continuing anyway as some SC202CS variants may have different IDs");
  }
  
  // Reset logiciel du capteur
  ESP_LOGI(TAG, "Performing software reset of SC202CS sensor");
  if (!this->write_sensor_reg_(SC202CS_REG_RESET, 0x01)) {
    ESP_LOGE(TAG, "Failed to reset sensor");
    return false;
  }
  delay(50);  // Attendre la fin du reset
  
  ESP_LOGI(TAG, "SC202CS sensor initialized successfully");
  return true;
}

void Tab5Camera::reset_camera_() {
  if (this->reset_pin_ != nullptr) {
    ESP_LOGI(TAG, "Hardware reset of camera sensor");
    this->reset_pin_->digital_write(false);
    delay(20);
    this->reset_pin_->digital_write(true);
    delay(50);
  } else {
    ESP_LOGI(TAG, "No reset pin configured, skipping hardware reset");
  }
}

bool Tab5Camera::write_sensor_reg_(uint16_t reg, uint8_t value) {
  // SC202CS utilise des adresses de registre 16 bits
  uint8_t data[3] = {
    static_cast<uint8_t>((reg >> 8) & 0xFF),  // Adresse haute
    static_cast<uint8_t>(reg & 0xFF),         // Adresse basse
    value                                      // Valeur
  };
  
  auto err = this->write(data, 3);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to write register 0x%04X = 0x%02X: error %d", reg, value, err);
    return false;
  }
  
  ESP_LOGVV(TAG, "Write register 0x%04X = 0x%02X", reg, value);
  return true;
}

bool Tab5Camera::read_sensor_reg_(uint16_t reg, uint8_t &value) {
  // Écrire l'adresse du registre (16 bits)
  uint8_t reg_addr[2] = {
    static_cast<uint8_t>((reg >> 8) & 0xFF),  // Adresse haute
    static_cast<uint8_t>(reg & 0xFF)          // Adresse basse
  };
  
  // Utiliser write_read pour l'écriture suivie de lecture
  auto err = this->write_read(reg_addr, 2, &value, 1);
  if (err != i2c::ERROR_OK) {
    ESP_LOGE(TAG, "Failed to read register 0x%04X: error %d", reg, err);
    return false;
  }
  
  ESP_LOGVV(TAG, "Read register 0x%04X = 0x%02X", reg, value);
  return true;
}

bool Tab5Camera::take_snapshot() {
#ifdef USE_ESP32
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }
  
  ESP_LOGI(TAG, "Taking snapshot...");
  
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // Note: L'API de frame capture n'est pas encore standardisée dans ESP-IDF 5.x
  // Pour l'instant, nous utilisons l'approche basée sur ioctl
  ESP_LOGI(TAG, "Snapshot capture not yet implemented in ESP-IDF 5.x API");
  return true;
#else
  ESP_LOGE(TAG, "Legacy API not supported");
  return false;
#endif

#else
  ESP_LOGE(TAG, "Camera not supported on this platform");
  return false;
#endif
}

bool Tab5Camera::start_streaming() {
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }
  
  if (this->streaming_) {
    ESP_LOGW(TAG, "Streaming already started");
    return true;
  }
  
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  if (this->cam_device_ != nullptr) {
    int stream_enable = 1;
    esp_err_t ret = esp_cam_sensor_ioctl(this->cam_device_, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_enable);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to start streaming: %s", esp_err_to_name(ret));
      return false;
    }
  }
#endif
  
  this->streaming_ = true;
  ESP_LOGI(TAG, "Camera streaming started");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_) {
    ESP_LOGW(TAG, "Streaming already stopped");
    return true;
  }
  
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  if (this->cam_device_ != nullptr) {
    int stream_enable = 0;
    esp_err_t ret = esp_cam_sensor_ioctl(this->cam_device_, ESP_CAM_SENSOR_IOC_S_STREAM, &stream_enable);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to stop streaming: %s", esp_err_to_name(ret));
      return false;
    }
  }
#endif
  
  this->streaming_ = false;
  ESP_LOGI(TAG, "Camera streaming stopped");
  return true;
}

bool Tab5Camera::get_frame(std::vector<uint8_t> &buffer) {
#ifdef USE_ESP32
  if (!this->initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }
  
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
  // Note: L'API de frame capture n'est pas encore standardisée dans ESP-IDF 5.x
  // Pour l'instant, nous retournons un buffer vide
  ESP_LOGW(TAG, "Frame capture not yet implemented in ESP-IDF 5.x API");
  buffer.clear();
  return true;
#else
  return false;
#endif

#else
  return false;
#endif
}

}  // namespace tab5_camera
}  // namespace esphome
























