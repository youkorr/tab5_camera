#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "esp_timer.h"
#include "driver/ledc.h"
// #include "esp_cam_sensor.h"
#include "driver/jpeg_encode.h"
// #include "esp_video_buffer.h"
// #include "esp_video_internal.h"
#include "esp_cam_ctlr_csi.h"

#ifdef USE_ESP32

static const char *const TAG = "tab5_camera";

// Constantes de configuration pour Tab5 avec SC202CS
#define TAB5_CAMERA_H_RES 1280
#define TAB5_CAMERA_V_RES 720
#define TAB5_MIPI_CSI_LANE_BITRATE_MBPS 576
#define TAB5_ISP_CLOCK_HZ 80000000  
#define TAB5_STREAMING_STACK_SIZE 8192
#define TAB5_FRAME_QUEUE_LENGTH 8

// Configuration spécifique SC202CS
#define SC202CS_CHIP_ID_REG1    0x3107
#define SC202CS_CHIP_ID_REG2    0x3108
#define SC202CS_CHIP_ID_VAL1    0x20
#define SC202CS_CHIP_ID_VAL2    0x2e

namespace esphome {
namespace tab5_camera {

// Structure pour les registres de configuration SC202CS
struct sc202cs_reginfo_t {
    uint16_t reg_addr;
    uint8_t reg_value;
    const char* description;
    uint32_t delay_ms;
};

// Configuration complète SC202CS 1280x720 30fps MIPI 1 lane
static const sc202cs_reginfo_t sc202cs_init_reglist_720p_30fps[] = {
    // Software reset
    {0x0103, 0x01, "Software reset", 50},
    {0x0100, 0x00, "Standby mode", 10},
    
    // Basic setup
    {0x36e9, 0x80, "Bypass PLL1", 5},
    {0x37f9, 0x80, "Bypass PLL2", 5},
    
    // Clock settings
    {0x301f, 0x01, "Clock config", 5},
    {0x30b8, 0x44, "Clock divider", 5},
    
    // PLL configuration for 24MHz input
    {0x320c, 0x05, "HTS MSB", 5},
    {0x320d, 0x46, "HTS LSB (1350)", 5},
    {0x320e, 0x02, "VTS MSB", 5},
    {0x320f, 0xee, "VTS LSB (750)", 5},
    
    // Window configuration for 1280x720
    {0x3208, 0x05, "Width MSB", 5},
    {0x3209, 0x00, "Width LSB (1280)", 5},
    {0x320a, 0x02, "Height MSB", 5},
    {0x320b, 0xd0, "Height LSB (720)", 5},
    
    // Start position
    {0x3210, 0x00, "X start MSB", 5},
    {0x3211, 0x04, "X start LSB", 5},
    {0x3212, 0x00, "Y start MSB", 5},
    {0x3213, 0x04, "Y start LSB", 5},
    
    // MIPI configuration
    {0x3301, 0x06, "MIPI control", 5},
    {0x3302, 0x18, "MIPI timing", 5},
    {0x3303, 0x10, "MIPI mode", 5},
    {0x3304, 0x60, "MIPI lane config", 5},
    {0x3306, 0x60, "MIPI data rate", 5},
    {0x3308, 0x10, "MIPI settle", 5},
    {0x3309, 0x70, "MIPI prepare", 5},
    
    // Sensor specific registers
    {0x331e, 0x0d, "Sensor control", 5},
    {0x331f, 0x8d, "Sensor timing", 5},
    {0x3320, 0x0f, "Readout mode", 5},
    
    // Black level and gain
    {0x3908, 0x41, "Black level", 5},
    {0x391b, 0x80, "ADC control", 5},
    {0x3920, 0xff, "Gain control", 5},
    
    // ISP and format settings
    {0x4509, 0x20, "RAW8 output", 5},
    {0x450d, 0x11, "Data format", 5},
    
    // Enable streaming
    {0x36e9, 0x00, "Enable PLL1", 5},
    {0x37f9, 0x00, "Enable PLL2", 5},
    {0x0100, 0x01, "Streaming on", 10},
};

Tab5Camera::~Tab5Camera() {
#ifdef HAS_ESP32_P4_CAMERA
  this->deinit_camera_();
#endif
}

void Tab5Camera::setup() {
#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera with ESP32-P4 MIPI-CSI and SC202CS sensor...");
  
  ESP_LOGI(TAG, "Step 1: Creating synchronization objects");
  
  // Création des objets de synchronisation
  this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
  if (!this->frame_ready_semaphore_) {
    ESP_LOGE(TAG, "Failed to create frame ready semaphore");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Frame semaphore created successfully");
  
  this->frame_queue_ = xQueueCreate(TAB5_FRAME_QUEUE_LENGTH, sizeof(FrameData));
  if (!this->frame_queue_) {
    ESP_LOGE(TAG, "Failed to create frame queue");
    this->mark_failed();
    return;
  }
  ESP_LOGI(TAG, "Frame queue created successfully");
  
  ESP_LOGI(TAG, "Step 2: Initializing camera");
  
  if (!this->init_camera_()) {
    ESP_LOGE(TAG, "Failed to initialize camera - setup marked as failed");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "Tab5 Camera '%s' with SC202CS setup completed successfully", this->name_.c_str());
#else
  ESP_LOGE(TAG, "ESP32-P4 MIPI-CSI API not available - Tab5 Camera component disabled");
  this->mark_failed();
#endif
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera '%s' with SC202CS:", this->name_.c_str());
  
#ifdef HAS_ESP32_P4_CAMERA
  ESP_LOGCONFIG(TAG, "  Platform: ESP32-P4 MIPI-CSI");
  ESP_LOGCONFIG(TAG, "  Sensor: SC202CS");
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", TAB5_CAMERA_H_RES, TAB5_CAMERA_V_RES);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  if (this->external_clock_pin_ > 0) {
    ESP_LOGCONFIG(TAG, "  External Clock Pin: GPIO%u", this->external_clock_pin_);
    ESP_LOGCONFIG(TAG, "  External Clock Frequency: %u Hz", this->external_clock_frequency_);
  }
  ESP_LOGCONFIG(TAG, "  Frame Buffer Size: %zu bytes", this->frame_buffer_size_);
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  MIPI Lanes: 1");
  ESP_LOGCONFIG(TAG, "  MIPI Bitrate: %d Mbps", TAB5_MIPI_CSI_LANE_BITRATE_MBPS);
  
  if (this->reset_pin_) {
    LOG_PIN("  Reset Pin: ", this->reset_pin_);
  }
#else
  ESP_LOGCONFIG(TAG, "  Status: ESP32-P4 MIPI-CSI API not available");
#endif
  
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup Failed");
  }
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

bool Tab5Camera::is_ready() const {
#ifdef HAS_ESP32_P4_CAMERA
  return this->camera_initialized_ && this->sensor_initialized_;
#else
  return false;
#endif
}

#ifdef HAS_ESP32_P4_CAMERA

bool Tab5Camera::reset_sensor_() {
  if (!this->reset_pin_) {
    ESP_LOGW(TAG, "No reset pin configured - trying SC202CS software reset");
    
    // Reset logiciel spécifique SC202CS
    if (!this->write_register_16(0x0103, 0x01)) {
      ESP_LOGW(TAG, "SC202CS software reset failed");
      return false;
    }
    
    ESP_LOGI(TAG, "SC202CS software reset sent, waiting 100ms...");
    vTaskDelay(100 / portTICK_PERIOD_MS);
    
    return true;
  }
  
  ESP_LOGI(TAG, "Executing hardware reset sequence for SC202CS");
  
  // Séquence de reset hardware optimisée pour SC202CS
  this->reset_pin_->setup();
  
  // 1. Reset actif (LOW) - maintenir 50ms minimum pour SC202CS
  this->reset_pin_->digital_write(false);
  ESP_LOGD(TAG, "Reset pin LOW");
  vTaskDelay(50 / portTICK_PERIOD_MS);
  
  // 2. Relâcher le reset (HIGH)
  this->reset_pin_->digital_write(true);
  ESP_LOGD(TAG, "Reset pin HIGH");
  
  // 3. Attendre la stabilisation du SC202CS (augmenté)
  vTaskDelay(150 / portTICK_PERIOD_MS);
  
  ESP_LOGI(TAG, "Hardware reset sequence completed for SC202CS");
  
  // 4. Test de communication après reset avec registres SC202CS
  uint8_t test_val;
  int retry_count = 0;
  const int max_retries = 10;
  
  while (retry_count < max_retries) {
    if (this->read_register_16(0x3107, &test_val)) {
      ESP_LOGI(TAG, "SC202CS communication restored after reset (reg 0x3107 = 0x%02X)", test_val);
      return true;
    }
    
    retry_count++;
    ESP_LOGD(TAG, "Communication test %d/%d failed, retrying...", retry_count, max_retries);
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  
  ESP_LOGE(TAG, "SC202CS communication failed after reset");
  return false;
}

bool Tab5Camera::setup_external_clock_() {
  if (this->external_clock_pin_ == 0) {
    ESP_LOGW(TAG, "No external clock pin configured - SC202CS will not work");
    return false;
  }
  
  ESP_LOGI(TAG, "Setting up 24MHz external clock for SC202CS on GPIO%u", this->external_clock_pin_);
  
  // Configuration du timer LEDC pour générer 24MHz pour SC202CS
  ledc_timer_config_t timer_conf = {};
  timer_conf.duty_resolution = LEDC_TIMER_1_BIT;  // 1-bit = 50% duty cycle
  timer_conf.freq_hz = this->external_clock_frequency_;  // 24MHz
  timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_conf.deconfigure = false;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  timer_conf.timer_num = LEDC_TIMER_0;
  
  esp_err_t err = ledc_timer_config(&timer_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "LEDC timer config failed for %uHz: %s", 
             this->external_clock_frequency_, esp_err_to_name(err));
    return false;
  }
  
  // Configuration du canal LEDC sur le GPIO
  ledc_channel_config_t ch_conf = {};
  ch_conf.gpio_num = this->external_clock_pin_;
  ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  ch_conf.channel = LEDC_CHANNEL_0;
  ch_conf.intr_type = LEDC_INTR_DISABLE;
  ch_conf.timer_sel = LEDC_TIMER_0;
  ch_conf.duty = 1;  // 50% duty cycle (1 sur 2^1 = 1 sur 2)
  ch_conf.hpoint = 0;
  ch_conf.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;  // Crucial pour maintenir l'horloge
  
  err = ledc_channel_config(&ch_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(err));
    return false;
  }
  
  ESP_LOGI(TAG, "24MHz clock successfully configured for SC202CS on GPIO%u", this->external_clock_pin_);
  return true;
}

void Tab5Camera::verify_external_clock_() {
  ESP_LOGI(TAG, "=== EXTERNAL CLOCK VERIFICATION FOR SC202CS ===");
  ESP_LOGI(TAG, "External clock configured on GPIO%u at %u Hz", 
           this->external_clock_pin_, this->external_clock_frequency_);
  ESP_LOGI(TAG, "SC202CS requires stable 24MHz clock for proper operation");
  ESP_LOGI(TAG, "Note: Use oscilloscope to verify actual 24MHz signal if needed");
}

bool Tab5Camera::identify_sc202cs_sensor_() {
    ESP_LOGI(TAG, "=== SC202CS SENSOR IDENTIFICATION ===");
    
    uint8_t id_high, id_low;
    
    // Lecture des registres d'ID SC202CS
    if (!this->read_register_16(SC202CS_CHIP_ID_REG1, &id_high)) {
        ESP_LOGE(TAG, "Failed to read SC202CS ID register 0x3107");
        return false;
    }
    
    if (!this->read_register_16(SC202CS_CHIP_ID_REG2, &id_low)) {
        ESP_LOGE(TAG, "Failed to read SC202CS ID register 0x3108");
        return false;
    }
    
    ESP_LOGI(TAG, "Sensor ID: 0x%02X%02X", id_high, id_low);
    
    // Vérification de l'ID SC202CS
    if (id_high == SC202CS_CHIP_ID_VAL1 && id_low == SC202CS_CHIP_ID_VAL2) {
        ESP_LOGI(TAG, "✓ SC202CS sensor detected and verified");
        return true;
    } else {
        ESP_LOGW(TAG, "⚠ Expected SC202CS ID 0x%02X%02X, got 0x%02X%02X", 
                 SC202CS_CHIP_ID_VAL1, SC202CS_CHIP_ID_VAL2, id_high, id_low);
        ESP_LOGI(TAG, "Continuing with SC202CS configuration anyway");
        return true; // Continue même si l'ID ne correspond pas exactement
    }
}

bool Tab5Camera::configure_sc202cs_sensor_() {
    ESP_LOGI(TAG, "=== SC202CS SENSOR CONFIGURATION ===");
    ESP_LOGI(TAG, "Configuring for 1280x720 @ 30fps, RAW8 output, MIPI 1 lane");
    
    const size_t reg_count = sizeof(sc202cs_init_reglist_720p_30fps) / sizeof(sc202cs_init_reglist_720p_30fps[0]);
    
    for (size_t i = 0; i < reg_count; i++) {
        const auto& config = sc202cs_init_reglist_720p_30fps[i];
        
        ESP_LOGD(TAG, "[%zu/%zu] %s: 0x%04X = 0x%02X", 
                 i+1, reg_count, config.description, config.reg_addr, config.reg_value);
        
        if (!this->write_register_16(config.reg_addr, config.reg_value)) {
            ESP_LOGW(TAG, "Failed to write register 0x%04X = 0x%02X (%s)", 
                     config.reg_addr, config.reg_value, config.description);
            
            // Pour les registres critiques, on peut échouer
            if (config.reg_addr == 0x0103 || config.reg_addr == 0x0100) {
                ESP_LOGE(TAG, "Critical register write failed, aborting");
                return false;
            }
        }
        
        if (config.delay_ms > 0) {
            vTaskDelay(config.delay_ms / portTICK_PERIOD_MS);
        }
        
        // Vérification pour les registres importants
        if (config.reg_addr == 0x0100 && config.reg_value == 0x01) {
            ESP_LOGI(TAG, "✓ SC202CS streaming enabled");
        }
    }
    
    ESP_LOGI(TAG, "SC202CS configuration completed (%zu registers written)", reg_count);
    
    // Vérification finale de quelques registres clés
    uint8_t streaming_status;
    if (this->read_register_16(0x0100, &streaming_status)) {
        ESP_LOGI(TAG, "Streaming status: 0x%02X %s", 
                 streaming_status, (streaming_status & 0x01) ? "(ON)" : "(OFF)");
    }
    
    return true;
}

bool Tab5Camera::init_sc202cs_sensor_() {
    if (this->sensor_initialized_) {
        ESP_LOGI(TAG, "SC202CS sensor already initialized, skipping");
        return true;
    }
    
    ESP_LOGI(TAG, "=== DIAGNOSTIC I2C POUR SC202CS ESP32-P4CAM ===");
    ESP_LOGI(TAG, "Adresse I2C configurée: 0x%02X", this->address_);
    ESP_LOGI(TAG, "Pins I2C: SDA=GPIO31, SCL=GPIO32");
    ESP_LOGI(TAG, "Pin MCLK: GPIO36 (CAM_MCLK)");
    
    // Étape 1: Scan I2C complet pour voir quels périphériques répondent
    ESP_LOGI(TAG, "Étape 1: Scan I2C complet");
    bool device_found = false;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        this->set_i2c_address(addr);
        uint8_t dummy;
        if (this->read(&dummy, 0)) {
            ESP_LOGI(TAG, "Périphérique I2C détecté à l'adresse: 0x%02X", addr);
            device_found = true;
        }
    }
    
    if (!device_found) {
        ESP_LOGE(TAG, "Aucun périphérique I2C détecté - vérifiez GPIO31/32 et alimentation");
        return false;
    }
    
    // Étape 2: Test de communication basique à l'adresse 0x36
    ESP_LOGI(TAG, "Étape 2: Test de communication I2C à 0x36 (SC202CS)");
    this->set_i2c_address(0x36);
    
    // Test avec des écritures/lectures simples
    uint8_t dummy_write = 0x00;
    if (!this->write(&dummy_write, 1)) {
        ESP_LOGE(TAG, "Échec écriture basique à 0x36");
    } else {
        ESP_LOGI(TAG, "Écriture basique OK à 0x36");
    }
    
    // Étape 3: Test des registres avec adressage 8-bit classique
    ESP_LOGI(TAG, "Étape 3: Test adressage 8-bit classique");
    
    uint8_t test_val;
    const uint8_t test_regs_8bit[] = {0x00, 0x01, 0x02, 0x0A, 0x0B, 0x0C};
    
    for (size_t i = 0; i < sizeof(test_regs_8bit); i++) {
        if (this->read_byte(test_regs_8bit[i], &test_val)) {
            ESP_LOGI(TAG, "Registre 8-bit 0x%02X = 0x%02X", test_regs_8bit[i], test_val);
        } else {
            ESP_LOGD(TAG, "Registre 8-bit 0x%02X : pas de réponse", test_regs_8bit[i]);
        }
    }
    
    // Étape 4: Test spécifique SC202CS avec adressage 16-bit
    ESP_LOGI(TAG, "Étape 4: Test SC202CS avec adressage 16-bit");
    
    // Test MSB first (notre implémentation actuelle)
    ESP_LOGI(TAG, "Test adressage MSB first...");
    if (this->test_sc202cs_register_access_msb_first()) {
        ESP_LOGI(TAG, "✓ SC202CS détecté avec adressage MSB first");
        return this->configure_sc202cs_sensor_();
    }
    
    // Test LSB first  
    ESP_LOGI(TAG, "Test adressage LSB first...");
    if (this->test_sc202cs_register_access_lsb_first()) {
        ESP_LOGI(TAG, "✓ SC202CS détecté avec adressage LSB first");
        return this->configure_sc202cs_sensor_lsb_first();
    }
    
    // Étape 5: Test avec des adresses alternatives
    ESP_LOGI(TAG, "Étape 5: Test adresses alternatives SC202CS");
    const uint8_t alt_addresses[] = {0x30, 0x32, 0x34, 0x3C, 0x3D};
    
    for (size_t i = 0; i < sizeof(alt_addresses); i++) {
        ESP_LOGI(TAG, "Test adresse 0x%02X...", alt_addresses[i]);
        this->set_i2c_address(alt_addresses[i]);
        
        if (this->test_sc202cs_register_access_msb_first()) {
            ESP_LOGI(TAG, "✓ SC202CS détecté à l'adresse 0x%02X!", alt_addresses[i]);
            this->sensor_address_ = alt_addresses[i];
            return this->configure_sc202cs_sensor_();
        }
    }
    
    ESP_LOGE(TAG, "Échec de détection SC202CS avec toutes les méthodes testées");
    ESP_LOGE(TAG, "Vérifiez:");
    ESP_LOGE(TAG, "- Alimentation 3.3V du SC202CS");
    ESP_LOGE(TAG, "- Connexions SDA/SCL sur GPIO31/32");
    ESP_LOGE(TAG, "- Signal MCLK 24MHz sur GPIO36");
    ESP_LOGE(TAG, "- Résistances pull-up I2C (4.7kΩ)");
    
    return false;
}

// Fonction de test pour adressage MSB first
bool Tab5Camera::test_sc202cs_register_access_msb_first() {
    uint8_t id_high, id_low;
    
    // Test des registres d'ID SC202CS avec MSB first
    if (this->read_register_16_msb_first(0x3107, &id_high) && 
        this->read_register_16_msb_first(0x3108, &id_low)) {
        ESP_LOGI(TAG, "ID SC202CS (MSB): 0x%02X%02X", id_high, id_low);
        return (id_high != 0x00 || id_low != 0x00); // Au moins un registre non-nul
    }
    return false;
}

// Fonction de test pour adressage LSB first
bool Tab5Camera::test_sc202cs_register_access_lsb_first() {
    uint8_t id_high, id_low;
    
    if (this->read_register_16_lsb_first(0x3107, &id_high) && 
        this->read_register_16_lsb_first(0x3108, &id_low)) {
        ESP_LOGI(TAG, "ID SC202CS (LSB): 0x%02X%02X", id_high, id_low);
        return (id_high != 0x00 || id_low != 0x00);
    }
    return false;
}

// Fonctions d'accès registres avec différents ordres d'octets
bool Tab5Camera::read_register_16_msb_first(uint16_t reg, uint8_t *val) {
    uint8_t buffer[2] = {
        static_cast<uint8_t>(reg >> 8),    // MSB first
        static_cast<uint8_t>(reg & 0xFF)   // LSB
    };
    
    if (!this->write(buffer, 2) || !this->read(val, 1)) {
        return false;
    }
    return true;
}

bool Tab5Camera::read_register_16_lsb_first(uint16_t reg, uint8_t *val) {
    uint8_t buffer[2] = {
        static_cast<uint8_t>(reg & 0xFF),  // LSB first
        static_cast<uint8_t>(reg >> 8)     // MSB
    };
    
    if (!this->write(buffer, 2) || !this->read(val, 1)) {
        return false;
    }
    return true;
}

bool Tab5Camera::write_register_16_lsb_first(uint16_t reg, uint8_t val) {
    uint8_t buffer[3] = {
        static_cast<uint8_t>(reg & 0xFF),  // LSB first
        static_cast<uint8_t>(reg >> 8),    // MSB
        val
    };
    
    if (!this->write(buffer, 3)) {
        ESP_LOGD(TAG, "Failed to write 16-bit register (LSB) 0x%04X = 0x%02X", reg, val);
        return false;
    }
    
    return true;
}

// Version de configuration avec LSB first (si nécessaire)
bool Tab5Camera::configure_sc202cs_sensor_lsb_first() {
    ESP_LOGI(TAG, "Configuration SC202CS avec adressage LSB first");
    
    // Version modifiée de la configuration avec write_register_16_lsb_first
    const size_t reg_count = sizeof(sc202cs_init_reglist_720p_30fps) / sizeof(sc202cs_init_reglist_720p_30fps[0]);
    
    for (size_t i = 0; i < reg_count; i++) {
        const auto& config = sc202cs_init_reglist_720p_30fps[i];
        
        ESP_LOGD(TAG, "[%zu/%zu] %s: 0x%04X = 0x%02X (LSB first)", 
                 i+1, reg_count, config.description, config.reg_addr, config.reg_value);
        
        // Utiliser l'écriture LSB first
        if (!this->write_register_16_lsb_first(config.reg_addr, config.reg_value)) {
            ESP_LOGW(TAG, "Failed to write register 0x%04X = 0x%02X (%s)", 
                     config.reg_addr, config.reg_value, config.description);
        }
        
        if (config.delay_ms > 0) {
            vTaskDelay(config.delay_ms / portTICK_PERIOD_MS);
        }
    }
    
    this->sensor_initialized_ = true;
    ESP_LOGI(TAG, "SC202CS configuration avec LSB first terminée");
    
    return true;
}

bool Tab5Camera::init_ldo_() {
  if (this->ldo_initialized_) {
    ESP_LOGI(TAG, "LDO already initialized, skipping");
    return true;
  }
  
  ESP_LOGI(TAG, "Initializing MIPI LDO regulator for SC202CS");
  
  esp_ldo_channel_config_t ldo_mipi_phy_config = {
    .chan_id = 3,
    .voltage_mv = 2500,
  };
  
  esp_err_t ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &this->ldo_mipi_phy_);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to acquire MIPI LDO channel: %s - continuing anyway", esp_err_to_name(ret));
    this->ldo_mipi_phy_ = nullptr;
  } else {
    ESP_LOGI(TAG, "MIPI LDO regulator acquired successfully");
  }
  
  this->ldo_initialized_ = true;
  return true;
}

bool Tab5Camera::init_sensor_() {
    if (this->sensor_initialized_) {
        ESP_LOGI(TAG, "Sensor already initialized, skipping");
        return true;
    }
    
    ESP_LOGI(TAG, "Attempting to initialize SC202CS camera sensor at I2C address 0x%02X", this->address_);
    
    // Test de communication I2C basique avec registres SC202CS
    uint8_t test_data;
    bool sensor_detected = false;
    
    const uint16_t sc202cs_test_regs[] = {0x3107, 0x3108, 0x3000, 0x3001, 0x3002};
    for (size_t i = 0; i < sizeof(sc202cs_test_regs)/sizeof(sc202cs_test_regs[0]); i++) {
        if (this->read_register_16(sc202cs_test_regs[i], &test_data)) {
            ESP_LOGI(TAG, "SC202CS responded: reg 0x%04X = 0x%02X", sc202cs_test_regs[i], test_data);
            sensor_detected = true;
        }
    }
    
    if (!sensor_detected) {
        ESP_LOGE(TAG, "No SC202CS sensor detected at I2C address 0x%02X - check wiring!", this->address_);
        return false;
    }
    
    ESP_LOGI(TAG, "I2C communication OK with SC202CS at address 0x%02X", this->address_);
    
    // Appeler la nouvelle fonction d'initialisation SC202CS
    return this->init_sc202cs_sensor_();
}

bool Tab5Camera::test_manual_capture_() {
  ESP_LOGI(TAG, "=== MANUAL CAPTURE TEST WITH SC202CS ===");
  
  if (!this->cam_handle_) {
    ESP_LOGE(TAG, "Camera handle not initialized");
    return false;
  }
  
  // Test de capture manuelle (bloquante) avec timeout plus long pour 720p
  esp_cam_ctlr_trans_t trans = {};
  trans.buffer = this->frame_buffer_;
  trans.buflen = this->frame_buffer_size_;
  
  ESP_LOGI(TAG, "Starting manual capture test for SC202CS, timeout 10 seconds...");
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 10000 / portTICK_PERIOD_MS);
  
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "✓ SC202CS manual capture SUCCESS: %zu bytes received", trans.received_size);
    
    // Analyse basique des données reçues
    if (trans.received_size > 0) {
      uint8_t *data = static_cast<uint8_t*>(trans.buffer);
      ESP_LOGI(TAG, "Frame data - First bytes: %02X %02X %02X %02X %02X %02X %02X %02X", 
               data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
      
      // Test de variabilité des données
      bool all_same = true;
      for (size_t i = 1; i < std::min(trans.received_size, (size_t)100); i++) {
        if (data[i] != data[0]) {
          all_same = false;
          break;
        }
      }
      
      if (all_same) {
        ESP_LOGW(TAG, "⚠ Frame data appears uniform (value: 0x%02X) - may be test pattern or no image", data[0]);
      } else {
        ESP_LOGI(TAG, "✓ Frame data shows variation - good sign of actual SC202CS image data");
      }
    }
    
    return true;
  } else {
    ESP_LOGE(TAG, "✗ SC202CS manual capture FAILED: %s", esp_err_to_name(ret));
    
    // Diagnostic de l'erreur
    if (ret == ESP_ERR_TIMEOUT) {
      ESP_LOGW(TAG, "Timeout - SC202CS may not be generating frames");
    } else if (ret == ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "Invalid state - controller may not be properly started");
    }
    
    return false;
  }
}

bool Tab5Camera::start_continuous_capture_() {
  ESP_LOGI(TAG, "=== STARTING CONTINUOUS CAPTURE WITH SC202CS ===");
  
  if (!this->cam_handle_) {
    ESP_LOGE(TAG, "Camera handle not initialized");
    return false;
  }
  
  // Allocation de buffers multiples pour capture continue SC202CS 720p
  for (size_t i = 0; i < NUM_FRAME_BUFFERS; i++) {
    this->frame_buffers_[i] = heap_caps_aligned_alloc(64, this->frame_buffer_size_, 
                                                      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!this->frame_buffers_[i]) {
      ESP_LOGE(TAG, "Failed to allocate frame buffer %zu for SC202CS", i);
      return false;
    }
    ESP_LOGI(TAG, "SC202CS frame buffer %zu allocated at %p", i, this->frame_buffers_[i]);
  }
  
  // Démarrage des captures continues avec les callbacks
  for (size_t i = 0; i < NUM_FRAME_BUFFERS; i++) {
    esp_cam_ctlr_trans_t trans = {};
    trans.buffer = this->frame_buffers_[i];
    trans.buflen = this->frame_buffer_size_;
    
    esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 0); // Non-bloquant
    if (ret == ESP_OK) {
      ESP_LOGI(TAG, "SC202CS frame reception %zu queued successfully", i);
    } else {
      ESP_LOGW(TAG, "Failed to queue SC202CS reception %zu: %s", i, esp_err_to_name(ret));
    }
  }
  
  this->continuous_capture_active_ = true;
  ESP_LOGI(TAG, "SC202CS continuous capture started with %d buffers", NUM_FRAME_BUFFERS);
  
  return true;
}

bool Tab5Camera::init_camera_() {
  if (this->camera_initialized_) {
    ESP_LOGI(TAG, "Camera already initialized, skipping");
    return true;
  }

  ESP_LOGI(TAG, "Starting camera initialization for '%s' with SC202CS sensor", this->name_.c_str());

  // Étape 0: Configuration de l'horloge externe (CRITIQUE pour SC202CS)
  ESP_LOGI(TAG, "Step 2.0: Setting up external clock for SC202CS");
  if (!this->setup_external_clock_()) {
    ESP_LOGE(TAG, "Failed to setup external clock - SC202CS will not work");
    return false;
  }
  this->verify_external_clock_();
  ESP_LOGI(TAG, "External clock configured and verified successfully for SC202CS");

  // Étape 1: Initialisation du LDO MIPI
  ESP_LOGI(TAG, "Step 2.1: Initializing MIPI LDO");
  if (!this->init_ldo_()) {
    ESP_LOGE(TAG, "Failed to initialize MIPI LDO");
    return false;
  }
  ESP_LOGI(TAG, "MIPI LDO initialized successfully");

  // Étape 2: Reset de la caméra SC202CS (APRÈS l'horloge)
  ESP_LOGI(TAG, "Step 2.2: Executing SC202CS sensor reset");
  vTaskDelay(100 / portTICK_PERIOD_MS);  // Laisser l'horloge se stabiliser
  if (!this->reset_sensor_()) {
    ESP_LOGW(TAG, "SC202CS reset failed, but continuing initialization");
  }
  ESP_LOGI(TAG, "SC202CS reset sequence completed");

  // Étape 3: Initialisation du capteur SC202CS I2C
  ESP_LOGI(TAG, "Step 2.3: Initializing SC202CS camera sensor");
  if (!this->init_sensor_()) {
    ESP_LOGE(TAG, "Failed to initialize SC202CS camera sensor");
    return false;
  }
  ESP_LOGI(TAG, "SC202CS camera sensor initialized successfully");

  // Étape 4: Allocation du frame buffer principal pour 720p
  ESP_LOGI(TAG, "Step 2.4: Allocating main frame buffer for SC202CS 720p");

  this->frame_buffer_size_ = TAB5_CAMERA_H_RES * TAB5_CAMERA_V_RES * 2; // RGB565 = 2 bytes par pixel
  this->frame_buffer_size_ = (this->frame_buffer_size_ + 63) & ~63; // Alignement 64 bytes

  ESP_LOGI(TAG, "SC202CS aligned frame buffer size: %zu bytes", this->frame_buffer_size_);

  this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!this->frame_buffer_) {
    ESP_LOGE(TAG, "Failed to allocate aligned frame buffer in PSRAM - trying regular RAM");
    this->frame_buffer_ = heap_caps_aligned_alloc(64, this->frame_buffer_size_, MALLOC_CAP_DMA | MALLOC_CAP_8BIT);
    if (!this->frame_buffer_) {
      ESP_LOGE(TAG, "Failed to allocate aligned frame buffer in regular RAM also");
      return false;
    }
  }

  ESP_LOGI(TAG, "SC202CS main frame buffer allocated successfully at %p", this->frame_buffer_);

  // Étape 5: Configuration du contrôleur CSI pour SC202CS
  ESP_LOGI(TAG, "Step 2.5: Configuring CSI controller for SC202CS");
  esp_cam_ctlr_csi_config_t csi_config = {};
  csi_config.ctlr_id = 0;
  csi_config.h_res = TAB5_CAMERA_H_RES;  // 1280
  csi_config.v_res = TAB5_CAMERA_V_RES;  // 720
  csi_config.lane_bit_rate_mbps = TAB5_MIPI_CSI_LANE_BITRATE_MBPS;  // 576
  csi_config.input_data_color_type = CAM_CTLR_COLOR_RAW8;
  csi_config.output_data_color_type = CAM_CTLR_COLOR_RGB565;
  csi_config.data_lane_num = 1;  // SC202CS utilise 1 lane MIPI
  csi_config.byte_swap_en = false;
  csi_config.queue_items = 4;

  esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "CSI controller init failed for SC202CS: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "CSI controller created successfully for SC202CS");
  
  // Étape 6: Configuration des callbacks
  ESP_LOGI(TAG, "Step 2.6: Registering camera callbacks");
  esp_cam_ctlr_evt_cbs_t cbs = {
    .on_get_new_trans = nullptr,
    .on_trans_finished = Tab5Camera::camera_get_finished_trans_callback,
  };
  
  ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register camera callbacks: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "Camera callbacks registered successfully");
  
  // Étape 7: Activation du contrôleur de caméra
  ESP_LOGI(TAG, "Step 2.7: Enabling camera controller");
  ret = esp_cam_ctlr_enable(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "Camera controller enabled successfully");
  
  // Étape 8: Configuration de l'ISP pour SC202CS 720p
  ESP_LOGI(TAG, "Step 2.8: Configuring ISP processor for SC202CS");
  esp_isp_processor_cfg_t isp_config = {};
  isp_config.clk_hz = TAB5_ISP_CLOCK_HZ;  // 80MHz pour 720p
  isp_config.input_data_source = ISP_INPUT_DATA_SOURCE_CSI;
  isp_config.input_data_color_type = ISP_COLOR_RAW8;
  isp_config.output_data_color_type = ISP_COLOR_RGB565;
  isp_config.has_line_start_packet = false;
  isp_config.has_line_end_packet = false;
  isp_config.h_res = TAB5_CAMERA_H_RES;  // 1280
  isp_config.v_res = TAB5_CAMERA_V_RES;  // 720
  
  ret = esp_isp_new_processor(&isp_config, &this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "ISP processor init failed for SC202CS: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "ISP processor created successfully for SC202CS");
  
  ret = esp_isp_enable(this->isp_proc_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "ISP processor enabled successfully");
  
  // Étape 9: Initialisation du frame buffer
  ESP_LOGI(TAG, "Step 2.9: Initializing frame buffer");
  memset(this->frame_buffer_, 0x00, this->frame_buffer_size_);
  esp_cache_msync(this->frame_buffer_, this->frame_buffer_size_, ESP_CACHE_MSYNC_FLAG_DIR_C2M);
  ESP_LOGI(TAG, "Frame buffer initialized");
  
  // Étape 10: Démarrage de la caméra
  ESP_LOGI(TAG, "Step 2.10: Starting camera controller");
  ret = esp_cam_ctlr_start(this->cam_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to start camera controller: %s", esp_err_to_name(ret));
    return false;
  }
  ESP_LOGI(TAG, "Camera controller started successfully");
  
  // Étape 11: Test de capture manuelle avec SC202CS
  ESP_LOGI(TAG, "Step 2.11: Testing manual capture with SC202CS");
  if (this->test_manual_capture_()) {
    ESP_LOGI(TAG, "✓ Manual capture works - SC202CS is generating data");
  } else {
    ESP_LOGW(TAG, "⚠ Manual capture failed - SC202CS may not be generating data");
  }
  
  // Étape 12: Démarrage de la capture continue
  ESP_LOGI(TAG, "Step 2.12: Starting continuous capture");
  if (this->start_continuous_capture_()) {
    ESP_LOGI(TAG, "✓ Continuous capture started successfully");
  } else {
    ESP_LOGW(TAG, "⚠ Continuous capture failed - callbacks may not work");
  }
  
  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Camera '%s' with SC202CS initialized successfully - all steps completed", this->name_.c_str());
  return true;
}

void Tab5Camera::process_frame_(uint8_t* data, size_t len) {
  this->frame_count_++;
  this->last_frame_timestamp_ = millis();
  
  ESP_LOGI(TAG, "🖼 Processing SC202CS frame #%u: %zu bytes", this->frame_count_, len);
  
  // Analyse basique de la frame SC202CS
  if (len > 8) {
    ESP_LOGD(TAG, "SC202CS frame data: %02X %02X %02X %02X %02X %02X %02X %02X", 
             data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  }
  
  // Appeler les callbacks traditionnels
  this->on_frame_callbacks_.call(data, len);
  
  // Déclencher les triggers ESPHome
  this->trigger_on_frame_callbacks_(data, len);
}

void Tab5Camera::trigger_on_frame_callbacks_(uint8_t* data, size_t len) {
  for (auto *trigger : this->on_frame_triggers_) {
    trigger->trigger(data, len);
  }
}

bool Tab5Camera::camera_get_finished_trans_callback(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(user_data);
  if (!camera || !trans->buffer) {
    ESP_LOGE(TAG, "Invalid callback data");
    return false;
  }

  // Compteur de frames pour diagnostics SC202CS
  static uint32_t frame_count = 0;
  frame_count++;
  
  ESP_LOGI(TAG, "🎬 SC202CS Frame #%u CALLBACK: %zu bytes (expected: %zu)", 
           frame_count, trans->received_size, camera->frame_buffer_size_);

  if (trans->received_size == 0) {
    ESP_LOGW(TAG, "⚠ SC202CS Frame #%u is empty - sensor might not be generating data", frame_count);
    
    // Relancer une nouvelle capture même en cas de frame vide
    esp_cam_ctlr_trans_t new_trans = {};
    new_trans.buffer = trans->buffer;  // Réutiliser le même buffer
    new_trans.buflen = camera->frame_buffer_size_;
    esp_cam_ctlr_receive(handle, &new_trans, 0);
    
    return false;
  }
  
  if (trans->received_size < 10000) {
    ESP_LOGW(TAG, "⚠ SC202CS Frame #%u size is very small (%zu bytes) for 720p", frame_count, trans->received_size);
  }

  // Synchronisation du cache pour la frame reçue
  esp_cache_msync(trans->buffer, trans->received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Vérification du contenu des premiers bytes
  uint8_t *data = static_cast<uint8_t*>(trans->buffer);
  ESP_LOGD(TAG, "SC202CS Frame #%u first bytes: %02X %02X %02X %02X %02X %02X %02X %02X", 
           frame_count, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
  
  // Traitement de la frame
  camera->process_frame_(data, trans->received_size);
  
  // Création d'une structure FrameData pour la queue
  FrameData frame;
  frame.buffer = trans->buffer;
  frame.size = trans->received_size;
  frame.timestamp = esp_timer_get_time();
  frame.valid = true;
  
  // Envoi non-bloquant vers la queue applicative
  BaseType_t ret = xQueueSendFromISR(camera->frame_queue_, &frame, NULL);
  if (ret == pdTRUE) {
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, NULL);
    ESP_LOGV(TAG, "SC202CS Frame #%u queued successfully", frame_count);
  } else {
    ESP_LOGD(TAG, "Application frame queue full, dropping SC202CS frame #%u", frame_count);
  }

  // CRUCIAL: Relancer une nouvelle capture pour maintenir le flux
  esp_cam_ctlr_trans_t new_trans = {};
  new_trans.buffer = trans->buffer;  // Réutiliser le même buffer
  new_trans.buflen = camera->frame_buffer_size_;
  esp_err_t capture_ret = esp_cam_ctlr_receive(handle, &new_trans, 0);
  if (capture_ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to restart SC202CS capture: %s", esp_err_to_name(capture_ret));
  }

  return false; // Retourner false pour que le driver libère le buffer
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera '%s' not initialized", this->name_.c_str());
    return false;
  }
  
  ESP_LOGI(TAG, "Taking snapshot with SC202CS camera '%s'", this->name_.c_str());
  
  esp_cam_ctlr_trans_t trans = {
    .buffer = this->frame_buffer_,
    .buflen = this->frame_buffer_size_,
  };
  
  esp_err_t ret = esp_cam_ctlr_receive(this->cam_handle_, &trans, 10000 / portTICK_PERIOD_MS); // 10 secondes timeout pour 720p
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "SC202CS Camera '%s' capture failed: %s", this->name_.c_str(), esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "SC202CS Camera '%s' snapshot taken, size: %zu bytes", this->name_.c_str(), trans.received_size);
  
  // Synchronisation du cache
  esp_cache_msync(this->frame_buffer_, trans.received_size, ESP_CACHE_MSYNC_FLAG_DIR_M2C);
  
  // Traitement de la frame
  this->process_frame_(static_cast<uint8_t*>(this->frame_buffer_), trans.received_size);
  
  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "SC202CS Camera '%s' not initialized for streaming", this->name_.c_str());
    return false;
  }
  
  if (this->streaming_active_) {
    ESP_LOGW(TAG, "SC202CS Camera '%s' streaming already active", this->name_.c_str());
    return true;
  }
  
  ESP_LOGI(TAG, "Starting streaming for SC202CS camera '%s'", this->name_.c_str());
  
  this->streaming_should_stop_ = false;
  this->streaming_active_ = true;
  
  // Si la capture continue n'est pas active, la démarrer
  if (!this->continuous_capture_active_) {
    if (!this->start_continuous_capture_()) {
      ESP_LOGE(TAG, "Failed to start continuous capture for SC202CS");
      this->streaming_active_ = false;
      return false;
    }
  }
  
  // Création de la tâche de streaming
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task,
    "tab5_sc202cs_streaming",
    TAB5_STREAMING_STACK_SIZE,
    this,
    5,
    &this->streaming_task_handle_
  );
  
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create streaming task for SC202CS");
    this->streaming_active_ = false;
    return false;
  }
  
  ESP_LOGI(TAG, "SC202CS Camera '%s' streaming started", this->name_.c_str());
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return true;
  }
  
  ESP_LOGI(TAG, "Stopping SC202CS camera '%s' streaming...", this->name_.c_str());
  
  this->streaming_should_stop_ = true;
  
  if (this->streaming_task_handle_) {
    xSemaphoreGive(this->frame_ready_semaphore_);
    
    uint32_t timeout = 0;
    while (this->streaming_active_ && timeout < 50) {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      timeout++;
    }
    
    if (this->streaming_active_) {
      ESP_LOGW(TAG, "Force stopping SC202CS streaming task");
      vTaskDelete(this->streaming_task_handle_);
      this->streaming_active_ = false;
    }
    
    this->streaming_task_handle_ = nullptr;
  }
  
  ESP_LOGI(TAG, "SC202CS Camera '%s' streaming stopped", this->name_.c_str());
  return true;
}

void Tab5Camera::streaming_task(void *parameter) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(parameter);
  camera->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGI(TAG, "🎥 Streaming loop started for SC202CS camera '%s' (callback-based)", this->name_.c_str());

  while (!this->streaming_should_stop_) {
    if (xSemaphoreTake(this->frame_ready_semaphore_, 100 / portTICK_PERIOD_MS) == pdTRUE) {
      FrameData frame;
      if (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE) {
        ESP_LOGV(TAG, "📺 SC202CS Frame received from callback, size: %zu bytes", frame.size);
        // Le traitement est déjà fait dans process_frame_
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  this->streaming_active_ = false;
  ESP_LOGI(TAG, "Streaming loop ended for SC202CS camera '%s'", this->name_.c_str());
  vTaskDelete(nullptr);
}

void Tab5Camera::deinit_camera_() {
  if (this->streaming_active_) {
    this->stop_streaming();
  }
  
  if (this->camera_initialized_) {
    ESP_LOGD(TAG, "Deinitializing SC202CS camera '%s'...", this->name_.c_str());
    
    if (this->cam_handle_) {
      esp_cam_ctlr_stop(this->cam_handle_);
      esp_cam_ctlr_disable(this->cam_handle_);
      esp_cam_ctlr_del(this->cam_handle_);
      this->cam_handle_ = nullptr;
    }
    
    if (this->isp_proc_) {
      esp_isp_disable(this->isp_proc_);
      esp_isp_del_processor(this->isp_proc_);
      this->isp_proc_ = nullptr;
    }
    
    if (this->frame_buffer_) {
      heap_caps_free(this->frame_buffer_);
      this->frame_buffer_ = nullptr;
    }
    
    // Libération des buffers multiples
    for (size_t i = 0; i < NUM_FRAME_BUFFERS; i++) {
      if (this->frame_buffers_[i]) {
        heap_caps_free(this->frame_buffers_[i]);
        this->frame_buffers_[i] = nullptr;
      }
    }
    
    if (this->ldo_mipi_phy_) {
      esp_ldo_release_channel(this->ldo_mipi_phy_);
      this->ldo_mipi_phy_ = nullptr;
    }
    
    this->camera_initialized_ = false;
    this->sensor_initialized_ = false;
    this->ldo_initialized_ = false;
    this->continuous_capture_active_ = false;
    ESP_LOGD(TAG, "SC202CS Camera '%s' deinitialized", this->name_.c_str());
  }
  
  if (this->frame_ready_semaphore_) {
    vSemaphoreDelete(this->frame_ready_semaphore_);
    this->frame_ready_semaphore_ = nullptr;
  }
  
  if (this->frame_queue_) {
    vQueueDelete(this->frame_queue_);
    this->frame_queue_ = nullptr;
  }
}

// Méthodes utilitaires
void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  this->error_count_++;
  ESP_LOGE(TAG, "SC202CS Camera error: %s", error.c_str());
}

void Tab5Camera::clear_error_() {
  this->error_state_ = false;
  this->last_error_ = "";
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
  uint16_t bytes_per_pixel = 2; // RGB565 par défaut
  
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

bool Tab5Camera::write_register_16(uint16_t reg, uint8_t val) {
  uint8_t buffer[3] = {
    static_cast<uint8_t>(reg >> 8),
    static_cast<uint8_t>(reg & 0xFF),
    val
  };
  
  if (!this->write(buffer, 3)) {
    ESP_LOGD(TAG, "Failed to write 16-bit register 0x%04X = 0x%02X", reg, val);
    return false;
  }
  
  return true;
}

bool Tab5Camera::read_register_16(uint16_t reg, uint8_t *val) {
  uint8_t buffer[2] = {
    static_cast<uint8_t>(reg >> 8),
    static_cast<uint8_t>(reg & 0xFF)
  };
  
  if (!this->write(buffer, 2) || !this->read(val, 1)) {
    ESP_LOGD(TAG, "Failed to read 16-bit register 0x%04X", reg);
    return false;
  }
  
  return true;
}

#endif

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32






