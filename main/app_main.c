// LED Light Example (with DHT11 sensor + OLED + LED/Buzzer alerts + FAN + PUSH NOTIFICATIONS)

#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <driver/gpio.h>
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <esp_rmaker_console.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <esp_rmaker_schedule.h>
#include <esp_rmaker_scenes.h>
#include <app_network.h>
#include <app_insights.h>
#include "app_priv.h"
#include "dht11.h"

static const char *TAG = "app_main";

// Hardware pins
#define DHT_PIN GPIO_NUM_2
#define LED_PIN GPIO_NUM_10
#define BUZZER_PIN GPIO_NUM_5
#define FAN_PIN GPIO_NUM_8
#define I2C_SDA GPIO_NUM_6
#define I2C_SCL GPIO_NUM_7
#define I2C_FREQ 100000

// PWM Configuration
#define PWM_FREQ 1000
#define PWM_RESOLUTION 8
#define PWM_CHANNEL 0

// Temperature threshold
#define TEMP_HIGH_THRESHOLD 30.0

// OLED config
#define OLED_ADDR 0x3C
#define OLED_WIDTH 128
#define OLED_HEIGHT 32

esp_rmaker_device_t *light_device;
esp_rmaker_device_t *fan_device;
static esp_rmaker_param_t *temperature_param = NULL;
static esp_rmaker_param_t *humidity_param = NULL;
static esp_rmaker_param_t *alert_param = NULL;
static esp_rmaker_param_t *temp_status_param = NULL;
static esp_rmaker_param_t *fan_power_param = NULL;
static esp_rmaker_param_t *fan_speed_param = NULL;
static float g_temperature = 0.0;
static float g_humidity = 0.0;
static bool g_fan_manual_override = false;
static bool g_fan_state = false;
static uint8_t g_fan_speed = 0;
static dht11_t dht_sensor;

// Complete 5x7 font for ASCII 32-127
static const uint8_t font5x7[][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, {0x00, 0x00, 0x5F, 0x00, 0x00}, {0x00, 0x07, 0x00, 0x07, 0x00},
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, {0x24, 0x2A, 0x7F, 0x2A, 0x12}, {0x23, 0x13, 0x08, 0x64, 0x62},
    {0x36, 0x49, 0x56, 0x20, 0x50}, {0x00, 0x08, 0x07, 0x03, 0x00}, {0x00, 0x1C, 0x22, 0x41, 0x00},
    {0x00, 0x41, 0x22, 0x1C, 0x00}, {0x2A, 0x1C, 0x7F, 0x1C, 0x2A}, {0x08, 0x08, 0x3E, 0x08, 0x08},
    {0x00, 0x80, 0x70, 0x30, 0x00}, {0x08, 0x08, 0x08, 0x08, 0x08}, {0x00, 0x00, 0x60, 0x60, 0x00},
    {0x20, 0x10, 0x08, 0x04, 0x02}, {0x3E, 0x51, 0x49, 0x45, 0x3E}, {0x00, 0x42, 0x7F, 0x40, 0x00},
    {0x72, 0x49, 0x49, 0x49, 0x46}, {0x21, 0x41, 0x49, 0x4D, 0x33}, {0x18, 0x14, 0x12, 0x7F, 0x10},
    {0x27, 0x45, 0x45, 0x45, 0x39}, {0x3C, 0x4A, 0x49, 0x49, 0x31}, {0x41, 0x21, 0x11, 0x09, 0x07},
    {0x36, 0x49, 0x49, 0x49, 0x36}, {0x46, 0x49, 0x49, 0x29, 0x1E}, {0x00, 0x00, 0x14, 0x00, 0x00},
    {0x00, 0x40, 0x34, 0x00, 0x00}, {0x00, 0x08, 0x14, 0x22, 0x41}, {0x14, 0x14, 0x14, 0x14, 0x14},
    {0x00, 0x41, 0x22, 0x14, 0x08}, {0x02, 0x01, 0x59, 0x09, 0x06}, {0x3E, 0x41, 0x5D, 0x59, 0x4E},
    {0x7C, 0x12, 0x11, 0x12, 0x7C}, {0x7F, 0x49, 0x49, 0x49, 0x36}, {0x3E, 0x41, 0x41, 0x41, 0x22},
    {0x7F, 0x41, 0x41, 0x41, 0x3E}, {0x7F, 0x49, 0x49, 0x49, 0x41}, {0x7F, 0x09, 0x09, 0x09, 0x01},
    {0x3E, 0x41, 0x41, 0x51, 0x73}, {0x7F, 0x08, 0x08, 0x08, 0x7F}, {0x00, 0x41, 0x7F, 0x41, 0x00},
    {0x20, 0x40, 0x41, 0x3F, 0x01}, {0x7F, 0x08, 0x14, 0x22, 0x41}, {0x7F, 0x40, 0x40, 0x40, 0x40},
    {0x7F, 0x02, 0x1C, 0x02, 0x7F}, {0x7F, 0x04, 0x08, 0x10, 0x7F}, {0x3E, 0x41, 0x41, 0x41, 0x3E},
    {0x7F, 0x09, 0x09, 0x09, 0x06}, {0x3E, 0x41, 0x51, 0x21, 0x5E}, {0x7F, 0x09, 0x19, 0x29, 0x46},
    {0x26, 0x49, 0x49, 0x49, 0x32}, {0x03, 0x01, 0x7F, 0x01, 0x03}, {0x3F, 0x40, 0x40, 0x40, 0x3F},
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, {0x3F, 0x40, 0x38, 0x40, 0x3F}, {0x63, 0x14, 0x08, 0x14, 0x63},
    {0x03, 0x04, 0x78, 0x04, 0x03}, {0x61, 0x59, 0x49, 0x4D, 0x43}, {0x00, 0x7F, 0x41, 0x41, 0x41},
    {0x02, 0x04, 0x08, 0x10, 0x20}, {0x00, 0x41, 0x41, 0x41, 0x7F}, {0x04, 0x02, 0x01, 0x02, 0x04},
    {0x40, 0x40, 0x40, 0x40, 0x40}, {0x00, 0x03, 0x07, 0x08, 0x00}, {0x20, 0x54, 0x54, 0x78, 0x40},
    {0x7F, 0x28, 0x44, 0x44, 0x38}, {0x38, 0x44, 0x44, 0x44, 0x28}, {0x38, 0x44, 0x44, 0x28, 0x7F},
    {0x38, 0x54, 0x54, 0x54, 0x18}, {0x00, 0x08, 0x7E, 0x09, 0x02}, {0x18, 0xA4, 0xA4, 0x9C, 0x78},
    {0x7F, 0x08, 0x04, 0x04, 0x78}, {0x00, 0x44, 0x7D, 0x40, 0x00}, {0x20, 0x40, 0x40, 0x3D, 0x00},
    {0x7F, 0x10, 0x28, 0x44, 0x00}, {0x00, 0x41, 0x7F, 0x40, 0x00}, {0x7C, 0x04, 0x78, 0x04, 0x78},
    {0x7C, 0x08, 0x04, 0x04, 0x78}, {0x38, 0x44, 0x44, 0x44, 0x38}, {0xFC, 0x18, 0x24, 0x24, 0x18},
    {0x18, 0x24, 0x24, 0x18, 0xFC}, {0x7C, 0x08, 0x04, 0x04, 0x08}, {0x48, 0x54, 0x54, 0x54, 0x24},
    {0x04, 0x04, 0x3F, 0x44, 0x24}, {0x3C, 0x40, 0x40, 0x20, 0x7C}, {0x1C, 0x20, 0x40, 0x20, 0x1C},
    {0x3C, 0x40, 0x30, 0x40, 0x3C}, {0x44, 0x28, 0x10, 0x28, 0x44}, {0x4C, 0x90, 0x90, 0x90, 0x7C},
    {0x44, 0x64, 0x54, 0x4C, 0x44}, {0x00, 0x08, 0x36, 0x41, 0x00}, {0x00, 0x00, 0x77, 0x00, 0x00},
    {0x00, 0x41, 0x36, 0x08, 0x00}, {0x02, 0x01, 0x02, 0x04, 0x02},
};

// I2C and OLED functions
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .scl_io_num = I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    return i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
}

static esp_err_t oled_write_cmd(uint8_t cmd) {
    uint8_t data[2] = {0x00, cmd};
    return i2c_master_write_to_device(I2C_NUM_0, OLED_ADDR, data, 2, pdMS_TO_TICKS(1000));
}

static esp_err_t oled_write_data(uint8_t *data, size_t len) {
    uint8_t *buf = malloc(len + 1);
    if (!buf) return ESP_ERR_NO_MEM;
    buf[0] = 0x40;
    memcpy(buf + 1, data, len);
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, OLED_ADDR, buf, len + 1, pdMS_TO_TICKS(1000));
    free(buf);
    return ret;
}

static void oled_init(void) {
    vTaskDelay(pdMS_TO_TICKS(100));
    oled_write_cmd(0xAE); oled_write_cmd(0xD5); oled_write_cmd(0x80);
    oled_write_cmd(0xA8); oled_write_cmd(0x1F); oled_write_cmd(0xD3);
    oled_write_cmd(0x00); oled_write_cmd(0x40); oled_write_cmd(0x8D);
    oled_write_cmd(0x14); oled_write_cmd(0x20); oled_write_cmd(0x00);
    oled_write_cmd(0xA1); oled_write_cmd(0xC8); oled_write_cmd(0xDA);
    oled_write_cmd(0x02); oled_write_cmd(0x81); oled_write_cmd(0x8F);
    oled_write_cmd(0xD9); oled_write_cmd(0xF1); oled_write_cmd(0xDB);
    oled_write_cmd(0x40); oled_write_cmd(0xA4); oled_write_cmd(0xA6);
    oled_write_cmd(0xAF);
}

static void oled_clear(void) {
    uint8_t zero[OLED_WIDTH] = {0};
    for (int page = 0; page < 4; page++) {
        oled_write_cmd(0xB0 + page);
        oled_write_cmd(0x00);
        oled_write_cmd(0x10);
        oled_write_data(zero, OLED_WIDTH);
    }
}

static void oled_draw_char(uint8_t x, uint8_t y, char ch) {
    if (ch < 32 || ch > 126) ch = ' ';
    uint8_t page = y / 8;
    oled_write_cmd(0xB0 + page);
    oled_write_cmd(0x00 | (x & 0x0F));
    oled_write_cmd(0x10 | (x >> 4));
    oled_write_data((uint8_t *)font5x7[ch - 32], 5);
    uint8_t space = 0x00;
    oled_write_data(&space, 1);
}

static void oled_draw_string(uint8_t x, uint8_t y, const char *str) {
    while (*str && x < OLED_WIDTH - 6) {
        oled_draw_char(x, y, *str);
        x += 6;
        str++;
    }
}

// Alert hardware init
static void alert_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN) | (1ULL << BUZZER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(LED_PIN, 0);
    gpio_set_level(BUZZER_PIN, 0);
    
    // Configure PWM for fan
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num = FAN_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ledc_channel);
}

static void trigger_alert(bool active) {
    gpio_set_level(LED_PIN, active ? 1 : 0);
    gpio_set_level(BUZZER_PIN, active ? 1 : 0);
}

// Fan control with speed (0-100%)
static void set_fan_speed(uint8_t speed_percent) {
    if (speed_percent > 100) speed_percent = 100;
    uint8_t duty = (speed_percent * 255) / 100;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    g_fan_state = (speed_percent > 0);
    g_fan_speed = speed_percent;
    ESP_LOGI(TAG, "Fan speed: %d%%", speed_percent);
}

static const char* get_temp_status(float temperature) {
    if (temperature >= 30.0) {
        return "Hot";
    } else if (temperature >= 20.0) {
        return "Normal";
    } else {
        return "Cold";
    }
}

// RainMaker callbacks
#ifdef CONFIG_ESP_RMAKER_CMD_RESP_ENABLE
#include <json_parser.h>
#include <esp_rmaker_cmd_resp.h>
#include <esp_rmaker_standard_types.h>
static char resp_data[100];
esp_err_t led_light_cmd_handler(const void *in_data, size_t in_len, void **out_data, size_t *out_len, esp_rmaker_cmd_ctx_t *ctx, void *priv) {
    if (!in_data) return ESP_FAIL;
    snprintf(resp_data, sizeof(resp_data), "{\"status\":\"success\"}");
    *out_data = resp_data;
    *out_len = strlen(resp_data);
    return ESP_OK;
}
#endif

// Fan write callback for RainMaker app control
static esp_err_t fan_write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_t *param,
        const esp_rmaker_param_val_t val, void *priv_data, esp_rmaker_write_ctx_t *ctx) {
    const char *param_name = esp_rmaker_param_get_name(param);
    
    if (strcmp(param_name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
        // Power switch toggled - MANUAL CONTROL
        bool power_state = val.val.b;
        g_fan_manual_override = true;
        
        if (power_state) {
            set_fan_speed(g_fan_speed > 0 ? g_fan_speed : 50);
        } else {
            set_fan_speed(0);
        }
        esp_rmaker_param_update_and_report(param, val);
        ESP_LOGI(TAG, "Manual fan power: %s", power_state ? "ON" : "OFF");
        
    } else if (strcmp(param_name, "Speed") == 0) {
        // Speed slider adjusted - MANUAL CONTROL
        int speed = val.val.i;
        g_fan_manual_override = true;
        set_fan_speed(speed);
        esp_rmaker_param_update_and_report(param, val);
        ESP_LOGI(TAG, "Manual fan speed: %d%%", speed);
        
        // Update power state
        if (fan_power_param) {
            esp_rmaker_param_update_and_report(fan_power_param, esp_rmaker_bool(speed > 0));
        }
        
    } else if (strcmp(param_name, "Auto Mode") == 0) {
        // Auto Mode toggle - RE-ENABLE AUTOMATIC CONTROL
        bool auto_enabled = val.val.b;
        g_fan_manual_override = !auto_enabled;
        
        if (auto_enabled) {
            ESP_LOGI(TAG, "AUTO MODE ENABLED - Fan will respond to temperature");
        } else {
            ESP_LOGI(TAG, "AUTO MODE DISABLED - Manual control active");
        }
        esp_rmaker_param_update_and_report(param, val);
    }
    
    return ESP_OK;
}

static esp_err_t bulk_write_cb(const esp_rmaker_device_t *device, const esp_rmaker_param_write_req_t write_req[],
        uint8_t count, void *priv_data, esp_rmaker_write_ctx_t *ctx) {
    for (int i = 0; i < count; i++) {
        const char *param_name = esp_rmaker_param_get_name(write_req[i].param);
        ESP_LOGI(TAG, "Param update: %s", param_name);
        esp_rmaker_param_update(write_req[i].param, write_req[i].val);
    }
    return ESP_OK;
}

// Sensor task with alerts and fan control
static void sensor_task(void *arg) {
    dht_sensor.dht11_pin = DHT_PIN;
    float prev_temp = 0.0, prev_hum = 0.0;
    int report_timer = 0;
    bool alert_sent = false;
    bool push_notification_sent = false;

    while (1) {
        int ret = dht11_read(&dht_sensor, 5);
        if (ret == 0) {
            g_temperature = dht_sensor.temperature;
            g_humidity = dht_sensor.humidity;
        } else {
            ESP_LOGW(TAG, "DHT11 read failed");
        }

        // Update OLED with proper formatting + fan status
        char line1[32], line2[32];
        snprintf(line1, sizeof(line1), "T:%.1fC H:%.0f%%", g_temperature, g_humidity);
        snprintf(line2, sizeof(line2), "Fan:%d%%%s", 
                 g_fan_speed,
                 g_fan_manual_override ? "M" : "A");

        oled_clear();
        oled_draw_string(0, 0, line1);
        oled_draw_string(0, 16, line2);

        // AUTOMATIC FAN CONTROL (only if not manually controlled)
        if (!g_fan_manual_override) {
            uint8_t target_speed = 0;
            
            if (g_temperature >= 35.0) {
                target_speed = 100;
                
            } else if (g_temperature >= 30.0) {
                target_speed = 50 + (uint8_t)(((g_temperature - 30.0) / 5.0) * 50);
                
            } else if (g_temperature > 27.0 && g_fan_speed > 0) {
                target_speed = 50;
                
            } else {
                target_speed = 0;
            }
            
            // Only update if speed changed
            if (target_speed != g_fan_speed) {
                set_fan_speed(target_speed);
                ESP_LOGI(TAG, "AUTO: Fan %s at %d%% (Temp: %.1f°C)", 
                         target_speed > 0 ? "ON" : "OFF",
                         target_speed, g_temperature);
                
                vTaskDelay(pdMS_TO_TICKS(100));
                if (fan_speed_param) {
                    esp_rmaker_param_update_and_report(fan_speed_param, esp_rmaker_int(target_speed));
                }
                vTaskDelay(pdMS_TO_TICKS(100));
                if (fan_power_param) {
                    esp_rmaker_param_update_and_report(fan_power_param, esp_rmaker_bool(target_speed > 0));
                }
            }
        }

        // High temperature alert
        if (g_temperature > TEMP_HIGH_THRESHOLD) {
            trigger_alert(true);
            
            if (!push_notification_sent) {
                char alert_msg[100];
                snprintf(alert_msg, sizeof(alert_msg), 
                         "⚠️ High Temperature Alert! Current: %.1f°C (Threshold: %.1f°C)", 
                         g_temperature, TEMP_HIGH_THRESHOLD);
                esp_rmaker_raise_alert(alert_msg);
                ESP_LOGW(TAG, "PUSH NOTIFICATION SENT: %s", alert_msg);
                push_notification_sent = true;
            }
            
            if (!alert_sent && alert_param) {
                esp_rmaker_param_update_and_report(alert_param, esp_rmaker_bool(true));
                ESP_LOGW(TAG, "HIGH TEMP ALERT! %.1f°C", g_temperature);
                alert_sent = true;
            }
        } else {
            trigger_alert(false);
            push_notification_sent = false;
            
            if (alert_sent && alert_param) {
                esp_rmaker_param_update_and_report(alert_param, esp_rmaker_bool(false));
                alert_sent = false;
            }
        }

        // CLOUD REPORTING
        report_timer++;
        bool big_change = (fabs(g_temperature - prev_temp) >= 0.5) || 
                          (fabs(g_humidity - prev_hum) >= 2.0);
        
        if (report_timer >= 30 || big_change) {
            if (temperature_param) {
                esp_rmaker_param_update_and_report(temperature_param, 
                                                   esp_rmaker_float(g_temperature));
            }
            
            vTaskDelay(pdMS_TO_TICKS(200));
            
            if (humidity_param) {
                esp_rmaker_param_update_and_report(humidity_param, 
                                                   esp_rmaker_float(g_humidity));
            }
            
            vTaskDelay(pdMS_TO_TICKS(200));
            
            if (temp_status_param) {
                const char* status = get_temp_status(g_temperature);
                esp_rmaker_param_update_and_report(temp_status_param, 
                                                   esp_rmaker_str(status));
            }
            
            prev_temp = g_temperature;
            prev_hum = g_humidity;
            report_timer = 0;
        }

        ESP_LOGI(TAG, "T=%.1f C, H=%.1f %%, Fan=%d%%%s", 
                 g_temperature, g_humidity, g_fan_speed,
                 g_fan_manual_override ? "(MANUAL)" : "(AUTO)");
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main() {
    esp_rmaker_console_init();
    app_driver_init();

    ESP_ERROR_CHECK(nvs_flash_init());
    app_network_init();

    // Init hardware
    alert_init();
    ESP_ERROR_CHECK(i2c_master_init());
    oled_init();
    oled_clear();

    // Enable time sync
    esp_rmaker_config_t rainmaker_cfg = {.enable_time_sync = true};
    esp_rmaker_node_t *node = esp_rmaker_node_init(&rainmaker_cfg, "Env_Monitor", "Environment");
    if (!node) abort();

    // Create temperature sensor device
    light_device = esp_rmaker_temp_sensor_device_create("NotiTemp", NULL, 25.0);
    esp_rmaker_device_add_bulk_cb(light_device, bulk_write_cb, NULL);

    temperature_param = esp_rmaker_device_get_param_by_type(light_device, ESP_RMAKER_PARAM_TEMPERATURE);

    if (temperature_param) {
        esp_rmaker_param_update_and_report(temperature_param, esp_rmaker_float(25.0));
    }

    humidity_param = esp_rmaker_param_create("Humidity", "esp.param.humidity", 
                                             esp_rmaker_float(60.0), 
                                             PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    esp_rmaker_device_add_param(light_device, humidity_param);
    
    alert_param = esp_rmaker_param_create("High Temp alert count", ESP_RMAKER_PARAM_TOGGLE, 
                                          esp_rmaker_bool(false), PROP_FLAG_READ);
    esp_rmaker_device_add_param(light_device, alert_param);
    
    temp_status_param = esp_rmaker_param_create("Status", "esp.param.status", 
                                                esp_rmaker_str("Normal"), 
                                                PROP_FLAG_READ);
    esp_rmaker_device_add_param(light_device, temp_status_param);

    esp_rmaker_node_add_device(node, light_device);

    // Create fan device
    fan_device = esp_rmaker_switch_device_create("Cooling Fan", NULL, false);
    esp_rmaker_device_add_cb(fan_device, fan_write_cb, NULL);
    fan_power_param = esp_rmaker_device_get_param_by_type(fan_device, ESP_RMAKER_PARAM_POWER);
    
    fan_speed_param = esp_rmaker_param_create("Speed", "esp.param.speed", 
                                              esp_rmaker_int(0), 
                                              PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_param_add_ui_type(fan_speed_param, ESP_RMAKER_UI_SLIDER);
    esp_rmaker_param_add_bounds(fan_speed_param, esp_rmaker_int(0), esp_rmaker_int(100), esp_rmaker_int(1));
    esp_rmaker_device_add_param(fan_device, fan_speed_param);
    
    // AUTO MODE TOGGLE
    esp_rmaker_param_t *auto_mode_param = esp_rmaker_param_create("Auto Mode", ESP_RMAKER_PARAM_TOGGLE, 
                                                                    esp_rmaker_bool(true), 
                                                                    PROP_FLAG_READ | PROP_FLAG_WRITE);
    esp_rmaker_device_add_param(fan_device, auto_mode_param);
    
    esp_rmaker_node_add_device(node, fan_device);

    esp_rmaker_ota_enable_default();
    esp_rmaker_timezone_service_enable();
    esp_rmaker_schedule_enable();
    esp_rmaker_scenes_enable();

    esp_rmaker_system_serv_config_t system_serv_config = {
        .flags = SYSTEM_SERV_FLAGS_ALL,
        .reboot_seconds = 2,
        .reset_seconds = 2,
        .reset_reboot_seconds = 2,
    };
    esp_rmaker_system_service_enable(&system_serv_config);
    app_insights_enable();

    esp_rmaker_start();
    app_network_set_custom_mfg_data(MGF_DATA_DEVICE_TYPE_LIGHT, MFG_DATA_DEVICE_SUBTYPE_LIGHT);
    app_network_start((app_network_pop_type_t)CONFIG_APP_POP_TYPE);

    xTaskCreate(sensor_task, "sensor_task", 8192, NULL, 5, NULL);
}
