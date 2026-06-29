// [B] APP IMPLEMENTATION – Gate control example for Intelligent Universal Part A
//  ✅ Part B remains product-specific
//  ✅ Part A remains universal and controls OTA/app lifecycle
//  ✅ App does NOT start itself immediately after px_start()
//  ✅ Part A starts Part B only after normal boot or after OTA validation
//  ✅ Safe-stop callback confirms all gate outputs are LOW before OTA flashing
//  ✅ OTA-safe Flash Encryption DEVELOPMENT -> RELEASE production lock
//
// PIN MAP:
//   OPEN  -> GPIO16
//   CLOSE -> GPIO17
//   STOP  -> GPIO18
//   STATE -> GPIO19 (input pulldown)
// ---------------------------------------------------------------------------------

#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_flash_encrypt.h"
#include "driver/gpio.h"
#include "cJSON.h"
#include "pearlexa_connect.h"

// ---------------------------------------------------------------------------------
// Part A API declarations.
// If Part B is below Part A in the same main.c file, these are already known.
// These declarations are kept here to make Part B clear and reusable.
// ---------------------------------------------------------------------------------
void px_init(const px_cfg_t* cfg);
void px_start(void);
void px_publish_json(const char* json);
void px_set_message_callback(px_msg_cb_t cb);
bool px_mqtt_is_connected(void);
bool px_is_config_mode(void);
bool px_ota_update_is_in_progress(void);
bool px_ota_first_boot_validation_pending(void);
bool px_app_is_started(void);

void px_set_app_start_callback(void (*cb)(void));
void px_set_app_stop_callback(void (*cb)(void));
void px_set_ota_prepare_callback(bool (*cb)(void));
void px_set_app_self_test_callback(bool (*cb)(void));

// ======= App config you fill in (common firmware endpoints only) =======
// Device identity is no longer hardcoded here.
// Part A loads these from NVS namespace "px":
//   deviceSerial = "SN:G1005005"
//   authNumber   = "device-specific-token"
static const char* APP_MQTT_HOST     = "mqttserver1.pearlexa.cloud";
static const int   APP_TLS_PORT      = 8883;
static const char* APP_WSS_URI       = "wss://mqttserver1.pearlexa.cloud/mqtt";
static const int   APP_WIFI_STRENGTH_PERIOD_MS = 60000;

// OTA
// Increase this version for the production-lock OTA.
static const char* APP_FW_VERSION = "1.0.2";
static const char* APP_OTA_VERSION_URL = "https://raw.githubusercontent.com/nithila31/pearlexa-ota-test/refs/heads/main/version.json";

// Optional metadata protection used by the new universal Part A.
// version.json may include:
//   "hardware": "PX-SG100-DC250-24-X-V1",
//   "model": "gate_controller",
//   "rollout": 100,
//   "min_current_version": "1.0.0"
static const char* APP_HW_MODEL    = "PX-SG100-DC250-24-X-V1";
static const char* APP_DEVICE_TYPE = "gate_controller";

static const char* TAG_APP = "APP";

// ---------------------------------------------------------------------------------
// OTA-safe one-time production lock for ESP32 Flash Encryption
//
// IMPORTANT:
//   1. This function is NOT called from app_main().
//   2. It is called from app_start() only after Part A allows the product app.
//   3. For OTA rollback mode, Part A allows app_start() only after validation.
//   4. This moves Flash Encryption from DEVELOPMENT mode to RELEASE mode.
//   5. After this runs, future normal firmware updates should be done by OTA.
//   6. After successful production lock, publish another normal OTA firmware with
//      APP_ENABLE_FLASH_ENCRYPTION_RELEASE_LOCK set to 0.
// ---------------------------------------------------------------------------------
#define APP_ENABLE_FLASH_ENCRYPTION_RELEASE_LOCK  0

static void app_flash_encryption_release_lock_once(void)
{
#if APP_ENABLE_FLASH_ENCRYPTION_RELEASE_LOCK
    esp_flash_enc_mode_t mode = esp_get_flash_encryption_mode();

    if (mode == ESP_FLASH_ENC_MODE_DEVELOPMENT) {
        ESP_LOGW(TAG_APP, "======================================================");
        ESP_LOGW(TAG_APP, "FLASH ENCRYPTION: DEVELOPMENT -> RELEASE MODE");
        ESP_LOGW(TAG_APP, "OTA image has already passed Part A validation.");
        ESP_LOGW(TAG_APP, "This is a one-way production security lock.");
        ESP_LOGW(TAG_APP, "Future normal firmware updates must be done by OTA.");
        ESP_LOGW(TAG_APP, "======================================================");

        vTaskDelay(pdMS_TO_TICKS(1000));

        esp_flash_encryption_set_release_mode();

        ESP_LOGW(TAG_APP, "Release mode eFuses burned. Restarting now...");
        vTaskDelay(pdMS_TO_TICKS(500));
        esp_restart();
    }

    if (mode == ESP_FLASH_ENC_MODE_RELEASE) {
        ESP_LOGI(TAG_APP, "Flash encryption already in RELEASE/production mode.");
        return;
    }

    ESP_LOGE(TAG_APP, "Flash encryption is not enabled. Production lock skipped.");
#else
    ESP_LOGI(TAG_APP, "Flash encryption release lock is disabled in this firmware.");
#endif
}

#ifdef CONFIG_IDF_TARGET_ESP32C3
#define OPEN_CMD_PIN            4
#define CLOSE_CMD_PIN           5
#define STOP_CMD_PIN            6
#define GATE_STATE_PIN          7
#else
#define OPEN_CMD_PIN            16
#define CLOSE_CMD_PIN           17
#define STOP_CMD_PIN            18
#define GATE_STATE_PIN          19
#endif

#define GATE_DEBOUNCE_MS        100

// ---------------- RX defer queue ----------------
typedef struct {
    char payload[256];
} app_rx_msg_t;

static QueueHandle_t s_rx_q = NULL;
static TaskHandle_t s_rx_worker_handle = NULL;
static TaskHandle_t s_gate_state_handle = NULL;

static bool s_gpio_ready = false;
static volatile bool s_app_running = false;
static volatile bool s_accept_commands = false;

static int s_last_gate_level = -1;

// ---------------- Helpers ----------------
static uint64_t app_now_ms(void)
{
    return esp_timer_get_time() / 1000ULL;
}

static int app_json_get_int_field(const char *json, const char *key)
{
    if (!json || !key || !key[0]) return -1;

    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *p = strstr(json, pattern);
    if (!p) return -1;

    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != ':') return -1;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;

    if (*p == '\"') p++;
    if (*p == '0') return 0;
    if (*p == '1') return 1;
    if (strncmp(p, "true", 4) == 0) return 1;
    if (strncmp(p, "false", 5) == 0) return 0;

    return -1;
}

static bool app_ota_or_validation_active(void)
{
    return px_ota_update_is_in_progress() || px_ota_first_boot_validation_pending();
}

static void app_all_outputs_off(void)
{
    if (!s_gpio_ready) return;

    gpio_set_level(OPEN_CMD_PIN, 0);
    gpio_set_level(CLOSE_CMD_PIN, 0);
    gpio_set_level(STOP_CMD_PIN, 0);
}

static bool app_outputs_are_off(void)
{
    if (!s_gpio_ready) return false;

    return gpio_get_level(OPEN_CMD_PIN) == 0 &&
           gpio_get_level(CLOSE_CMD_PIN) == 0 &&
           gpio_get_level(STOP_CMD_PIN) == 0;
}

// ---------------- GPIO init ----------------
static void app_gate_gpio_init(void)
{
    if (s_gpio_ready) {
        app_all_outputs_off();
        return;
    }

    gpio_config_t io = {0};
    io.intr_type = GPIO_INTR_DISABLE;
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask =
        (1ULL << OPEN_CMD_PIN) |
        (1ULL << CLOSE_CMD_PIN) |
        (1ULL << STOP_CMD_PIN);
    io.pull_down_en = 0;
    io.pull_up_en   = 0;
    ESP_ERROR_CHECK(gpio_config(&io));

    gpio_set_level(OPEN_CMD_PIN, 0);
    gpio_set_level(CLOSE_CMD_PIN, 0);
    gpio_set_level(STOP_CMD_PIN, 0);

    gpio_config_t in = {0};
    in.intr_type = GPIO_INTR_DISABLE;
    in.mode = GPIO_MODE_INPUT;
    in.pin_bit_mask = (1ULL << GATE_STATE_PIN);
    in.pull_up_en = 0;
    in.pull_down_en = 1;
    ESP_ERROR_CHECK(gpio_config(&in));

    s_gpio_ready = true;

    ESP_LOGI(TAG_APP, "GPIO init done. GateState GPIO%d = INPUT PULLDOWN", GATE_STATE_PIN);
    ESP_LOGI(TAG_APP, "CMD pins: OPEN=%d CLOSE=%d STOP=%d",
             OPEN_CMD_PIN, CLOSE_CMD_PIN, STOP_CMD_PIN);
}

// ---------------- Publishing ----------------
static void app_publish_gate_state_level(int level_now)
{
    if (!s_app_running || app_ota_or_validation_active()) return;

    int gate_state = level_now ? 1 : 0;

    cJSON* root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddNumberToObject(root, "Gate_State", gate_state);

    char* out = cJSON_PrintUnformatted(root);
    if (out) {
        px_publish_json(out);
        cJSON_free(out);
    }
    cJSON_Delete(root);

    ESP_LOGI(TAG_APP, "Gate_State -> %d (GPIO%d=%d)", gate_state, GATE_STATE_PIN, level_now);
}

static void app_publish_boot_msg(void)
{
    if (!s_app_running || app_ota_or_validation_active()) return;

    cJSON* hello = cJSON_CreateObject();
    if (!hello) return;

    cJSON_AddStringToObject(hello, "boot", "gate controller online");

    char* out = cJSON_PrintUnformatted(hello);
    if (out) {
        px_publish_json(out);
        cJSON_free(out);
    }
    cJSON_Delete(hello);
}

// ---------------- Tasks ----------------
static void app_gate_state_task(void* arg)
{
    (void)arg;

    int raw_now = gpio_get_level(GATE_STATE_PIN);
    int last_raw = raw_now;
    int stable_state = raw_now;
    uint64_t last_change_ms = app_now_ms();

    s_last_gate_level = stable_state;
    app_publish_gate_state_level(stable_state);

    for (;;) {
        if (!s_app_running || app_ota_or_validation_active()) {
            app_all_outputs_off();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        raw_now = gpio_get_level(GATE_STATE_PIN);

        if (raw_now != last_raw) {
            last_raw = raw_now;
            last_change_ms = app_now_ms();
            ESP_LOGI(TAG_APP, "Gate raw change detected -> %d, waiting debounce", raw_now);
        }

        if ((last_raw != stable_state) &&
            ((app_now_ms() - last_change_ms) >= GATE_DEBOUNCE_MS)) {

            stable_state = last_raw;
            s_last_gate_level = stable_state;
            app_publish_gate_state_level(stable_state);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void app_rx_worker_task(void* arg)
{
    (void)arg;
    app_rx_msg_t m;

    for (;;) {
        if (!s_app_running || !s_accept_commands || app_ota_or_validation_active()) {
            app_all_outputs_off();
            vTaskDelay(pdMS_TO_TICKS(300));
            continue;
        }

        if (xQueueReceive(s_rx_q, &m, pdMS_TO_TICKS(100)) == pdTRUE) {

            if (!s_accept_commands || app_ota_or_validation_active()) {
                ESP_LOGW(TAG_APP, "Command ignored because app is not accepting commands");
                app_all_outputs_off();
                continue;
            }

            ESP_LOGI(TAG_APP, "RX(worker): %s", m.payload);

            // ESP32-C3-safe command parsing: no cJSON_Parse() here.
            int v;

            v = app_json_get_int_field(m.payload, "Open_Command");
            if (v >= 0) {
                gpio_set_level(OPEN_CMD_PIN, v ? 1 : 0);
                ESP_LOGI(TAG_APP, "Open_Command -> %s", v ? "HIGH" : "LOW");
            }

            v = app_json_get_int_field(m.payload, "Close_Command");
            if (v >= 0) {
                gpio_set_level(CLOSE_CMD_PIN, v ? 1 : 0);
                ESP_LOGI(TAG_APP, "Close_Command -> %s", v ? "HIGH" : "LOW");
            }

            v = app_json_get_int_field(m.payload, "Stop_Command");
            if (v >= 0) {
                gpio_set_level(STOP_CMD_PIN, v ? 1 : 0);
                ESP_LOGI(TAG_APP, "Stop_Command -> %s", v ? "HIGH" : "LOW");
            }
        }
    }
}

// ---------------- Part A lifecycle callbacks ----------------
static bool app_self_test(void)
{
    app_gate_gpio_init();
    app_all_outputs_off();
    vTaskDelay(pdMS_TO_TICKS(50));

    bool ok = app_outputs_are_off();
    ESP_LOGI(TAG_APP, "App self-test: outputs_off=%d", (int)ok);
    return ok;
}

static bool app_prepare_for_ota(void)
{
    ESP_LOGW(TAG_APP, "Preparing gate app for OTA safe-stop");

    s_accept_commands = false;

    if (s_rx_q) {
        xQueueReset(s_rx_q);
    }

    app_gate_gpio_init();
    app_all_outputs_off();
    vTaskDelay(pdMS_TO_TICKS(100));

    bool ok = app_outputs_are_off();
    ESP_LOGI(TAG_APP, "OTA safe-stop confirmation: outputs_off=%d", (int)ok);
    return ok;
}

static void app_stop(void)
{
    ESP_LOGW(TAG_APP, "Stopping gate app");

    s_accept_commands = false;
    s_app_running = false;

    if (s_rx_q) {
        xQueueReset(s_rx_q);
    }

    app_all_outputs_off();

    if (s_rx_worker_handle) {
        TaskHandle_t h = s_rx_worker_handle;
        s_rx_worker_handle = NULL;
        vTaskDelete(h);
    }

    if (s_gate_state_handle) {
        TaskHandle_t h = s_gate_state_handle;
        s_gate_state_handle = NULL;
        vTaskDelete(h);
    }

    app_all_outputs_off();
}

static void app_start(void)
{
    if (s_app_running) return;
    if (app_ota_or_validation_active()) return;

    /*
     * OTA-safe production lock point:
     * Part A calls app_start() only when the product app is allowed to run.
     * During OTA rollback validation, app_ota_or_validation_active() remains true,
     * so this lock will not run before Part A marks the OTA image valid.
     */
    app_flash_encryption_release_lock_once();

    ESP_LOGI(TAG_APP, "Starting gate app after Part A approval");

    app_gate_gpio_init();
    app_all_outputs_off();

    if (!s_rx_q) {
        s_rx_q = xQueueCreate(10, sizeof(app_rx_msg_t));
        if (!s_rx_q) {
            ESP_LOGE(TAG_APP, "Failed to create RX queue");
            return;
        }
    } else {
        xQueueReset(s_rx_q);
    }

    s_app_running = true;
    s_accept_commands = true;

    if (!s_rx_worker_handle) {
        xTaskCreate(app_rx_worker_task, "app_rx_worker", 4096, NULL, 5, &s_rx_worker_handle);
    }

    if (!s_gate_state_handle) {
        xTaskCreate(app_gate_state_task, "app_gate_state", 4096, NULL, 5, &s_gate_state_handle);
    }

    if (px_mqtt_is_connected()) {
        app_publish_boot_msg();
    }
}

// MQTT callback from Part A. Keep it lightweight.
static void app_on_mqtt_rx(const char* topic, const char* json)
{
    ESP_LOGI(TAG_APP, "RX on %s: %s", topic, json ? json : "(null)");

    if (!s_rx_q || !json) return;

    if (!s_app_running || !s_accept_commands || app_ota_or_validation_active()) {
        ESP_LOGW(TAG_APP, "Command ignored because app is stopped or OTA validation is active");
        app_all_outputs_off();
        return;
    }

    app_rx_msg_t m = {0};
    strlcpy(m.payload, json, sizeof(m.payload));

    if (xQueueSend(s_rx_q, &m, 0) != pdTRUE) {
        ESP_LOGW(TAG_APP, "RX queue full -> dropped");
    }
}

void app_main(void)
{
    ESP_LOGI(TAG_APP, "=== Pearlexa Connect (Universal Part A + Gate Part B) ===");

    /*
     * Do NOT call app_flash_encryption_release_lock_once() here for OTA delivery.
     * It must run from app_start(), after Part A has completed OTA validation.
     */

    const px_cfg_t cfg = {
        // NULL means Part A will read deviceSerial/authNumber from NVS.
        .device_serial           = NULL,
        .auth_number             = NULL,
        .mqtt_host               = APP_MQTT_HOST,
        .mqtt_tls_port           = APP_TLS_PORT,
        .mqtt_wss_uri            = APP_WSS_URI,
        .wifi_strength_period_ms = APP_WIFI_STRENGTH_PERIOD_MS,

        // OTA
        .firmware_version        = APP_FW_VERSION,
        .ota_version_url         = APP_OTA_VERSION_URL,
        .ota_check_on_boot       = false,
        .ota_check_delay_ms      = 10000,
        .ota_periodic_check_ms   = 0,

        // Optional universal metadata protection
        .hardware_model          = APP_HW_MODEL,
        .device_type             = APP_DEVICE_TYPE,
        .ota_max_attempts        = 3,
    };

    px_init(&cfg);

    // Register product lifecycle callbacks before px_start().
    px_set_app_start_callback(app_start);
    px_set_app_stop_callback(app_stop);
    px_set_ota_prepare_callback(app_prepare_for_ota);
    px_set_app_self_test_callback(app_self_test);
    px_set_message_callback(app_on_mqtt_rx);

    px_start();

    if (px_is_config_mode()) {
        ESP_LOGW(TAG_APP, "Config/BLE mode active -> Part A will not start gate app.");
        return;
    }

    /*
     * Do not start gate tasks here.
     * Part A will call app_start() when it is safe:
     *   - normal boot: after Wi-Fi/MQTT path is ready according to Part A
     *   - first OTA boot: only after rollback validation passes
     */
}