// [B] APP IMPLEMENTATION – Gate control example for Intelligent Universal Part A
//  ✅ Part B remains product-specific
//  ✅ Part A remains universal and controls OTA/app lifecycle
//  ✅ App does NOT start itself immediately after px_start()
//  ✅ Part A starts Part B only after normal boot or after OTA validation
//  ✅ Safe-stop callback confirms all gate outputs are LOW before OTA flashing
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
#include "driver/gpio.h"
#include "cJSON.h"

#include "pearlexa_connect.h"

// ======= App config you fill in (serial, tokens, endpoints) =======
static const char* APP_DEVICE_SERIAL = "SN:G1005005";
static const char* APP_AUTH_NUMBER   = "vow9kltt9vXXQfgvZ49fK8d3TaLTUg7XIUb1SE0VT4QKFxjippcai2DFx78EHzRh";
static const char* APP_MQTT_HOST     = "mqttserver1.pearlexa.cloud";
static const int   APP_TLS_PORT      = 8883;
static const char* APP_WSS_URI       = "wss://mqttserver1.pearlexa.cloud/mqtt";
static const int   APP_WIFI_STRENGTH_PERIOD_MS = 60000;

// OTA
static const char* APP_FW_VERSION = "1.0.0";
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

#define OPEN_CMD_PIN            16
#define CLOSE_CMD_PIN           17
#define STOP_CMD_PIN            18
#define GATE_STATE_PIN          19

#define GATE_DEBOUNCE_MS        1000
#define GATE_CMD_PULSE_MS        500
#define GATE_CMD_GAP_MS          50

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

static int json_as_int(cJSON* n)
{
    if (!n) return -1;
    if (cJSON_IsNumber(n)) return (int)n->valuedouble;
    if (cJSON_IsString(n) && n->valuestring) return atoi(n->valuestring);
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

static void app_pulse_gate_output(gpio_num_t pin, const char* name)
{
    if (!s_gpio_ready || !s_app_running || !s_accept_commands) return;

    // Fail-safe behavior: only one command output can ever be active.
    app_all_outputs_off();
    vTaskDelay(pdMS_TO_TICKS(GATE_CMD_GAP_MS));

    gpio_set_level(pin, 1);
    ESP_LOGI(TAG_APP, "%s pulse HIGH", name);

    vTaskDelay(pdMS_TO_TICKS(GATE_CMD_PULSE_MS));

    gpio_set_level(pin, 0);
    ESP_LOGI(TAG_APP, "%s pulse LOW", name);
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

            cJSON* root = cJSON_Parse(m.payload);
            if (!root) {
                ESP_LOGW(TAG_APP, "Bad JSON (worker)");
                continue;
            }

            int open_cmd  = json_as_int(cJSON_GetObjectItem(root, "Open_Command"));
            int close_cmd = json_as_int(cJSON_GetObjectItem(root, "Close_Command"));
            int stop_cmd  = json_as_int(cJSON_GetObjectItem(root, "Stop_Command"));

            bool open_req  = (open_cmd == 1);
            bool close_req = (close_cmd == 1);
            bool stop_req  = (stop_cmd == 1);

            int request_count = (open_req ? 1 : 0) +
                                (close_req ? 1 : 0) +
                                (stop_req ? 1 : 0);

            if (request_count > 1) {
                ESP_LOGW(TAG_APP, "Rejected unsafe simultaneous gate commands");
                app_all_outputs_off();
            } else if (stop_req) {
                app_pulse_gate_output(STOP_CMD_PIN, "Stop_Command");
            } else if (open_req) {
                app_pulse_gate_output(OPEN_CMD_PIN, "Open_Command");
            } else if (close_req) {
                app_pulse_gate_output(CLOSE_CMD_PIN, "Close_Command");
            } else if (open_cmd == 0 || close_cmd == 0 || stop_cmd == 0) {
                // Safe idle command. Pulsed outputs should already be LOW,
                // but force LOW in case the command comes after a reset or fault.
                app_all_outputs_off();
            }

            cJSON_Delete(root);
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
    ESP_LOGI(TAG_APP, "=== Pearlexa Gate app using pearlexa_connect component ===");

    const px_cfg_t cfg = {
        .device_serial           = APP_DEVICE_SERIAL,
        .auth_number             = APP_AUTH_NUMBER,
        .mqtt_host               = APP_MQTT_HOST,
        .mqtt_tls_port           = APP_TLS_PORT,
        .mqtt_wss_uri            = APP_WSS_URI,
        .wifi_strength_period_ms = APP_WIFI_STRENGTH_PERIOD_MS,

        // Wi-Fi roaming
        .wifi_roaming_enable         = true,
        .wifi_roam_scan_period_ms    = 60000,
        .wifi_roam_weak_rssi         = -75,
        .wifi_roam_margin_db         = 12,
        .wifi_roam_min_switch_gap_ms = 300000,

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
     *   - normal boot: after Wi-Fi + MQTT are connected
     *   - first OTA boot: only after rollback validation passes
     */
}
