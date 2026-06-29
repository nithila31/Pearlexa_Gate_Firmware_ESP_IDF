// =================================================================================
// [A] Pearlexa_Connect "library" (UNIVERSAL, no pins) — FULL PART A WITH OTA - stable Wi-Fi/MQTT
//
// Added OTA support:
//  - automatic OTA check only after Wi-Fi + MQTT are connected and stable
//  - version.json download over HTTPS using ESP certificate bundle
//  - semantic version comparison
//  - .bin download and install using esp_https_ota()
//  - rollback-safe OTA confirmation only after stable Wi-Fi + MQTT after reboot
//  - optional MQTT command trigger: {"Firmware_Update":1}
//
// Part B must provide OTA fields in px_cfg_t:
//  .firmware_version = APP_FW_VERSION,
//  .ota_version_url = APP_OTA_VERSION_URL,
//  .ota_check_on_boot = true,
//  .ota_check_delay_ms = 10000,
//  .ota_periodic_check_ms = 0,
//
// GitHub/raw version.json example:
// {
//   "version": "1.0.1",
//   "bin_url": "https://raw.githubusercontent.com/YOUR_USERNAME/pearlexa-ota-test/main/Pearlexa-BP5758D-v1.0.1.bin",
//   "force": false
// }
//
// CMake note: if your main/CMakeLists.txt uses REQUIRES, add:
// esp_https_ota esp_http_client app_update esp-tls esp_http_server
// =================================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "esp_err.h"
#include "esp_mac.h"

#include "nvs_flash.h"
#include "nvs.h"

#include "esp_netif.h"
#include "esp_netif_ip_addr.h"
#include "esp_wifi.h"

#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_http_server.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "cJSON.h"

#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_uuid.h"
#include "os/os_mbuf.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#define PX_TAG "Pearlexa"

// ---------- Library Config ----------
typedef struct {
    const char* device_serial;
    const char* auth_number;

    const char* mqtt_host;
    int         mqtt_tls_port;
    const char* mqtt_wss_uri;

    int wifi_strength_period_ms;

    // ---------- OTA Config ----------
    // Example firmware_version: "1.0.0"
    // Example ota_version_url: "https://raw.githubusercontent.com/user/repo/main/version.json"
    const char* firmware_version;
    const char* ota_version_url;
    bool        ota_check_on_boot;
    int         ota_check_delay_ms;
    int         ota_periodic_check_ms;   // 0 disables periodic checking

    // Optional universal product metadata. Leave NULL/0 if not needed.
    // If version.json includes "hardware" or "model", Part A can reject wrong firmware.
    const char* hardware_model;          // example: "PX-SG100-DC250-24-X-V1"
    const char* device_type;             // example: "gate_controller" / "smart_bulb"
    int         ota_max_attempts;         // 0 = default 3 attempts per target version
} px_cfg_t;

typedef void (*px_msg_cb_t)(const char* topic, const char* json);
typedef void (*px_app_void_cb_t)(void);
typedef bool (*px_app_bool_cb_t)(void);

// ---------- Library API ----------
void px_init(const px_cfg_t* cfg);
void px_start(void);
void px_publish_json(const char* json);
void px_set_message_callback(px_msg_cb_t cb);
void px_force_ble_mode_and_restart(void);
void px_ota_check_now(void);
int  px_wifi_rssi(void);
bool px_is_config_mode(void);
bool px_boot_is_config_mode(void);
bool px_mqtt_is_connected(void);
bool px_wait_mqtt_connected(int timeout_ms);
bool px_ota_update_is_in_progress(void);
bool px_ota_first_boot_validation_pending(void);
bool px_app_is_started(void);
void px_set_app_start_callback(px_app_void_cb_t cb);
void px_set_app_stop_callback(px_app_void_cb_t cb);
void px_set_ota_prepare_callback(px_app_bool_cb_t cb);
void px_set_app_self_test_callback(px_app_bool_cb_t cb);
// ---------- Internal consts ----------
#define PX_NVS_NS            "px"
#define PX_KEY_SSID          "wifiSSID"
#define PX_KEY_PASS          "wifiPassword"
#define PX_KEY_CFG_MODE      "configMode"
#define PX_KEY_RST_COUNT     "restartCounter"
#define PX_KEY_CFG_LOCK      "cfgLock"
#define PX_KEY_OTA_PHASE     "otaPhase"
#define PX_KEY_OTA_TARGET    "otaTarget"
#define PX_KEY_OTA_ATTEMPT   "otaAttempt"
#define PX_KEY_OTA_FAIL      "otaFail"

// Per-device identity keys stored in the existing px namespace.
// These replace hardcoded APP_DEVICE_SERIAL / APP_AUTH_NUMBER in production.
#define PX_KEY_DEVICE_SERIAL "deviceSerial"
#define PX_KEY_AUTH_NUMBER   "authNumber"

// Factory identity setup mode.
// If deviceSerial/authNumber are missing, Part A connects to this factory Wi-Fi
// using a static IP and exposes: http://192.168.1.10
#ifndef PX_FACTORY_WIFI_SSID
#define PX_FACTORY_WIFI_SSID      "Dream_theatre"
#endif

#ifndef PX_FACTORY_WIFI_PASSWORD
#define PX_FACTORY_WIFI_PASSWORD  "Justlogin#365"
#endif

#ifndef PX_FACTORY_STATIC_IP_A
#define PX_FACTORY_STATIC_IP_A 192
#endif
#ifndef PX_FACTORY_STATIC_IP_B
#define PX_FACTORY_STATIC_IP_B 168
#endif
#ifndef PX_FACTORY_STATIC_IP_C
#define PX_FACTORY_STATIC_IP_C 1
#endif
#ifndef PX_FACTORY_STATIC_IP_D
#define PX_FACTORY_STATIC_IP_D 10
#endif

#ifndef PX_FACTORY_GATEWAY_A
#define PX_FACTORY_GATEWAY_A 192
#endif
#ifndef PX_FACTORY_GATEWAY_B
#define PX_FACTORY_GATEWAY_B 168
#endif
#ifndef PX_FACTORY_GATEWAY_C
#define PX_FACTORY_GATEWAY_C 1
#endif
#ifndef PX_FACTORY_GATEWAY_D
#define PX_FACTORY_GATEWAY_D 1
#endif

#ifndef PX_FACTORY_NETMASK_A
#define PX_FACTORY_NETMASK_A 255
#endif
#ifndef PX_FACTORY_NETMASK_B
#define PX_FACTORY_NETMASK_B 255
#endif
#ifndef PX_FACTORY_NETMASK_C
#define PX_FACTORY_NETMASK_C 255
#endif
#ifndef PX_FACTORY_NETMASK_D
#define PX_FACTORY_NETMASK_D 0
#endif

#ifndef PX_FACTORY_DNS_A
#define PX_FACTORY_DNS_A 1
#endif
#ifndef PX_FACTORY_DNS_B
#define PX_FACTORY_DNS_B 1
#endif
#ifndef PX_FACTORY_DNS_C
#define PX_FACTORY_DNS_C 1
#endif
#ifndef PX_FACTORY_DNS_D
#define PX_FACTORY_DNS_D 1
#endif

#define PX_FACTORY_SETUP_URL "http://192.168.1.10"

// 1 = device identity must come from NVS/factory setup, even if Part B still
// contains old hardcoded serial/token values. This helps enforce one common firmware.
#ifndef PX_FORCE_NVS_IDENTITY
#define PX_FORCE_NVS_IDENTITY 1
#endif

#define PX_FACTORY_HTTP_PORT             80
#define PX_FACTORY_IDENTITY_POST_MAX    512
#define PX_FACTORY_RECONNECT_PERIOD_MS 5000

#define PX_KEEPALIVE_SEC               45
#define PX_WIFI_CONNECT_TIMEOUT_MS  10000
#define PX_WIFI_RECONNECT_PERIOD_MS 5000

/*
 * WSS-only MQTT reliability policy:
 * - Keep one esp-mqtt client alive and let esp-mqtt do its own reconnects.
 * - Do not destroy/recreate the MQTT client on every WebSocket EOF.
 * - Use a unique MQTT client ID suffix so ESP32 and ESP32-C3 test boards with
 *   the same serial number do not kick each other off the broker.
 */
#define PX_ATTEMPT_TIMEOUT_MS       900000
#define PX_ATTEMPT_COOLDOWN_MS        5000
#define PX_MQTT_RECONNECT_TIMEOUT_MS  10000
#define PX_MQTT_CLIENT_ID_ADD_MAC        1


// ESP32-C3 optimized startup:
// - Start product application after Wi-Fi is ready, without waiting for MQTT.
// - Start the product app immediately after Wi-Fi so gate logic is available fast.
// - Keep MQTT in the background with a reliable timeout.
// - Use WSS only for both ESP32 and ESP32-C3. Native TLS is intentionally disabled.
// - Use a moderate MQTT keepalive so WSS/proxy paths are not stressed by aggressive PING timing.
#define PX_FAST_APP_START_AFTER_WIFI      1
#define PX_PREFER_TLS_FIRST              0   // kept for compatibility; WSS-only runtime ignores TLS
#define PX_MQTT_CONNECT_TIMEOUT_MS   20000
#define PX_WIFI_ASSEMBLY_TIMEOUT_MS   1500
#define PX_STATUS_JSON_MAX             512
#define PX_WIFI_JSON_BUF_MAX           384
#define PX_WIFI_SCAN_MAX_RESULTS        15
#define PX_STATUS_REPLAY_PERIOD_MS     700
#define PX_COMMIT_TIMEOUT_MS         20000
#define PX_SCAN_RESULT_REPLAY_DELAY_MS  90
#define PX_SCAN_CACHE_GRACE_MS        12000

#define PX_OTA_VERSION_JSON_MAX       1024
#define PX_OTA_URL_MAX                 384
#define PX_OTA_TARGET_MAX               40
#define PX_OTA_DEFAULT_DELAY_MS      10000
#define PX_OTA_HTTP_TIMEOUT_MS       20000
#define PX_OTA_BIN_TIMEOUT_MS        30000
#define PX_OTA_PERIOD_MIN_MS         60000
#define PX_OTA_STABLE_WAIT_MS        10000
#define PX_OTA_STATUS_SETTLE_MS        800
#define PX_OTA_DEFAULT_MAX_ATTEMPTS      3

#define PX_MQTT_WIFI_FAULT_SUPPRESS_MS     8000
#define PX_MQTT_WIFI_SUSPEND_GRACE_MS    8000


/*
 * Wi-Fi + MQTT intelligence layer.
 * - Wi-Fi connected means GOT_IP, not only associated.
 */
#define PX_WIFI_WAIT_IP_AFTER_ASSOC_MS      30000
#define PX_WIFI_IP_WAIT_LOG_PERIOD_MS        2000
#define PX_MQTT_FAULT_WINDOW_MS            120000

/*
 * Wi-Fi/MQTT cross-intelligence:
 * A MQTT PING timeout can happen first while the Wi-Fi driver still holds an IP
 * bit during beacon timeout / AP probing. Do not immediately blame MQTT. Hold
 * the MQTT fault briefly; if Wi-Fi disconnects in this window, classify it as a
 * Wi-Fi root fault. Only count it as a real MQTT fault if Wi-Fi remains IP-ready.
 */
#define PX_MQTT_FAULT_CLASSIFY_DELAY_MS     20000
#define PX_WIFI_RECENT_LOSS_CLASSIFY_MS     12000


/*
 * RSSI averaging:
 * A single RSSI read can jump several dB. Use a few samples for published
 * Wi-Fi strength publishing.
 */
#define PX_WIFI_RSSI_SAMPLE_COUNT             5
#define PX_WIFI_RSSI_SAMPLE_DELAY_MS         80

typedef enum { PX_TRANS_WSS=0, PX_TRANS_TLS=1 } px_transport_t;
typedef enum { PX_ST_DISCONNECTED=0, PX_ST_CONNECTING=1, PX_ST_CONNECTED=2 } px_conn_state_t;

typedef enum {
    PX_NET_FAULT_NONE = 0,
    PX_NET_FAULT_WIFI_DISCONNECT,
    PX_NET_FAULT_WIFI_BEACON_TIMEOUT,
    PX_NET_FAULT_MQTT_TRANSPORT,
} px_net_fault_t;


typedef enum {
    PX_OTA_PHASE_IDLE = 0,
    PX_OTA_PHASE_CHECKING_VERSION = 1,
    PX_OTA_PHASE_UPDATE_AVAILABLE = 2,
    PX_OTA_PHASE_PREPARE_SAFE_STOP = 3,
    PX_OTA_PHASE_SAFE_STOP_CONFIRMED = 4,
    PX_OTA_PHASE_DOWNLOADING = 5,
    PX_OTA_PHASE_REBOOTING_TO_NEW_APP = 6,
    PX_OTA_PHASE_FIRST_BOOT_VALIDATION = 7,
    PX_OTA_PHASE_VALIDATED = 8,
    PX_OTA_PHASE_FAILED = 9,
    PX_OTA_PHASE_ROLLED_BACK = 10,
} px_ota_phase_t;

typedef enum {
    PX_PROV_CMD_NONE = 0,
    PX_PROV_CMD_SCAN_WIFI = 1,
    PX_PROV_CMD_SET_WIFI = 2,
    PX_PROV_CMD_COMMIT_WIFI = 3,
    PX_PROV_CMD_RESEND_STATUS = 4,
} px_prov_cmd_type_t;

typedef struct {
    px_prov_cmd_type_t type;
    int req_id;
    char ssid[64];
    char pass[64];
} px_prov_cmd_t;

typedef enum {
    PX_TXN_IDLE = 0,
    PX_TXN_READY,
    PX_TXN_SCAN_STARTED,
    PX_TXN_SCAN_DONE,
    PX_TXN_WIFI_PAYLOAD_RECEIVED,
    PX_TXN_WIFI_JSON_VALID,
    PX_TXN_SAVING_WIFI,
    PX_TXN_SAVED_WAITING_COMMIT,
    PX_TXN_COMMIT_RECEIVED,
    PX_TXN_RESTARTING,
    PX_TXN_ERROR
} px_txn_stage_t;

// ---------- Global state ----------
static px_cfg_t G = {0};

static nvs_handle_t px_nvs = 0;
static EventGroupHandle_t px_wifi_group = NULL;
static const int PXWIFI_CONNECTED_BIT = BIT0;
static const int PXWIFI_FAIL_BIT      = BIT1;

static char px_topic_tx[64], px_topic_rx[64], px_topic_status[64];
static char px_mqtt_client_id[96] = {0};

// NVS-loaded identity buffers. G.device_serial and G.auth_number point to these
// when Part B passes NULL/empty values.
static char px_device_serial_nvs[64] = {0};
static char px_auth_number_nvs[128] = {0};
static bool px_identity_ready = false;
static bool px_factory_identity_mode = false;
static httpd_handle_t px_factory_httpd = NULL;
static uint64_t px_factory_last_reconnect_ms = 0;

static px_transport_t px_current_transport = PX_TRANS_WSS;
static px_conn_state_t px_ws_state = PX_ST_DISCONNECTED, px_tls_state = PX_ST_DISCONNECTED;
static uint64_t px_attempt_start_us = 0, px_last_flip_us = 0;

static esp_mqtt_client_handle_t px_mqtt = NULL;
static bool px_is_ble_mode_flag = false;
static px_msg_cb_t px_user_msg_cb = NULL;

static uint64_t px_last_wifi_ms = 0;
static uint64_t px_last_wifi_reconnect_ms = 0;
static int32_t px_last_wifi_disconnect_reason = -1;
static uint64_t px_last_wifi_disconnect_ms = 0;
static uint64_t px_wifi_ip_lost_since_ms = 0;
static uint64_t px_mqtt_wifi_fault_suppress_until_ms = 0;
static px_net_fault_t px_last_net_fault = PX_NET_FAULT_NONE;
static uint64_t px_wifi_associated_ms = 0;
static uint64_t px_wifi_last_wait_ip_log_ms = 0;

static int px_mqtt_transport_fault_count = 0;
static uint64_t px_mqtt_transport_fault_window_ms = 0;
static uint64_t px_mqtt_last_connected_ms = 0;
static uint64_t px_mqtt_last_fault_ms = 0;

static volatile bool px_mqtt_suspended_for_wifi = false;
static uint64_t px_mqtt_fault_pending_until_ms = 0;
static uint64_t px_mqtt_fault_pending_started_ms = 0;



static char     px_wifi_buf[PX_WIFI_JSON_BUF_MAX];
static size_t   px_wifi_len = 0;
static uint64_t px_wifi_last_ms = 0;

static char px_devname[64] = {0};
static uint8_t px_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static uint16_t px_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t px_status_val_handle = 0;
static bool px_status_subscribed = false;
static bool px_ble_client_connected = false;
static char px_status_json[PX_STATUS_JSON_MAX] = "{\"stage\":\"idle\",\"protocol\":\"v3\",\"statusSeq\":0}";

static char px_hostname[64] = {0};
static esp_netif_t* px_netif = NULL;
static esp_timer_handle_t px_rst_clear_timer = NULL;

static bool px_wifi_stack_ready = false;
static bool px_wifi_started = false;
static bool px_wifi_handlers_registered = false;

static QueueHandle_t px_prov_queue = NULL;

// BLE JSON parsing is moved out of the NimBLE GATT callback for ESP32-C3 safety.
typedef struct {
    uint16_t len;
    char json[PX_WIFI_JSON_BUF_MAX];
} px_ble_json_msg_t;

static QueueHandle_t px_ble_json_queue = NULL;
static TaskHandle_t px_ble_json_task_handle = NULL;

// ---------- OTA state ----------
static volatile bool px_ota_task_running = false;
static volatile bool px_ota_update_in_progress_flag = false;
static volatile bool px_ota_checked_once = false;
static volatile bool px_ota_manual_request = false;

static bool px_ota_pending_verify = false;
static char px_last_ota_state_sent[40] = {0};
static char px_last_ota_target_sent[40] = {0};
static bool px_last_ota_progress_sent = false;
static bool px_last_ota_progress_valid = false;

static uint64_t px_ota_last_check_ms = 0;
static uint64_t px_ota_online_stable_since_ms = 0;

// ---------- Universal app lifecycle callbacks ----------
static px_app_void_cb_t px_app_start_cb = NULL;
static px_app_void_cb_t px_app_stop_cb = NULL;
static px_app_bool_cb_t px_app_prepare_ota_cb = NULL;
static px_app_bool_cb_t px_app_self_test_cb = NULL;
static volatile bool px_app_started_flag = false;
static volatile bool px_app_start_allowed = true;
static TaskHandle_t px_app_task_handle = NULL;

#define PX_APP_TASK_STACK_SIZE 8192
#define PX_APP_TASK_PRIO       5

// ---------- Provisioning txn state ----------
static int px_status_seq = 0;
static int px_active_req_id = -1;
static px_txn_stage_t px_active_stage = PX_TXN_IDLE;
static char px_active_ssid[64] = {0};
static char px_active_pass[64] = {0};
static char px_active_code[32] = {0};
static char px_active_msg[128] = {0};
static uint64_t px_active_stage_ms = 0;
static uint64_t px_last_status_replay_ms = 0;
static bool px_waiting_commit = false;
static uint64_t px_waiting_commit_resume_ms = 0;

// ---------- Scan cache ----------
typedef struct {
    bool valid;
    int req_id;
    uint16_t count;
    wifi_ap_record_t aps[PX_WIFI_SCAN_MAX_RESULTS];
    uint64_t completed_ms;
} px_scan_cache_t;

static px_scan_cache_t px_scan_cache = {0};

// ---------- Internal forward declarations ----------
static void px_mqtt_stop_if_any(void);
static bool px_wifi_is_connected(void);
static bool px_ota_system_online_stable_now(void);
static void px_ota_confirm_running_app_if_stable(void);
static bool px_ota_running_app_is_pending_verify(void);
static void px_app_start_if_allowed(void);
static bool px_app_prepare_safe_stop_for_ota(void);
static bool px_app_self_test_ok(void);
static void px_ota_set_phase(px_ota_phase_t phase);
static px_ota_phase_t px_ota_get_phase(void);
static void px_publish_ota_status(const char *state, const char *server_version, const char *message);
static void px_publish_firmware_info_now(void);
static void px_mqtt_transport_fault_note(bool wifi_root_fault);
static void px_mqtt_transport_connected_note(void);
static void px_mqtt_suspend_until_wifi_ready(void);
static void px_mqtt_fault_deferred_check(void);
static void px_wifi_stack_ensure_ready(void);
static bool px_identity_load_or_bind_from_nvs(void);
static void px_factory_identity_start(void);
static void px_factory_identity_reconnect_if_needed(void);


// ---------- Small utils ----------
static inline uint64_t px_nowUs(void){ return esp_timer_get_time(); }
static inline uint64_t px_nowMs(void){ return px_nowUs()/1000ULL; }
static inline bool px_ok_to_flip(void){
    return (px_nowUs()-px_last_flip_us) >= (uint64_t)PX_ATTEMPT_COOLDOWN_MS*1000ULL;
}

static void px_wifi_accum_reset(void){
    px_wifi_len = 0;
    px_wifi_buf[0] = 0;
}

static bool px_wifi_accum_add(const uint8_t *data, uint16_t len) {
    if (len == 0) return true;
    if (len > 256) return false;
    if (px_wifi_len + len >= sizeof(px_wifi_buf)) return false;
    memcpy(px_wifi_buf + px_wifi_len, data, len);
    px_wifi_len += len;
    px_wifi_buf[px_wifi_len] = 0;
    px_wifi_last_ms = px_nowMs();
    return true;
}

static bool px_json_looks_complete(const char *s, size_t n) {
    if (!n) return false;
    int depth = 0;
    bool in_str = false, esc = false;
    size_t i0 = 0;

    while (i0 < n && (s[i0]==' '||s[i0]=='\r'||s[i0]=='\n'||s[i0]=='\t')) i0++;
    if (i0 >= n || s[i0] != '{') return false;

    for (size_t i = i0; i < n; i++) {
        char c = s[i];
        if (in_str) {
            if (esc){ esc = false; continue; }
            if (c == '\\'){ esc = true; continue; }
            if (c == '"'){ in_str = false; }
            continue;
        }
        if (c == '"'){ in_str = true; continue; }
        if (c == '{') depth++;
        else if (c == '}') {
            depth--;
            if (depth == 0 && i == n - 1) return true;
        }
    }
    return false;
}

static const char* px_signal_label_from_rssi(int rssi) {
    if (rssi >= -50) return "Excellent";
    if (rssi >= -60) return "Very Good";
    if (rssi >= -70) return "Good";
    if (rssi >= -80) return "Fair";
    if (rssi >= -90) return "Poor";
    return "Very Weak";
}

static const char* px_auth_mode_str(wifi_auth_mode_t mode) {
    switch (mode) {
        case WIFI_AUTH_OPEN: return "OPEN";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA_PSK";
        case WIFI_AUTH_WPA2_PSK: return "WPA2_PSK";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA_WPA2_PSK";
        case WIFI_AUTH_WPA2_ENTERPRISE: return "WPA2_ENTERPRISE";
        case WIFI_AUTH_WPA3_PSK: return "WPA3_PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2_WPA3_PSK";
        case WIFI_AUTH_WAPI_PSK: return "WAPI_PSK";
        default: return "UNKNOWN";
    }
}



static void px_wifi_build_config_from_ap(wifi_config_t *w,
                                         const char *ssid,
                                         const char *pass,
                                         const wifi_ap_record_t *ap)
{
    (void)ap;

    if (!w) return;

    memset(w, 0, sizeof(*w));

    strlcpy((char *)w->sta.ssid, ssid ? ssid : "", sizeof(w->sta.ssid));
    strlcpy((char *)w->sta.password, pass ? pass : "", sizeof(w->sta.password));

    /*
     * Stable normal station connection:
     * Do not lock to a specific AP MAC/channel. Let the ESP-IDF Wi-Fi driver
     * associate normally using the saved SSID/password.
     */
    w->sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    w->sta.pmf_cfg.capable = true;
    w->sta.pmf_cfg.required = false;
    w->sta.channel = 0;

    ESP_LOGI(PX_TAG,
             "Wi-Fi config: SSID=%s AUTH>=%s PMF(cap=%d req=%d)",
             ssid ? ssid : "",
             px_auth_mode_str(w->sta.threshold.authmode),
             (int)w->sta.pmf_cfg.capable,
             (int)w->sta.pmf_cfg.required);
}

static const char* px_stage_name(px_txn_stage_t s) {
    switch (s) {
        case PX_TXN_IDLE: return "idle";
        case PX_TXN_READY: return "ready";
        case PX_TXN_SCAN_STARTED: return "wifi_scan_started";
        case PX_TXN_SCAN_DONE: return "wifi_scan_done";
        case PX_TXN_WIFI_PAYLOAD_RECEIVED: return "wifi_payload_received";
        case PX_TXN_WIFI_JSON_VALID: return "wifi_json_valid";
        case PX_TXN_SAVING_WIFI: return "saving_wifi";
        case PX_TXN_SAVED_WAITING_COMMIT: return "saved_waiting_commit";
        case PX_TXN_COMMIT_RECEIVED: return "commit_received";
        case PX_TXN_RESTARTING: return "restarting";
        case PX_TXN_ERROR: return "error";
        default: return "unknown";
    }
}

static bool px_url_is_https(const char *url) {
    return url && strncmp(url, "https://", 8) == 0;
}

static int px_read_uint_component(const char **p) {
    int v = 0;
    int digits = 0;
    const char *s = *p;

    while (*s && (*s < '0' || *s > '9')) s++;
    while (*s >= '0' && *s <= '9') {
        if (v < 1000000) v = (v * 10) + (*s - '0');
        s++;
        digits++;
    }

    *p = s;
    return digits ? v : 0;
}

static int px_version_compare(const char *server, const char *current) {
    if (!server || !current) return 0;

    const char *a = server;
    const char *b = current;

    for (int i = 0; i < 4; i++) {
        int va = px_read_uint_component(&a);
        int vb = px_read_uint_component(&b);

        if (va > vb) return 1;
        if (va < vb) return -1;

        if (*a == '.') a++;
        if (*b == '.') b++;
    }

    return 0;
}

// ---------- NVS ----------
static void px_nvs_init_open(void){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(nvs_open(PX_NVS_NS, NVS_READWRITE, &px_nvs));
}

static void px_nvs_set_bool(const char* key, bool v){
    nvs_set_u8(px_nvs, key, v ? 1 : 0);
    nvs_commit(px_nvs);
}

static bool px_nvs_get_bool(const char* key, bool deflt){
    uint8_t v = deflt ? 1 : 0;
    nvs_get_u8(px_nvs, key, &v);
    return v != 0;
}

static void px_nvs_get_strz(const char* key, char* out, size_t outlen){
    size_t req = 0;
    if (nvs_get_str(px_nvs, key, NULL, &req) != ESP_OK || !req){ out[0] = 0; return; }
    if (req > outlen) req = outlen;
    if (nvs_get_str(px_nvs, key, out, &req) != ESP_OK){ out[0] = 0; return; }
    out[outlen - 1] = 0;
}

static void px_nvs_set_str_safe(const char* key, const char* v){
    if (!v) v = "";
    nvs_set_str(px_nvs, key, v);
    nvs_commit(px_nvs);
}

static void px_nvs_set_int(const char* key, int v){
    nvs_set_i32(px_nvs, key, v);
    nvs_commit(px_nvs);
}

static int px_nvs_get_int(const char* key, int deflt){
    int32_t v = deflt;
    nvs_get_i32(px_nvs, key, &v);
    return (int)v;
}

static void px_rst_clear_timer_cb(void* arg){
    (void)arg;
    int c = px_nvs_get_int(PX_KEY_RST_COUNT, 0);
    if (c > 0) {
        ESP_LOGI(PX_TAG, "5s elapsed -> clearing rapid power-cycle counter (was %d)", c);
        px_nvs_set_int(PX_KEY_RST_COUNT, 0);
    }
}


// ---------- Universal App lifecycle ----------
static void px_app_task_entry(void *arg)
{
    (void)arg;

    ESP_LOGI(PX_TAG, "Part B application task started");

    if (px_app_start_cb) {
        px_app_start_cb();
    }

    /*
     * If Part B's start callback returns, keep the flag true but clear the
     * task handle. Most products should create their own tasks and return,
     * or run here with regular vTaskDelay()/yield calls.
     */
    ESP_LOGW(PX_TAG, "Part B start callback returned");
    px_app_task_handle = NULL;
    vTaskDelete(NULL);
}

static void px_app_start_if_allowed(void)
{
    if (px_is_ble_mode_flag) return;
    if (!px_app_start_allowed) return;
    if (px_ota_update_in_progress_flag) return;
    if (px_ota_pending_verify) return;       // Do not start product logic before OTA validation.
    if (px_app_started_flag) return;

    if (px_app_start_cb) {
        ESP_LOGI(PX_TAG, "Starting product application via Part B callback");

        px_app_started_flag = true;

        BaseType_t ok = xTaskCreatePinnedToCore(
            px_app_task_entry,
            "px_app_task",
            PX_APP_TASK_STACK_SIZE,
            NULL,
            PX_APP_TASK_PRIO,
            &px_app_task_handle,
            tskNO_AFFINITY
        );

        if (ok != pdPASS) {
            ESP_LOGE(PX_TAG, "Failed to create Part B application task");
            px_app_task_handle = NULL;
            px_app_started_flag = false;
        }
    }
}

static void px_app_stop_if_started(void)
{
    if (!px_app_started_flag) return;

    ESP_LOGI(PX_TAG, "Stopping product application via Part B callback");

    if (px_app_stop_cb) {
        px_app_stop_cb();
    }

    /*
     * Do not force-delete the Part B task here unless the Part B stop callback
     * cannot stop it cleanly. A forced delete can leave drivers/peripherals in
     * an unsafe state. Part B should exit/idle its own loop when stop is called.
     */
    px_app_started_flag = false;
}

static bool px_app_self_test_ok(void)
{
    if (esp_get_free_heap_size() < 25000) {
        ESP_LOGW(PX_TAG, "App self-test failed: low heap: %lu", (unsigned long)esp_get_free_heap_size());
        return false;
    }

    if (px_app_self_test_cb) {
        bool ok = px_app_self_test_cb();
        if (!ok) ESP_LOGW(PX_TAG, "Part B self-test callback returned false");
        return ok;
    }

    return true;
}

static bool px_app_prepare_safe_stop_for_ota(void)
{
    /*
     * Universal design:
     * Part A does not know whether this is a gate controller, bulb, sensor, PLC, etc.
     * Part A only asks Part B to enter a safe idle state and confirm it.
     */
    px_app_start_allowed = false;

    px_ota_set_phase(PX_OTA_PHASE_PREPARE_SAFE_STOP);
    px_publish_ota_status("preparing_safe_stop", NULL, "Requesting product application safe stop");
    vTaskDelay(pdMS_TO_TICKS(250));

    bool ok = true;

    if (px_app_prepare_ota_cb) {
        ok = px_app_prepare_ota_cb();
    }

    if (ok) {
        px_app_stop_if_started();
    }

    if (ok && px_app_self_test_cb) {
        ok = px_app_self_test_cb();
    }

    if (!ok) {
        px_ota_set_phase(PX_OTA_PHASE_FAILED);
        px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "SAFE_STOP_FAILED");
        px_publish_ota_status("failed", NULL, "Product application did not confirm safe stop");
        px_app_start_allowed = true;
        px_app_start_if_allowed();
        return false;
    }

    px_ota_set_phase(PX_OTA_PHASE_SAFE_STOP_CONFIRMED);
    px_publish_ota_status("safe_stop_confirmed", NULL, "Product application stopped safely");
    vTaskDelay(pdMS_TO_TICKS(PX_OTA_STATUS_SETTLE_MS));
    return true;
}

// ---------- OTA transaction persistence ----------
static void px_ota_set_phase(px_ota_phase_t phase)
{
    px_nvs_set_int(PX_KEY_OTA_PHASE, (int)phase);
}

static px_ota_phase_t px_ota_get_phase(void)
{
    int phase = px_nvs_get_int(PX_KEY_OTA_PHASE, PX_OTA_PHASE_IDLE);
    if (phase < PX_OTA_PHASE_IDLE || phase > PX_OTA_PHASE_ROLLED_BACK) {
        return PX_OTA_PHASE_IDLE;
    }
    return (px_ota_phase_t)phase;
}

static void px_ota_clear_transaction(void)
{
    px_ota_set_phase(PX_OTA_PHASE_IDLE);
    px_nvs_set_str_safe(PX_KEY_OTA_TARGET, "");
    px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "");
    px_nvs_set_int(PX_KEY_OTA_ATTEMPT, 0);
}

static int px_ota_max_attempts(void)
{
    return G.ota_max_attempts > 0 ? G.ota_max_attempts : PX_OTA_DEFAULT_MAX_ATTEMPTS;
}

static bool px_ota_target_attempt_blocked(const char *target_version, bool force)
{
    if (force) return false;
    if (!target_version || !target_version[0]) return false;

    char old_target[PX_OTA_TARGET_MAX] = {0};
    px_nvs_get_strz(PX_KEY_OTA_TARGET, old_target, sizeof(old_target));

    int attempt = px_nvs_get_int(PX_KEY_OTA_ATTEMPT, 0);
    if (strcmp(old_target, target_version) == 0 && attempt >= px_ota_max_attempts()) {
        ESP_LOGW(PX_TAG, "OTA target %s blocked after %d failed attempts", target_version, attempt);
        return true;
    }
    return false;
}

static void px_ota_record_attempt(const char *target_version)
{
    if (!target_version) target_version = "";

    char old_target[PX_OTA_TARGET_MAX] = {0};
    px_nvs_get_strz(PX_KEY_OTA_TARGET, old_target, sizeof(old_target));

    int attempt = px_nvs_get_int(PX_KEY_OTA_ATTEMPT, 0);
    if (strcmp(old_target, target_version) == 0) {
        attempt++;
    } else {
        attempt = 1;
        px_nvs_set_str_safe(PX_KEY_OTA_TARGET, target_version);
    }
    px_nvs_set_int(PX_KEY_OTA_ATTEMPT, attempt);
}

/* OTA metadata parsing is handled by px_ota_metadata_allowed_json() without cJSON_Parse(). */

// ---------- OTA boot confirmation ----------
static bool px_ota_running_app_is_pending_verify(void)
{
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t ota_state;

    if (running && esp_ota_get_state_partition(running, &ota_state) == ESP_OK) {
        return ota_state == ESP_OTA_IMG_PENDING_VERIFY;
    }
#endif
    return false;
}

static bool px_ota_system_online_stable_now(void)
{
    if (px_is_ble_mode_flag ||
        !px_wifi_is_connected() ||
        !px_mqtt_is_connected()) {

        px_ota_online_stable_since_ms = 0;
        return false;
    }

    uint64_t now = px_nowMs();

    if (px_ota_online_stable_since_ms == 0) {
        px_ota_online_stable_since_ms = now;
        return false;
    }

    int stable_wait_ms = G.ota_check_delay_ms > 0
                         ? G.ota_check_delay_ms
                         : PX_OTA_STABLE_WAIT_MS;

    if (stable_wait_ms < PX_OTA_STABLE_WAIT_MS) {
        stable_wait_ms = PX_OTA_STABLE_WAIT_MS;
    }

    return (now - px_ota_online_stable_since_ms) >= (uint64_t)stable_wait_ms;
}

static void px_ota_confirm_running_app_if_stable(void)
{
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    if (!px_ota_pending_verify) return;
    if (px_ota_update_in_progress_flag) return;

    /* During rollback-pending first boot, keep Part B stopped.
     * Validate only connectivity + universal/Part B self-test.
     */
    px_app_start_allowed = false;

    if (!px_ota_system_online_stable_now()) return;

    px_ota_set_phase(PX_OTA_PHASE_FIRST_BOOT_VALIDATION);
    px_publish_ota_status("first_boot_validation", NULL, "Validating new firmware before enabling application work");

    if (!px_app_self_test_ok()) {
        ESP_LOGW(PX_TAG, "OTA first boot validation not passed yet");
        return;
    }

    ESP_LOGI(PX_TAG, "OTA image verified after stable Wi-Fi + MQTT + app self-test. Marking app valid.");

    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        px_ota_pending_verify = false;
        px_ota_set_phase(PX_OTA_PHASE_VALIDATED);
        px_publish_ota_status("validated", NULL, "New firmware validated and rollback cancelled");
        px_ota_clear_transaction();
        px_app_start_allowed = true;
        ESP_LOGI(PX_TAG, "OTA rollback cancelled. Current app is now valid.");
        px_app_start_if_allowed();
    } else {
        ESP_LOGE(PX_TAG, "Failed to mark OTA app valid: %s", esp_err_to_name(err));
    }
#endif
}

// ---------- BLE status ----------
static void px_status_set_json(cJSON* root){
    if (!root) return;
    char* out = cJSON_PrintUnformatted(root);
    if (!out) {
        cJSON_Delete(root);
        return;
    }
    strlcpy(px_status_json, out, sizeof(px_status_json));
    cJSON_free(out);
    cJSON_Delete(root);
}

static void px_txn_set_state(px_txn_stage_t stage, int reqId, const char* code, const char* message) {
    px_active_stage = stage;
    px_active_req_id = reqId;
    px_active_stage_ms = px_nowMs();
    px_last_status_replay_ms = 0;

    strlcpy(px_active_code, code ? code : "", sizeof(px_active_code));
    strlcpy(px_active_msg, message ? message : "", sizeof(px_active_msg));
}

static void px_status_build_current_txn(void){
    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "stage", px_stage_name(px_active_stage));
    cJSON_AddStringToObject(root, "protocol", "v3");
    cJSON_AddNumberToObject(root, "statusSeq", ++px_status_seq);

    if (px_active_req_id >= 0) cJSON_AddNumberToObject(root, "reqId", px_active_req_id);
    if (px_active_code[0] != '\0') cJSON_AddStringToObject(root, "code", px_active_code);
    if (px_active_msg[0] != '\0') cJSON_AddStringToObject(root, "message", px_active_msg);

    px_status_set_json(root);
}

static void px_status_notify_now(void){
    if (px_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    if (px_status_val_handle == 0) return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(px_status_json, strlen(px_status_json));
    if (!om) {
        ESP_LOGW(PX_TAG, "BLE status notify skipped: mbuf allocation failed");
        return;
    }

    int rc = ble_gatts_notify_custom(
        px_conn_handle,
        px_status_val_handle,
        om
    );

    if (rc == 0) {
        ESP_LOGI(PX_TAG, "BLE STATUS TX: %s", px_status_json);
    } else {
        ESP_LOGW(PX_TAG, "BLE status notify rc=%d", rc);
    }
}

static void px_status_publish_current(void){
    px_status_build_current_txn();
    px_status_notify_now();
}

static void px_status_publish_stage(px_txn_stage_t stage, int reqId){
    px_txn_set_state(stage, reqId, "", "");
    px_status_publish_current();
}

static void px_status_publish_error(const char* code, const char* message, int reqId){
    px_txn_set_state(PX_TXN_ERROR, reqId, code ? code : "UNKNOWN", message ? message : "");
    px_status_publish_current();
}

static void px_status_build_scan_item(int reqId, const wifi_ap_record_t* ap){
    cJSON* root = cJSON_CreateObject();
    cJSON* item = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "stage", "wifi_scan_result");
    cJSON_AddStringToObject(root, "protocol", "v3");
    cJSON_AddNumberToObject(root, "statusSeq", ++px_status_seq);
    cJSON_AddNumberToObject(root, "reqId", reqId);

    cJSON_AddStringToObject(item, "ssid", (const char*)ap->ssid);
    cJSON_AddNumberToObject(item, "rssi", ap->rssi);
    cJSON_AddStringToObject(item, "signalLabel", px_signal_label_from_rssi(ap->rssi));
    cJSON_AddBoolToObject(item, "secure", ap->authmode != WIFI_AUTH_OPEN);
    cJSON_AddStringToObject(item, "authMode", px_auth_mode_str(ap->authmode));

    cJSON_AddItemToObject(root, "item", item);
    px_status_set_json(root);
}

static void px_status_publish_scan_item(int reqId, const wifi_ap_record_t* ap){
    px_status_build_scan_item(reqId, ap);
    px_status_notify_now();
}

static void px_txn_reset_idle(void) {
    px_waiting_commit = false;
    px_waiting_commit_resume_ms = 0;
    px_active_req_id = -1;
    px_active_ssid[0] = 0;
    px_active_pass[0] = 0;
    px_active_code[0] = 0;
    px_active_msg[0] = 0;
    px_txn_set_state(PX_TXN_IDLE, -1, "", "");
    px_status_build_current_txn();
}

static bool px_same_active_request(const px_prov_cmd_t* cmd){
    if (!cmd) return false;
    return (px_active_req_id == cmd->req_id &&
            strcmp(px_active_ssid, cmd->ssid) == 0 &&
            strcmp(px_active_pass, cmd->pass) == 0);
}

static bool px_scan_cache_valid_for_req(int req_id) {
    if (!px_scan_cache.valid) return false;
    if (px_scan_cache.req_id != req_id) return false;
    if ((px_nowMs() - px_scan_cache.completed_ms) > PX_SCAN_CACHE_GRACE_MS) return false;
    return true;
}

static void px_scan_cache_clear(void){
    memset(&px_scan_cache, 0, sizeof(px_scan_cache));
}

static void px_replay_scan_results_for_req(int reqId){
    if (!px_scan_cache_valid_for_req(reqId)) return;

    px_status_publish_stage(PX_TXN_SCAN_STARTED, reqId);
    vTaskDelay(pdMS_TO_TICKS(60));

    for (uint16_t i = 0; i < px_scan_cache.count; i++) {
        if (px_scan_cache.aps[i].ssid[0] == '\0') continue;
        px_status_publish_scan_item(reqId, &px_scan_cache.aps[i]);
        vTaskDelay(pdMS_TO_TICKS(PX_SCAN_RESULT_REPLAY_DELAY_MS));
    }

    px_status_publish_stage(PX_TXN_SCAN_DONE, reqId);
}

static void px_replay_status_if_needed(void){
    if (!px_is_ble_mode_flag) return;
    if (px_conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    if (!px_status_subscribed) return;

    if (px_active_stage == PX_TXN_SAVED_WAITING_COMMIT ||
        px_active_stage == PX_TXN_ERROR) {
        uint64_t now = px_nowMs();
        if ((now - px_last_status_replay_ms) >= PX_STATUS_REPLAY_PERIOD_MS) {
            px_last_status_replay_ms = now;
            px_status_publish_current();
        }
    }
}

// ---------- Wi-Fi ----------
static EventGroupHandle_t px_wifi_events(void){
    if (!px_wifi_group) px_wifi_group = xEventGroupCreate();
    return px_wifi_group;
}

static bool px_wifi_is_connected(void) {
    if (!px_wifi_group) return false;
    EventBits_t bits = xEventGroupGetBits(px_wifi_group);
    return (bits & PXWIFI_CONNECTED_BIT) != 0;
}

static bool px_wifi_reason_is_fast_connect_fail(int32_t reason)
{
    /*
     * These are connection/authentication/association failures. When one of
     * these happens during the first selected connection attempt, do not wait for
     * the full Wi-Fi timeout. Exit the wait quickly and try the safe fallback.
     * Numeric values are used intentionally to avoid SDK enum-name differences.
     * Common ESP-IDF values:
     * 202 = AUTH_FAIL
     * 203 = ASSOC_FAIL
     * 204 = HANDSHAKE_TIMEOUT
     * 205 = CONNECTION_FAIL
     * 36  = association/auth comeback related failure seen on some WPA3 APs
     */
    switch ((int)reason) {
        case 202:
        case 203:
        case 204:
        case 205:
        case 36:
            return true;
        default:
            return false;
    }
}


static void px_wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_CONNECTED) {
        /*
         * Associated is not equal to internet-ready. Wait for IP_EVENT_STA_GOT_IP
         * before starting MQTT/DNS/OTA. This prevents getaddrinfo() failures when
         * MQTT starts during DHCP.
         */
        xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT);
        px_wifi_associated_ms = px_nowMs();
        px_wifi_last_wait_ip_log_ms = px_wifi_associated_ms;
        ESP_LOGI(PX_TAG, "Wi-Fi associated; waiting for GOT_IP before MQTT/OTA");

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT);
        px_wifi_associated_ms = 0;
        px_wifi_last_wait_ip_log_ms = 0;
        px_ota_online_stable_since_ms = 0;

        uint64_t now = px_nowMs();
        px_last_wifi_disconnect_ms = now;
        if (px_wifi_ip_lost_since_ms == 0) {
            px_wifi_ip_lost_since_ms = now;
        }
        px_mqtt_wifi_fault_suppress_until_ms = now + PX_MQTT_WIFI_FAULT_SUPPRESS_MS;

        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)data;
        px_last_wifi_disconnect_reason = disc ? disc->reason : -1;

        if (px_wifi_reason_is_fast_connect_fail(px_last_wifi_disconnect_reason)) {
            xEventGroupSetBits(px_wifi_events(), PXWIFI_FAIL_BIT);
            ESP_LOGW(PX_TAG,
                     "Wi-Fi fast-fail reason=%ld -> releasing startup wait for fallback retry",
                     (long)px_last_wifi_disconnect_reason);
        }

        if (px_last_wifi_disconnect_reason == WIFI_REASON_BEACON_TIMEOUT) {
            px_last_net_fault = PX_NET_FAULT_WIFI_BEACON_TIMEOUT;
            ESP_LOGW(PX_TAG,
                     "Wi-Fi root fault: beacon timeout / AP lost, reason=%ld heap=%lu",
                     (long)px_last_wifi_disconnect_reason,
                     (unsigned long)esp_get_free_heap_size());
        } else {
            px_last_net_fault = PX_NET_FAULT_WIFI_DISCONNECT;
            ESP_LOGW(PX_TAG,
                     "Wi-Fi root fault: disconnected, reason=%ld heap=%lu",
                     (long)px_last_wifi_disconnect_reason,
                     (unsigned long)esp_get_free_heap_size());
        }

        /*
         * Do not stop/destroy esp-mqtt from the Wi-Fi event callback.
         * Keep the client alive and let esp-mqtt auto-reconnect after Wi-Fi returns.
         */
        px_mqtt_suspended_for_wifi = true;
        px_mqtt_fault_pending_until_ms = 0;
        px_mqtt_fault_pending_started_ms = 0;
        px_mqtt_transport_fault_count = 0;

        if (px_mqtt) {
            if (px_current_transport == PX_TRANS_WSS) px_ws_state = PX_ST_CONNECTING;
            else px_tls_state = PX_ST_CONNECTING;
            px_attempt_start_us = px_nowUs();
        } else {
            px_ws_state = PX_ST_DISCONNECTED;
            px_tls_state = PX_ST_DISCONNECTED;
        }

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(px_wifi_events(), PXWIFI_CONNECTED_BIT);
        px_wifi_ip_lost_since_ms = 0;
        px_mqtt_suspended_for_wifi = false;
        px_mqtt_wifi_fault_suppress_until_ms = px_nowMs() + 2000;
        px_wifi_associated_ms = 0;
        px_wifi_last_wait_ip_log_ms = 0;
        px_mqtt_fault_pending_until_ms = 0;
        px_mqtt_fault_pending_started_ms = 0;
        px_mqtt_transport_fault_count = 0;
        px_last_wifi_reconnect_ms = 0;

        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            ESP_LOGI(PX_TAG,
                     "Wi-Fi got IP. Connected AP RSSI=%d CH=%d last_reason=%ld",
                     ap.rssi, ap.primary,
                     (long)px_last_wifi_disconnect_reason);
        } else {
            ESP_LOGI(PX_TAG, "Wi-Fi got IP. Network recovery complete, last_reason=%ld",
                     (long)px_last_wifi_disconnect_reason);
        }
    }
}

static void px_build_hostname_from_serial(const char* serial, char* out, size_t outlen){
    if (!out || outlen == 0) return;
    if (!serial) serial = "";
    while (*serial==' ' || *serial=='\t' || *serial=='\r' || *serial=='\n') serial++;

    const char* s = serial;
    if ((s[0]=='S' || s[0]=='s') && (s[1]=='N' || s[1]=='n') && s[2]==':') s += 3;
    while (*s==' ' || *s=='-' || *s=='_') s++;

    snprintf(out, outlen, "Pearlexa-SN-%s", s);
}

static void px_build_mqtt_client_id(void)
{
    if (!G.device_serial || !G.device_serial[0]) {
        strlcpy(px_mqtt_client_id, "Pearlexa-unknown", sizeof(px_mqtt_client_id));
        return;
    }

#if PX_MQTT_CLIENT_ID_ADD_MAC
    uint8_t mac[6] = {0};
    if (esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK) {
        esp_read_mac(mac, ESP_MAC_BASE);
    }

    /*
     * MQTT brokers allow only one active connection for a given client_id.
     * Keeping username as the serial preserves auth/ACL behavior, while adding
     * the MAC suffix to client_id prevents ESP32 and ESP32-C3 lab boards with
     * the same serial from disconnecting each other.
     */
    snprintf(px_mqtt_client_id, sizeof(px_mqtt_client_id),
             "%s-%02X%02X%02X", G.device_serial, mac[3], mac[4], mac[5]);
#else
    strlcpy(px_mqtt_client_id, G.device_serial, sizeof(px_mqtt_client_id));
#endif

    ESP_LOGI(PX_TAG, "MQTT client_id=%s username=%s", px_mqtt_client_id, G.device_serial);
}


// ---------- Factory identity setup over factory Wi-Fi + static IP ----------
static bool px_identity_load_or_bind_from_nvs(void)
{
    px_device_serial_nvs[0] = 0;
    px_auth_number_nvs[0] = 0;

    /* Production mode: NVS is the source of truth. This allows one common
     * firmware for all devices and prevents old hardcoded Part B values from
     * accidentally being used.
     */
    px_nvs_get_strz(PX_KEY_DEVICE_SERIAL, px_device_serial_nvs, sizeof(px_device_serial_nvs));
    px_nvs_get_strz(PX_KEY_AUTH_NUMBER, px_auth_number_nvs, sizeof(px_auth_number_nvs));

#if !PX_FORCE_NVS_IDENTITY
    /* Development fallback only. Leave PX_FORCE_NVS_IDENTITY=1 for production. */
    if (px_device_serial_nvs[0] == '\0' && G.device_serial && G.device_serial[0]) {
        strlcpy(px_device_serial_nvs, G.device_serial, sizeof(px_device_serial_nvs));
    }
    if (px_auth_number_nvs[0] == '\0' && G.auth_number && G.auth_number[0]) {
        strlcpy(px_auth_number_nvs, G.auth_number, sizeof(px_auth_number_nvs));
    }
#endif

    px_identity_ready = (px_device_serial_nvs[0] != '\0' && px_auth_number_nvs[0] != '\0');

    if (px_identity_ready) {
        G.device_serial = px_device_serial_nvs;
        G.auth_number = px_auth_number_nvs;
        ESP_LOGI(PX_TAG, "Device identity ready: serial=%s", G.device_serial);
        return true;
    }

    ESP_LOGW(PX_TAG,
             "Device identity missing. Factory setup will start at %s after factory Wi-Fi connection.",
             PX_FACTORY_SETUP_URL);
    return false;
}

static char px_hex_to_char(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    return 0;
}

static void px_url_decode_inplace(char *s)
{
    if (!s) return;
    char *r = s;
    char *w = s;

    while (*r) {
        if (*r == '+') {
            *w++ = ' ';
            r++;
        } else if (*r == '%' && r[1] && r[2]) {
            *w++ = (char)((px_hex_to_char(r[1]) << 4) | px_hex_to_char(r[2]));
            r += 3;
        } else {
            *w++ = *r++;
        }
    }
    *w = 0;
}

static bool px_form_get_value(const char *body, const char *key, char *out, size_t outlen)
{
    if (!body || !key || !out || outlen == 0) return false;
    out[0] = 0;

    size_t key_len = strlen(key);
    const char *p = body;

    while (p && *p) {
        if (strncmp(p, key, key_len) == 0 && p[key_len] == '=') {
            p += key_len + 1;
            size_t n = 0;
            while (*p && *p != '&' && n < outlen - 1) {
                out[n++] = *p++;
            }
            out[n] = 0;
            px_url_decode_inplace(out);
            return out[0] != 0;
        }

        p = strchr(p, '&');
        if (p) p++;
    }

    return false;
}

static void px_factory_delayed_restart_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(900));
    esp_restart();
}

static esp_err_t px_factory_http_root_get(httpd_req_t *req)
{
    char page[1600];
    snprintf(page, sizeof(page),
             "<!doctype html><html><head>"
             "<meta name='viewport' content='width=device-width,initial-scale=1'>"
             "<title>Pearlexa Factory Setup</title>"
             "<style>body{font-family:Arial,sans-serif;background:#f7f7f7;margin:0;padding:24px;}"
             ".card{max-width:520px;margin:auto;background:white;padding:24px;border-radius:18px;box-shadow:0 8px 28px #0001;}"
             "h2{margin-top:0;}label{display:block;margin-top:14px;font-weight:600;}"
             "input{width:100%%;box-sizing:border-box;padding:12px;margin-top:6px;border:1px solid #ccc;border-radius:10px;font-size:16px;}"
             "button{width:100%%;margin-top:20px;padding:13px;border:0;border-radius:10px;background:#111;color:white;font-size:16px;}"
             ".hint{font-size:13px;color:#666;line-height:1.45;}code{background:#eee;padding:2px 5px;border-radius:4px;}"
             "</style></head><body><div class='card'>"
             "<h2>Pearlexa Factory Identity Setup</h2>"
             "<p class='hint'>Enter the per-device serial number and auth token. "
             "After saving, the device will restart and this page will be disabled.</p>"
             "<form method='POST' action='/save'>"
             "<label>Device Serial</label>"
             "<input name='deviceSerial' placeholder='SN:G1005005' required maxlength='63'>"
             "<label>Auth Token</label>"
             "<input name='authNumber' placeholder='Device-specific token' required maxlength='127'>"
             "<button type='submit'>Save Identity</button>"
             "</form>"
             "<p class='hint'>URL: <code>%s</code></p>"
             "</div></body></html>", PX_FACTORY_SETUP_URL);

    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, page, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t px_factory_http_save_post(httpd_req_t *req)
{
    char body[PX_FACTORY_IDENTITY_POST_MAX] = {0};
    int remaining = req->content_len;
    int received_total = 0;

    while (remaining > 0 && received_total < (int)sizeof(body) - 1) {
        int chunk = remaining;
        int room = (int)sizeof(body) - 1 - received_total;
        if (chunk > room) chunk = room;

        int r = httpd_req_recv(req, body + received_total, chunk);
        if (r <= 0) {
            if (r == HTTPD_SOCK_ERR_TIMEOUT) continue;
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Receive failed");
            return ESP_FAIL;
        }

        received_total += r;
        remaining -= r;
    }
    body[received_total] = 0;

    char serial[64] = {0};
    char token[128] = {0};

    bool ok_serial = px_form_get_value(body, "deviceSerial", serial, sizeof(serial));
    bool ok_token  = px_form_get_value(body, "authNumber", token, sizeof(token));

    if (!ok_serial || !ok_token || strlen(serial) < 3 || strlen(token) < 8) {
        httpd_resp_set_type(req, "text/html");
        return httpd_resp_sendstr(req,
            "<html><body><h3>Invalid serial or token</h3>"
            "<p>Please go back and check the values.</p></body></html>");
    }

    px_nvs_set_str_safe(PX_KEY_DEVICE_SERIAL, serial);
    px_nvs_set_str_safe(PX_KEY_AUTH_NUMBER, token);

    /* Keep customer Wi-Fi provisioning behavior available after identity setup. */
    px_nvs_set_bool(PX_KEY_CFG_MODE, false);
    px_nvs_set_bool(PX_KEY_CFG_LOCK, false);

    ESP_LOGI(PX_TAG, "Factory identity saved: serial=%s. Restarting.", serial);

    httpd_resp_set_type(req, "text/html");
    httpd_resp_sendstr(req,
        "<html><head><meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<title>Saved</title></head><body style='font-family:Arial;padding:24px;'>"
        "<h2>Identity Saved</h2><p>The device is restarting now.</p></body></html>");

    xTaskCreate(px_factory_delayed_restart_task,
                "px_id_restart",
                2048,
                NULL,
                5,
                NULL);
    return ESP_OK;
}

static esp_err_t px_factory_http_not_found(httpd_req_t *req, httpd_err_code_t err)
{
    (void)err;
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "/");
    return httpd_resp_send(req, NULL, 0);
}

static void px_factory_web_start(void)
{
    if (px_factory_httpd) return;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = PX_FACTORY_HTTP_PORT;
    config.lru_purge_enable = true;
    config.stack_size = 6144;

    esp_err_t err = httpd_start(&px_factory_httpd, &config);
    if (err != ESP_OK) {
        ESP_LOGE(PX_TAG, "Factory HTTP server start failed: %s", esp_err_to_name(err));
        px_factory_httpd = NULL;
        return;
    }

    httpd_uri_t root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = px_factory_http_root_get,
        .user_ctx = NULL
    };

    httpd_uri_t save = {
        .uri = "/save",
        .method = HTTP_POST,
        .handler = px_factory_http_save_post,
        .user_ctx = NULL
    };

    httpd_register_uri_handler(px_factory_httpd, &root);
    httpd_register_uri_handler(px_factory_httpd, &save);
    httpd_register_err_handler(px_factory_httpd, HTTPD_404_NOT_FOUND, px_factory_http_not_found);

    ESP_LOGW(PX_TAG,
             "Factory identity web setup ready. Open: %s",
             PX_FACTORY_SETUP_URL);
}

static void px_factory_identity_start(void)
{
    ESP_LOGW(PX_TAG,
             "Factory identity mode: connecting to SSID='%s' with static IP %s",
             PX_FACTORY_WIFI_SSID,
             PX_FACTORY_SETUP_URL);

    px_wifi_stack_ensure_ready();

    if (px_netif) {
        /* Factory setup uses a fixed IP so the operator can always open
         * http://192.168.1.10 without checking the router DHCP list.
         */
        (void)esp_netif_dhcpc_stop(px_netif);

        esp_netif_ip_info_t ip_info = {0};
        ip_info.ip.addr = ESP_IP4TOADDR(PX_FACTORY_STATIC_IP_A, PX_FACTORY_STATIC_IP_B, PX_FACTORY_STATIC_IP_C, PX_FACTORY_STATIC_IP_D);
        ip_info.gw.addr = ESP_IP4TOADDR(PX_FACTORY_GATEWAY_A, PX_FACTORY_GATEWAY_B, PX_FACTORY_GATEWAY_C, PX_FACTORY_GATEWAY_D);
        ip_info.netmask.addr = ESP_IP4TOADDR(PX_FACTORY_NETMASK_A, PX_FACTORY_NETMASK_B, PX_FACTORY_NETMASK_C, PX_FACTORY_NETMASK_D);

        esp_err_t ip_err = esp_netif_set_ip_info(px_netif, &ip_info);
        if (ip_err != ESP_OK) {
            ESP_LOGW(PX_TAG, "Factory static IP set failed: %s", esp_err_to_name(ip_err));
        }

        esp_netif_dns_info_t dns = {0};
        dns.ip.type = ESP_IPADDR_TYPE_V4;
        dns.ip.u_addr.ip4.addr = ESP_IP4TOADDR(PX_FACTORY_DNS_A, PX_FACTORY_DNS_B, PX_FACTORY_DNS_C, PX_FACTORY_DNS_D);
        (void)esp_netif_set_dns_info(px_netif, ESP_NETIF_DNS_MAIN, &dns);
    }

    wifi_config_t w = {0};
    strlcpy((char *)w.sta.ssid, PX_FACTORY_WIFI_SSID, sizeof(w.sta.ssid));
    strlcpy((char *)w.sta.password, PX_FACTORY_WIFI_PASSWORD, sizeof(w.sta.password));
    w.sta.pmf_cfg.capable = true;
    w.sta.pmf_cfg.required = false;
    w.sta.threshold.authmode = (PX_FACTORY_WIFI_PASSWORD[0] == '\0') ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT | PXWIFI_FAIL_BIT);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &w));

    esp_err_t c = esp_wifi_connect();
    if (!(c == ESP_OK || c == ESP_ERR_WIFI_CONN)) {
        ESP_LOGW(PX_TAG, "Factory Wi-Fi connect start failed: %s", esp_err_to_name(c));
    }

    EventBits_t bits = xEventGroupWaitBits(
        px_wifi_events(),
        PXWIFI_CONNECTED_BIT | PXWIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(20000)
    );

    if (bits & PXWIFI_CONNECTED_BIT) {
        ESP_LOGI(PX_TAG, "Factory Wi-Fi connected. Static setup URL: %s", PX_FACTORY_SETUP_URL);
        px_factory_web_start();
    } else {
        ESP_LOGW(PX_TAG, "Factory Wi-Fi not connected yet. Will keep retrying.");
    }
}

static void px_factory_identity_reconnect_if_needed(void)
{
    if (!px_factory_identity_mode) return;

    if (px_wifi_is_connected()) {
        if (!px_factory_httpd) px_factory_web_start();
        return;
    }

    uint64_t now = px_nowMs();
    if (px_factory_last_reconnect_ms != 0 &&
        (now - px_factory_last_reconnect_ms) < PX_FACTORY_RECONNECT_PERIOD_MS) {
        return;
    }
    px_factory_last_reconnect_ms = now;

    ESP_LOGW(PX_TAG, "Factory Wi-Fi reconnecting...");
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_wifi_connect();
}

static void px_wifi_stack_ensure_ready(void){
    if (!px_wifi_stack_ready) {
        ESP_ERROR_CHECK(esp_netif_init());
        esp_err_t el = esp_event_loop_create_default();
        if (!(el == ESP_OK || el == ESP_ERR_INVALID_STATE)) {
            ESP_ERROR_CHECK(el);
        }

        px_netif = esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));

        if (px_netif && px_hostname[0] != '\0') {
            esp_netif_set_hostname(px_netif, px_hostname);
        }

        esp_wifi_set_ps(WIFI_PS_NONE);
        esp_wifi_set_max_tx_power(78);

        px_wifi_stack_ready = true;
    }

    if (!px_wifi_handlers_registered) {
        ESP_ERROR_CHECK(esp_event_handler_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID, &px_wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP, &px_wifi_event_handler, NULL));
        px_wifi_handlers_registered = true;
    }

    if (!px_wifi_started) {
      ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
      ESP_ERROR_CHECK(esp_wifi_start());
      px_wifi_started = true;
    }
}

static void px_wifi_disconnect_now(void){
    if (!px_wifi_started) return;
    xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT | PXWIFI_FAIL_BIT);
    esp_wifi_disconnect();
}

static void px_wifi_reconnect_if_needed(void)
{
    if (px_is_ble_mode_flag) return;
    if (!px_wifi_started) return;

    if (px_wifi_is_connected()) return;

    uint64_t now = px_nowMs();

    /*
     * Associated but not yet IP-ready: wait for GOT_IP. Do NOT set the
     * PXWIFI_CONNECTED_BIT from esp_wifi_sta_get_ap_info(). The log showed MQTT
     * DNS starting before GOT_IP, causing getaddrinfo() failures.
     */
    wifi_ap_record_t ap_check;
    if (esp_wifi_sta_get_ap_info(&ap_check) == ESP_OK) {
        if (px_wifi_associated_ms == 0) px_wifi_associated_ms = now;

        if ((now - px_wifi_associated_ms) < PX_WIFI_WAIT_IP_AFTER_ASSOC_MS) {
            if (px_wifi_last_wait_ip_log_ms == 0 ||
                (now - px_wifi_last_wait_ip_log_ms) >= PX_WIFI_IP_WAIT_LOG_PERIOD_MS) {
                px_wifi_last_wait_ip_log_ms = now;
                ESP_LOGI(PX_TAG, "Wi-Fi associated but IP not ready yet; waiting for GOT_IP");
            }
            return;
        }

        ESP_LOGW(PX_TAG, "Wi-Fi associated but no IP for too long -> reconnecting");
    }

    if (px_last_wifi_reconnect_ms != 0 &&
        (now - px_last_wifi_reconnect_ms) < PX_WIFI_RECONNECT_PERIOD_MS) {
        return;
    }

    px_last_wifi_reconnect_ms = now;

    ESP_LOGW(PX_TAG, "Wi-Fi not connected -> reconnect. last_reason=%ld net_fault=%d",
             (long)px_last_wifi_disconnect_reason, (int)px_last_net_fault);

    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_err_t err = esp_wifi_connect();

    if (err == ESP_OK) {
        ESP_LOGI(PX_TAG, "esp_wifi_connect started");
    } else if (err == ESP_ERR_WIFI_CONN) {
        ESP_LOGW(PX_TAG, "Wi-Fi connection already in progress");
    } else if (err == ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(PX_TAG, "Wi-Fi not started, starting Wi-Fi again");

        esp_err_t s = esp_wifi_start();
        if (s == ESP_OK || s == ESP_ERR_WIFI_CONN) {
            ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_wifi_connect();
        }
    } else {
        ESP_LOGW(PX_TAG, "esp_wifi_connect failed: %s", esp_err_to_name(err));
    }
}

static void px_wifi_init_and_connect(void){
    px_wifi_stack_ensure_ready();

    char ssid[64] = {0};
    char pass[64] = {0};
    bool cfgLock = px_nvs_get_bool(PX_KEY_CFG_LOCK, false);

    px_nvs_get_strz(PX_KEY_SSID, ssid, sizeof(ssid));
    px_nvs_get_strz(PX_KEY_PASS, pass, sizeof(pass));

    if (ssid[0] == '\0') {
        if (cfgLock) {
            ESP_LOGW(PX_TAG, "No SSID in NVS; config locked -> staying offline (no BLE).");
            px_is_ble_mode_flag = false;
            return;
        } else {
            ESP_LOGW(PX_TAG, "No SSID in NVS; using BLE provisioning.");
            px_is_ble_mode_flag = true;
            return;
        }
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_config_t w = {0};
    px_wifi_build_config_from_ap(&w, ssid, pass, NULL);
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &w));

    xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT | PXWIFI_FAIL_BIT);
    ESP_ERROR_CHECK(esp_wifi_connect());

    EventBits_t bits = xEventGroupWaitBits(
        px_wifi_events(),
        PXWIFI_CONNECTED_BIT | PXWIFI_FAIL_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(PX_WIFI_CONNECT_TIMEOUT_MS)
    );

    if (!(bits & PXWIFI_CONNECTED_BIT)) {
        ESP_LOGW(PX_TAG, "Wi-Fi connect timeout or fast-fail");
    } else {
        ESP_LOGI(PX_TAG, "Wi-Fi connected");
    }
}

static int px_wifi_rssi_raw_now(void)
{
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        return ap.rssi;
    }
    return -127;
}

static int px_wifi_rssi_avg_now(void)
{
    int total = 0;
    int count = 0;

    for (int i = 0; i < PX_WIFI_RSSI_SAMPLE_COUNT; i++) {
        int rssi = px_wifi_rssi_raw_now();

        if (rssi > -120 && rssi < 0) {
            total += rssi;
            count++;
        }

        if (i < PX_WIFI_RSSI_SAMPLE_COUNT - 1) {
            vTaskDelay(pdMS_TO_TICKS(PX_WIFI_RSSI_SAMPLE_DELAY_MS));
        }
    }

    if (count == 0) {
        return -127;
    }

    return total / count;
}

static int px_wifi_rssi_now(void)
{
    return px_wifi_rssi_avg_now();
}

int px_wifi_rssi(void)
{
    return px_wifi_rssi_avg_now();
}

// ---------- MQTT ----------
static void px_mqtt_stop_if_any(void){
    if (px_mqtt){
        esp_mqtt_client_stop(px_mqtt);
        esp_mqtt_client_destroy(px_mqtt);
        px_mqtt = NULL;
    }
}

static void px_mqtt_suspend_until_wifi_ready(void)
{
    uint64_t now = px_nowMs();

    if (px_wifi_ip_lost_since_ms == 0) {
        px_wifi_ip_lost_since_ms = now;
    }

    px_mqtt_suspended_for_wifi = true;
    px_mqtt_fault_pending_until_ms = 0;
    px_mqtt_fault_pending_started_ms = 0;
    /*
     * Do not destroy MQTT immediately for a short Wi-Fi/IP blip.
     * This prevents a one-second beacon/DHCP interruption from forcing a full
     * WSS/TLS client destroy + recreate cycle. esp-mqtt can survive short
     * network interruptions and reconnect after GOT_IP returns.
     */
    if ((now - px_wifi_ip_lost_since_ms) < PX_MQTT_WIFI_SUSPEND_GRACE_MS) {
        if (px_mqtt) {
            if (px_current_transport == PX_TRANS_WSS) {
                px_ws_state = PX_ST_CONNECTING;
            } else {
                px_tls_state = PX_ST_CONNECTING;
            }
            px_attempt_start_us = px_nowUs();
        }
        return;
    }

    /*
     * If Wi-Fi/IP is absent for longer than the grace period, stop the MQTT
     * client cleanly from the service task. After IP_EVENT_STA_GOT_IP, the
     * service task will create a new WSS client.
     */
    if (px_mqtt) {
        ESP_LOGW(PX_TAG, "Wi-Fi/IP not ready for too long -> suspending MQTT client until GOT_IP");
        px_mqtt_stop_if_any();
    }

    px_current_transport = PX_TRANS_WSS;
    px_ws_state = PX_ST_DISCONNECTED;
    px_tls_state = PX_ST_DISCONNECTED;
    px_attempt_start_us = 0;
}

bool px_mqtt_is_connected(void)
{
    if (!px_mqtt) {
        return false;
    }

    if (px_current_transport == PX_TRANS_WSS) {
        return px_ws_state == PX_ST_CONNECTED;
    } else {
        return px_tls_state == PX_ST_CONNECTED;
    }
}

bool px_wait_mqtt_connected(int timeout_ms)
{
    uint64_t start = px_nowMs();

    while ((px_nowMs() - start) < (uint64_t)timeout_ms) {
        if (px_mqtt_is_connected()) {
            return true;
        }

        if (px_is_ble_mode_flag) {
            return false;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return px_mqtt_is_connected();
}

static void px_publish_status(const char* s){
    if (!px_mqtt_is_connected()) return;
    esp_mqtt_client_publish(px_mqtt, px_topic_status, s, 0, 1, true);
}

static void px_publish_signal_strength_now(void){
    if (!px_mqtt_is_connected()) return;
    if (px_ota_update_in_progress_flag) return;
    int r = px_wifi_rssi_now();

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "Signal_Strength", px_signal_label_from_rssi(r));
    char* out = cJSON_PrintUnformatted(root);
    if (out) {
        esp_mqtt_client_publish(px_mqtt, px_topic_tx, out, 0, 1, true);
        cJSON_free(out);
    }
    cJSON_Delete(root);
}

static void px_publish_wifi_strength_rssi_now(void){
    if (!px_mqtt_is_connected()) return;
    if (px_ota_update_in_progress_flag) return;
    int r = px_wifi_rssi_now();

    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "WiFi_Strength_RSSI", r);
    char* out = cJSON_PrintUnformatted(root);
    if (out) {
        esp_mqtt_client_publish(px_mqtt, px_topic_tx, out, 0, 1, true);
        cJSON_free(out);
    }
    cJSON_Delete(root);
}

static void px_publish_json_raw(const char* json){
    if (!px_mqtt_is_connected() || !json) return;
    esp_mqtt_client_publish(px_mqtt, px_topic_tx, json, 0, 1, true);
}

static void px_publish_firmware_info_now(void)
{
    if (!px_mqtt_is_connected()) return;

    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddStringToObject(root, "Firmware_Version",
                            G.firmware_version ? G.firmware_version : "unknown");

    if (G.hardware_model && G.hardware_model[0]) {
        cJSON_AddStringToObject(root, "Hardware_Model", G.hardware_model);
    }

    if (G.device_type && G.device_type[0]) {
        cJSON_AddStringToObject(root, "Device_Type", G.device_type);
    }

    char *out = cJSON_PrintUnformatted(root);
    if (out) {
        px_publish_json_raw(out);
        cJSON_free(out);
    }

    cJSON_Delete(root);
}

static void px_publish_json_internal(const char* json){
    if (!px_mqtt) return;

    /*
     * Once a real OTA update starts, block normal user/device publishing.
     * OTA status messages use px_publish_json_raw() and are still allowed
     * until MQTT is intentionally stopped for the HTTPS OTA download.
     */
    if (px_ota_update_in_progress_flag) return;

    px_publish_json_raw(json);
}

static void px_publish_ota_status(const char *state,
                                  const char *target_version,
                                  const char *message)
{
    if (!px_mqtt_is_connected()) return;

    const char *safe_state = state ? state : "unknown";
    const char *safe_target = target_version ? target_version : "";
    bool progress = px_ota_update_in_progress_flag;

    /*
     * Do not repeatedly publish the same OTA state.
     */
    if (px_last_ota_progress_valid &&
        strcmp(px_last_ota_state_sent, safe_state) == 0 &&
        strcmp(px_last_ota_target_sent, safe_target) == 0 &&
        px_last_ota_progress_sent == progress) {
        return;
    }

    strlcpy(px_last_ota_state_sent, safe_state, sizeof(px_last_ota_state_sent));
    strlcpy(px_last_ota_target_sent, safe_target, sizeof(px_last_ota_target_sent));
    px_last_ota_progress_sent = progress;
    px_last_ota_progress_valid = true;

    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddStringToObject(root, "OTA_Status", safe_state);
    cJSON_AddBoolToObject(root, "OTA_Update_In_Progress", progress);

    if (safe_target[0]) {
        cJSON_AddStringToObject(root, "OTA_Target_Version", safe_target);
    }

    /*
     * Only send error message when OTA failed.
     * Do not keep sending general OTA_Message.
     */
    if (strcmp(safe_state, "failed") == 0 && message && message[0]) {
        cJSON_AddStringToObject(root, "OTA_Last_Error", message);
    }

    char *out = cJSON_PrintUnformatted(root);
    if (out) {
        px_publish_json_raw(out);
        cJSON_free(out);
    }

    cJSON_Delete(root);
}

// ---------- Small JSON key reader for MQTT control commands ----------
// ESP32-C3 safety: avoid cJSON_Parse() on RX command payloads because number parsing
// can enter strtod on RISC-V and crash on some builds. This only checks simple
// top-level control flags such as {"Device_Reboot":1} or {"OTA_Check":true}.
static const char *px_json_find_value_ptr(const char *json, const char *key)
{
    if (!json || !key || !key[0]) return NULL;

    char pattern[64];
    snprintf(pattern, sizeof(pattern), "\"%s\"", key);

    const char *p = strstr(json, pattern);
    if (!p) return NULL;

    p += strlen(pattern);
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    if (*p != ':') return NULL;
    p++;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n') p++;
    return p;
}

static bool px_json_key_is_one_or_true(const char *json, const char *key)
{
    const char *p = px_json_find_value_ptr(json, key);
    if (!p) return false;

    if (*p == '1') return true;
    if (strncmp(p, "true", 4) == 0 || strncmp(p, "TRUE", 4) == 0) return true;
    if (*p == '"') {
        p++;
        if (*p == '1') return true;
        if (strncmp(p, "true", 4) == 0 || strncmp(p, "TRUE", 4) == 0) return true;
    }

    return false;
}

static bool px_json_get_string_value(const char *json, const char *key, char *out, size_t outlen)
{
    if (!out || outlen == 0) return false;
    out[0] = 0;

    const char *p = px_json_find_value_ptr(json, key);
    if (!p || *p != '"') return false;
    p++;

    size_t n = 0;
    bool esc = false;
    while (*p && n < outlen - 1) {
        char c = *p++;
        if (esc) {
            out[n++] = c;
            esc = false;
            continue;
        }
        if (c == '\\') {
            esc = true;
            continue;
        }
        if (c == '"') {
            out[n] = 0;
            return true;
        }
        out[n++] = c;
    }

    out[n] = 0;
    return false;
}

static bool px_json_get_int_value(const char *json, const char *key, int *out)
{
    if (!out) return false;
    const char *p = px_json_find_value_ptr(json, key);
    if (!p) return false;

    if (*p == '"') p++;

    int sign = 1;
    if (*p == '-') {
        sign = -1;
        p++;
    }

    if (*p < '0' || *p > '9') {
        if (strncmp(p, "true", 4) == 0 || strncmp(p, "TRUE", 4) == 0) { *out = 1; return true; }
        if (strncmp(p, "false", 5) == 0 || strncmp(p, "FALSE", 5) == 0) { *out = 0; return true; }
        return false;
    }

    int v = 0;
    while (*p >= '0' && *p <= '9') {
        if (v < 100000000) v = (v * 10) + (*p - '0');
        p++;
    }

    *out = v * sign;
    return true;
}

static void px_graceful_restart(bool enter_config_mode){
    px_publish_status("offline");
    vTaskDelay(pdMS_TO_TICKS(600));

    if (enter_config_mode){
        px_nvs_set_bool(PX_KEY_CFG_MODE, true);
        px_nvs_set_bool(PX_KEY_CFG_LOCK, false);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

// ---------- OTA helpers ----------
typedef struct {
    char *buf;
    size_t max;
    size_t len;
    bool overflow;
} px_http_buf_t;

static esp_err_t px_http_event_to_buffer(esp_http_client_event_t *evt) {
    px_http_buf_t *ctx = (px_http_buf_t *)evt->user_data;
    if (!ctx) return ESP_OK;

    if (evt->event_id == HTTP_EVENT_ON_DATA && evt->data && evt->data_len > 0) {
        if (ctx->len + evt->data_len >= ctx->max) {
            ctx->overflow = true;
            return ESP_FAIL;
        }
        memcpy(ctx->buf + ctx->len, evt->data, evt->data_len);
        ctx->len += evt->data_len;
        ctx->buf[ctx->len] = 0;
    }
    return ESP_OK;
}

static esp_err_t px_http_get_text(const char *url, char *out, size_t out_size) {
    if (!px_url_is_https(url) || !out || out_size < 16) return ESP_ERR_INVALID_ARG;

    out[0] = 0;

    px_http_buf_t ctx = {
        .buf = out,
        .max = out_size,
        .len = 0,
        .overflow = false,
    };

    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = PX_OTA_HTTP_TIMEOUT_MS,
        .event_handler = px_http_event_to_buffer,
        .user_data = &ctx,
        .buffer_size = 1024,
        .buffer_size_tx = 1024,
        .keep_alive_enable = false,
        .disable_auto_redirect = false,
        .max_redirection_count = 5,
    };

    esp_http_client_handle_t client = esp_http_client_init(&http_cfg);
    if (!client) return ESP_FAIL;

    esp_err_t err = esp_http_client_perform(client);
    int status = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client);

    if (err != ESP_OK) return err;
    if (ctx.overflow) return ESP_ERR_NO_MEM;
    if (status < 200 || status >= 300) return ESP_FAIL;

    out[out_size - 1] = 0;
    return ESP_OK;
}

static bool px_ota_parse_version_json(const char *json,
                                      char *version_out,
                                      size_t version_out_len,
                                      char *bin_url_out,
                                      size_t bin_url_out_len,
                                      bool *force_out) {
    if (!json || !version_out || !bin_url_out || !force_out) return false;

    version_out[0] = 0;
    bin_url_out[0] = 0;
    *force_out = false;

    if (!px_json_get_string_value(json, "version", version_out, version_out_len)) {
        return false;
    }

    if (!px_json_get_string_value(json, "bin_url", bin_url_out, bin_url_out_len)) {
        if (!px_json_get_string_value(json, "firmware_url", bin_url_out, bin_url_out_len)) {
            return false;
        }
    }

    *force_out = px_json_key_is_one_or_true(json, "force");

    return version_out[0] != 0 && px_url_is_https(bin_url_out);
}

static bool px_ota_metadata_allowed_json(const char *json, const char *server_version)
{
    (void)server_version;
    if (!json) return true;

    char tmp[96] = {0};

    if (G.hardware_model && G.hardware_model[0] &&
        px_json_get_string_value(json, "hardware", tmp, sizeof(tmp))) {
        if (strcmp(tmp, G.hardware_model) != 0) {
            ESP_LOGW(PX_TAG, "OTA rejected: hardware mismatch server=%s device=%s", tmp, G.hardware_model);
            return false;
        }
    }

    tmp[0] = 0;
    if (G.device_type && G.device_type[0] &&
        px_json_get_string_value(json, "model", tmp, sizeof(tmp))) {
        if (strcmp(tmp, G.device_type) != 0) {
            ESP_LOGW(PX_TAG, "OTA rejected: model mismatch server=%s device=%s", tmp, G.device_type);
            return false;
        }
    }

    tmp[0] = 0;
    if (G.firmware_version &&
        px_json_get_string_value(json, "min_current_version", tmp, sizeof(tmp))) {
        if (px_version_compare(G.firmware_version, tmp) < 0) {
            ESP_LOGW(PX_TAG, "OTA rejected: current version %s is lower than minimum %s", G.firmware_version, tmp);
            return false;
        }
    }

    int percent = 100;
    if (px_json_get_int_value(json, "rollout", &percent)) {
        if (percent < 0) percent = 0;
        if (percent > 100) percent = 100;

        if (percent < 100 && G.device_serial) {
            uint32_t h = 2166136261u;
            for (const char *p = G.device_serial; *p; ++p) {
                h ^= (uint8_t)(*p);
                h *= 16777619u;
            }
            int bucket = (int)(h % 100u);
            if (bucket >= percent) {
                ESP_LOGI(PX_TAG, "OTA rollout skip: bucket=%d rollout=%d", bucket, percent);
                return false;
            }
        }
    }

    return true;
}

static esp_err_t px_ota_install_from_url(const char *bin_url) {
    if (!px_url_is_https(bin_url)) return ESP_ERR_INVALID_ARG;

    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);

    if (running) {
        ESP_LOGI(PX_TAG, "OTA running partition: %s offset=0x%08" PRIx32,
                 running->label, running->address);
    }
    if (next) {
        ESP_LOGI(PX_TAG, "OTA next partition: %s offset=0x%08" PRIx32,
                 next->label, next->address);
    }

    esp_http_client_config_t http_cfg = {
        .url = bin_url,
        .crt_bundle_attach = esp_crt_bundle_attach,
        .timeout_ms = PX_OTA_BIN_TIMEOUT_MS,
        .buffer_size = 4096,
        .buffer_size_tx = 1024,
        .keep_alive_enable = true,
        .disable_auto_redirect = false,
        .max_redirection_count = 5,
    };

    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
        .bulk_flash_erase = false,
    };

    return esp_https_ota(&ota_cfg);
}

static void px_ota_task(void *arg) {
    (void)arg;

    ESP_LOGI(PX_TAG, "OTA task started");

    if (!G.firmware_version || !G.ota_version_url || !px_url_is_https(G.ota_version_url)) {
        ESP_LOGW(PX_TAG, "OTA not configured. firmware_version or ota_version_url missing/invalid.");
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    if (!px_ota_system_online_stable_now()) {
        ESP_LOGW(PX_TAG, "OTA skipped: Wi-Fi + MQTT are not stable yet.");
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    px_ota_set_phase(PX_OTA_PHASE_CHECKING_VERSION);
    px_publish_ota_status("checking", NULL, "Checking firmware version");
    vTaskDelay(pdMS_TO_TICKS(350));

    char *json = (char *)calloc(1, PX_OTA_VERSION_JSON_MAX);
    if (!json) {
        ESP_LOGE(PX_TAG, "OTA version JSON buffer allocation failed");
        px_ota_set_phase(PX_OTA_PHASE_FAILED);
        px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "JSON_ALLOC_FAILED");
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    esp_err_t err = px_http_get_text(G.ota_version_url, json, PX_OTA_VERSION_JSON_MAX);
    if (err != ESP_OK) {
        ESP_LOGE(PX_TAG, "OTA version check failed: %s", esp_err_to_name(err));
        px_ota_set_phase(PX_OTA_PHASE_FAILED);
        px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "VERSION_CHECK_FAILED");
        free(json);
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    char server_version[40];
    char bin_url[PX_OTA_URL_MAX];
    bool force_update = false;
    if (!px_ota_parse_version_json(json,
                                   server_version, sizeof(server_version),
                                   bin_url, sizeof(bin_url),
                                   &force_update)) {
        ESP_LOGE(PX_TAG, "OTA version JSON missing version/bin_url: %s", json);
        free(json);
        px_ota_set_phase(PX_OTA_PHASE_FAILED);
        px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "VERSION_JSON_INVALID");
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    int cmp = px_version_compare(server_version, G.firmware_version);

    ESP_LOGI(PX_TAG, "OTA current=%s server=%s force=%d",
             G.firmware_version, server_version, (int)force_update);

    if (!force_update && cmp <= 0) {
        ESP_LOGI(PX_TAG, "OTA: firmware is already up to date");
        px_publish_ota_status("up_to_date", server_version, "Firmware is already up to date");
        free(json);
        px_ota_set_phase(PX_OTA_PHASE_IDLE);
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    if (!px_ota_metadata_allowed_json(json, server_version)) {
        px_publish_ota_status("skipped", server_version, "Firmware metadata did not match this device or rollout rule");
        free(json);
        px_ota_set_phase(PX_OTA_PHASE_IDLE);
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }
    free(json);

    if (px_ota_target_attempt_blocked(server_version, force_update)) {
        px_publish_ota_status("blocked", server_version, "This target version failed too many times and is blocked");
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(PX_TAG, "OTA update available. Preparing safe stop before download: %s", bin_url);

    px_ota_set_phase(PX_OTA_PHASE_UPDATE_AVAILABLE);
    px_publish_ota_status("update_available", server_version, "New firmware available. Preparing safe stop");
    vTaskDelay(pdMS_TO_TICKS(300));

    if (!px_app_prepare_safe_stop_for_ota()) {
        ESP_LOGE(PX_TAG, "OTA aborted: product application safe-stop failed");
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    /* Real OTA begins only after safe-stop is confirmed. */
    px_ota_update_in_progress_flag = true;
    px_ota_record_attempt(server_version);
    px_ota_set_phase(PX_OTA_PHASE_DOWNLOADING);
    px_nvs_set_str_safe(PX_KEY_OTA_TARGET, server_version);
    px_publish_ota_status("downloading", server_version, "Safe stop confirmed. Starting OTA download");
    vTaskDelay(pdMS_TO_TICKS(PX_OTA_STATUS_SETTLE_MS));

    /* Free MQTT/TLS heap only after reporting safe-stop and before HTTPS OTA binary download. */
    px_mqtt_stop_if_any();
    px_ws_state = PX_ST_DISCONNECTED;
    px_tls_state = PX_ST_DISCONNECTED;

    err = px_ota_install_from_url(bin_url);

    if (err == ESP_OK) {
        ESP_LOGI(PX_TAG, "OTA successful. Restarting into new firmware.");
        px_ota_set_phase(PX_OTA_PHASE_REBOOTING_TO_NEW_APP);
        vTaskDelay(pdMS_TO_TICKS(300));
        esp_restart();
    }

    ESP_LOGE(PX_TAG, "OTA failed: %s. Restarting previous firmware cleanly.", esp_err_to_name(err));
    px_ota_set_phase(PX_OTA_PHASE_FAILED);
    px_nvs_set_str_safe(PX_KEY_OTA_FAIL, esp_err_to_name(err));

    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();

    vTaskDelete(NULL);
}

static bool px_ota_config_valid(void) {
    return G.firmware_version && G.firmware_version[0] &&
           G.ota_version_url && px_url_is_https(G.ota_version_url);
}

static void px_ota_start_task_if_possible(bool manual) {
    if (px_ota_task_running) return;
    if (!px_ota_config_valid()) return;
    if (px_is_ble_mode_flag) return;

    /*
     * OTA check must only start after Wi-Fi + MQTT are both stable.
     */
    if (!px_ota_system_online_stable_now()) return;

    px_ota_task_running = true;
    px_ota_checked_once = true;
    px_ota_last_check_ms = px_nowMs();
    px_ota_manual_request = false;

    BaseType_t ok = xTaskCreatePinnedToCore(
        px_ota_task,
        manual ? "px_ota_manual" : "px_ota_auto",
        8192,
        NULL,
        4,
        NULL,
        tskNO_AFFINITY
    );

    if (ok != pdPASS) {
        ESP_LOGE(PX_TAG, "Failed to create OTA task");
        px_ota_task_running = false;
    }
}

static void px_ota_auto_check_if_due(void) {
    if (!G.ota_check_on_boot && G.ota_periodic_check_ms <= 0 && !px_ota_manual_request) return;
    if (!px_ota_config_valid()) return;
    if (px_is_ble_mode_flag) return;
    if (px_ota_task_running) return;
    if (px_ota_update_in_progress_flag) return;

    /*
     * Do not check versioning until Wi-Fi + MQTT are stable for 10 seconds.
     */
    if (!px_ota_system_online_stable_now()) return;

    uint64_t now = px_nowMs();

    if (px_ota_manual_request) {
        px_ota_start_task_if_possible(true);
        return;
    }

    if (G.ota_check_on_boot && !px_ota_checked_once) {
        px_ota_start_task_if_possible(false);
        return;
    }

    if (G.ota_periodic_check_ms > 0 && px_ota_checked_once) {
        int period = G.ota_periodic_check_ms;
        if (period < PX_OTA_PERIOD_MIN_MS) period = PX_OTA_PERIOD_MIN_MS;

        if ((now - px_ota_last_check_ms) >= (uint64_t)period) {
            px_ota_start_task_if_possible(false);
        }
    }
}

void px_ota_check_now(void) {
    px_ota_manual_request = true;
}


static void px_mqtt_transport_connected_note(void)
{
    px_mqtt_last_connected_ms = px_nowMs();
    px_mqtt_transport_fault_count = 0;
    px_mqtt_transport_fault_window_ms = 0;    px_mqtt_suspended_for_wifi = false;
    px_mqtt_fault_pending_until_ms = 0;
    px_mqtt_fault_pending_started_ms = 0;
}

static void px_mqtt_fault_pending_clear(void)
{
    px_mqtt_fault_pending_until_ms = 0;
    px_mqtt_fault_pending_started_ms = 0;
}

static void px_mqtt_transport_fault_note(bool wifi_root_fault)
{
    uint64_t now = px_nowMs();
    px_mqtt_last_fault_ms = now;

    if (wifi_root_fault) {
        /* Do not count MQTT errors caused by Wi-Fi loss as MQTT faults. */
        return;
    }

    if (px_mqtt_transport_fault_window_ms == 0 ||
        (now - px_mqtt_transport_fault_window_ms) > PX_MQTT_FAULT_WINDOW_MS) {
        px_mqtt_transport_fault_window_ms = now;
        px_mqtt_transport_fault_count = 1;
    } else {
        if (px_mqtt_transport_fault_count < 100) px_mqtt_transport_fault_count++;
    }
}

static void px_mqtt_fault_deferred_check(void)
{
    if (px_mqtt_fault_pending_until_ms == 0) return;

    uint64_t now = px_nowMs();

    /*
     * If Wi-Fi drops shortly after the MQTT event, the MQTT error was only a
     * symptom of beacon timeout/AP loss. Do not count it as an MQTT fault.
     */
    if (px_mqtt_suspended_for_wifi ||
        !px_wifi_is_connected() ||
        (px_last_wifi_disconnect_ms != 0 &&
         (now - px_last_wifi_disconnect_ms) < PX_WIFI_RECENT_LOSS_CLASSIFY_MS)) {
        ESP_LOGW(PX_TAG,
                 "Deferred MQTT fault resolved as Wi-Fi root fault. last_reason=%ld",
                 (long)px_last_wifi_disconnect_reason);
        px_mqtt_transport_fault_note(true);
        px_mqtt_fault_pending_clear();
        return;
    }

    if (now < px_mqtt_fault_pending_until_ms) return;

    if (px_mqtt_is_connected()) {
        ESP_LOGI(PX_TAG, "Deferred MQTT fault cleared; MQTT reconnected while Wi-Fi stayed ready");
        px_mqtt_fault_pending_clear();
        return;
    }

    /*
     * Wi-Fi remained IP-ready throughout the hold window and MQTT did not
     * recover, so only now it is a true MQTT/WSS transport fault.
     */
    px_last_net_fault = PX_NET_FAULT_MQTT_TRANSPORT;
    px_mqtt_transport_fault_note(false);
    ESP_LOGW(PX_TAG,
             "MQTT/WSS fault confirmed after Wi-Fi stayed IP-ready for %d ms",
             PX_MQTT_FAULT_CLASSIFY_DELAY_MS);
    px_mqtt_fault_pending_clear();
}

static esp_err_t px_mqtt_event_handler_cb(esp_mqtt_event_handle_t e){
    switch (e->event_id){
    case MQTT_EVENT_CONNECTED:
    if (px_current_transport == PX_TRANS_WSS) {
        px_ws_state = PX_ST_CONNECTED;
    } else {
        px_tls_state = PX_ST_CONNECTED;
    }
    px_mqtt_transport_connected_note();

    /*
     * Critical startup data.
     * Send firmware/device metadata immediately on every MQTT connect/reconnect.
     * The app/backend can use this to decide whether an update is available.
     */
    px_publish_status("online");
    px_publish_firmware_info_now();

    /*
     * Normal live telemetry after critical firmware info.
     */
    px_publish_signal_strength_now();
    px_publish_wifi_strength_rssi_now();

    px_last_wifi_ms = px_nowMs();

    esp_mqtt_client_subscribe(px_mqtt, px_topic_rx, 1);
    break;

    case MQTT_EVENT_ERROR: {
        uint64_t now = px_nowMs();
        px_ota_online_stable_since_ms = 0;

        bool recent_wifi_loss = (px_last_wifi_disconnect_ms != 0 &&
                                 (now - px_last_wifi_disconnect_ms) < PX_WIFI_RECENT_LOSS_CLASSIFY_MS);

        if (px_mqtt_suspended_for_wifi ||
            !px_wifi_is_connected() ||
            now < px_mqtt_wifi_fault_suppress_until_ms ||
            recent_wifi_loss) {
            px_mqtt_transport_fault_note(true);
            px_mqtt_fault_pending_clear();
            ESP_LOGW(PX_TAG,
                     "MQTT error during Wi-Fi recovery; treating root cause as Wi-Fi, last_reason=%ld",
                     (long)px_last_wifi_disconnect_reason);
        } else {
            /*
             * MQTT_EVENT_ERROR alone is not always a completed disconnect.
             * Keep the client under esp-mqtt reconnect control and only delay
             * classification. Refresh the pending window on every new error.
             */
            px_mqtt_fault_pending_started_ms = now;
            px_mqtt_fault_pending_until_ms = now + PX_MQTT_FAULT_CLASSIFY_DELAY_MS;
            ESP_LOGW(PX_TAG,
                     "MQTT transport error while Wi-Fi has IP; delaying classification");
        }
        break;
    }

    case MQTT_EVENT_DISCONNECTED: {
        /* esp-mqtt auto reconnect remains active. Keep state as CONNECTING. */
        if (px_current_transport == PX_TRANS_WSS) {
            px_ws_state = PX_ST_CONNECTING;
        } else {
            px_tls_state = PX_ST_CONNECTING;
        }
        px_attempt_start_us = px_nowUs();
        px_ota_online_stable_since_ms = 0;

        uint64_t now = px_nowMs();
        bool recent_wifi_loss = (px_last_wifi_disconnect_ms != 0 &&
                                 (now - px_last_wifi_disconnect_ms) < PX_WIFI_RECENT_LOSS_CLASSIFY_MS);

        if (px_mqtt_suspended_for_wifi ||
            !px_wifi_is_connected() ||
            now < px_mqtt_wifi_fault_suppress_until_ms ||
            recent_wifi_loss) {
            px_mqtt_transport_fault_note(true);
            px_mqtt_fault_pending_clear();
            ESP_LOGW(PX_TAG,
                     "MQTT disconnected because Wi-Fi is down/recovering; root fault is Wi-Fi, last_reason=%ld",
                     (long)px_last_wifi_disconnect_reason);
        } else {
            /*
             * Wi-Fi still has IP, but WSS may need more than a few seconds to
             * reconnect. Refresh the hold window instead of counting this as an
             * immediate MQTT fault.
             */
            px_mqtt_fault_pending_started_ms = now;
            px_mqtt_fault_pending_until_ms = now + PX_MQTT_FAULT_CLASSIFY_DELAY_MS;
            ESP_LOGW(PX_TAG,
                     "MQTT disconnected while Wi-Fi still has IP; holding classification");
        }
        break;
    }

    case MQTT_EVENT_DATA: {
        int tlen = e->topic_len;
        int dlen = e->data_len;
        char* topic = NULL;
        char* payload = NULL;

        if (tlen > 0) {
            topic = (char*)malloc(tlen + 1);
            if (topic) { memcpy(topic, e->topic, tlen); topic[tlen] = 0; }
        }
        if (dlen > 0) {
            payload = (char*)malloc(dlen + 1);
            if (payload) { memcpy(payload, e->data, dlen); payload[dlen] = 0; }
        }

        if (topic && payload &&
            tlen == (int)strlen(px_topic_rx) &&
            strncmp(topic, px_topic_rx, tlen) == 0) {

            // ESP32-C3-safe MQTT control parsing: no cJSON_Parse() here.
            if (px_json_key_is_one_or_true(payload, "Device_Reboot")) {
                free(topic); free(payload);
                px_graceful_restart(false);
                return ESP_OK;
            }

            if (px_json_key_is_one_or_true(payload, "Device_Reset")) {
                free(topic); free(payload);
                px_graceful_restart(true);
                return ESP_OK;
            }

            if (px_json_key_is_one_or_true(payload, "Firmware_Info")) {
                ESP_LOGI(PX_TAG, "Firmware info requested by MQTT");
                px_publish_firmware_info_now();
                free(topic); free(payload);
                return ESP_OK;
            }

            if (px_json_key_is_one_or_true(payload, "Firmware_Update") ||
                px_json_key_is_one_or_true(payload, "OTA_Check")) {
                ESP_LOGI(PX_TAG, "Manual OTA check requested by MQTT");
                px_ota_check_now();
                free(topic); free(payload);
                return ESP_OK;
            }

            if (px_user_msg_cb) px_user_msg_cb(px_topic_rx, payload);
        }

        if (topic) free(topic);
        if (payload) free(payload);
        break;
    }

    default:
        break;
    }
    return ESP_OK;
}

static void px_mqtt_event_handler(void* a, esp_event_base_t b, int32_t id, void* data){
    (void)a; (void)b; (void)id;
    px_mqtt_event_handler_cb((esp_mqtt_event_handle_t)data);
}

static void px_mqtt_try_once_wss(void){
    if (!px_wifi_is_connected()) {
        ESP_LOGW(PX_TAG, "MQTT WSS skipped: Wi-Fi/GOT_IP not ready");
        return;
    }

    px_mqtt_suspended_for_wifi = false;

    ESP_LOGI(PX_TAG, "MQTT WSS connect start: keepalive=%ds timeout=%dms", PX_KEEPALIVE_SEC, PX_MQTT_CONNECT_TIMEOUT_MS);
    px_mqtt_stop_if_any();
    esp_mqtt_client_config_t cfg = {0};
    cfg.broker.address.uri = G.mqtt_wss_uri;
    cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.session.keepalive = PX_KEEPALIVE_SEC;
    cfg.network.timeout_ms = PX_MQTT_CONNECT_TIMEOUT_MS;
    cfg.network.reconnect_timeout_ms = PX_MQTT_RECONNECT_TIMEOUT_MS;
    cfg.network.disable_auto_reconnect = false;
    cfg.credentials.client_id = px_mqtt_client_id[0] ? px_mqtt_client_id : G.device_serial;
    cfg.credentials.username  = G.device_serial;
    cfg.credentials.authentication.password = G.auth_number;

    static const char will_payload[]="offline";
    cfg.session.last_will.topic  = px_topic_status;
    cfg.session.last_will.msg    = will_payload;
    cfg.session.last_will.qos    = 1;
    cfg.session.last_will.retain = true;

    px_mqtt = esp_mqtt_client_init(&cfg);
    if (!px_mqtt) {
        ESP_LOGE(PX_TAG, "esp_mqtt_client_init failed for WSS");
        px_ws_state = PX_ST_DISCONNECTED;
        return;
    }
    esp_mqtt_client_register_event(px_mqtt, MQTT_EVENT_ANY, px_mqtt_event_handler, NULL);
    px_ws_state = PX_ST_CONNECTING;
    px_attempt_start_us = px_nowUs();
    esp_err_t start_err = esp_mqtt_client_start(px_mqtt);
    if (start_err != ESP_OK) {
        ESP_LOGE(PX_TAG, "esp_mqtt_client_start WSS failed: %s", esp_err_to_name(start_err));
        esp_mqtt_client_destroy(px_mqtt);
        px_mqtt = NULL;
        px_ws_state = PX_ST_DISCONNECTED;
    }
}


static void px_restart_wss_transport(void){
    /*
     * WSS-only policy:
     * Do not flip to native MQTT TLS. Some customer/firewall networks only allow
     * WebSocket HTTPS paths, and the product requirement is WSS only.
     */
    px_last_flip_us = px_nowUs();
    px_mqtt_stop_if_any();
    px_current_transport = PX_TRANS_WSS;
    px_ws_state = PX_ST_DISCONNECTED;
    px_tls_state = PX_ST_DISCONNECTED;
    px_mqtt_try_once_wss();
}

// ---------- BLE UUIDs ----------
static const ble_uuid128_t PX_SERVICE_UUID =
    BLE_UUID128_INIT(0xF0,0xDE,0xBC,0x9A,0x78,0x56,0x34,0x12,0x78,0x56,0x34,0x12,0x78,0x56,0x34,0x12);

static const ble_uuid128_t PX_DEVINFO_UUID =
    BLE_UUID128_INIT(0x01,0xC0,0xAB,0xEF,0xCD,0xAB,0x78,0x56,0x34,0x12,0xEF,0xCD,0xAB,0xEF,0xCD,0xAB);

static const ble_uuid128_t PX_WIFI_UUID =
    BLE_UUID128_INIT(0x02,0xC0,0xAB,0xEF,0xCD,0xAB,0x78,0x56,0x34,0x12,0xEF,0xCD,0xAB,0xEF,0xCD,0xAB);

static const ble_uuid128_t PX_STATUS_UUID =
    BLE_UUID128_INIT(0x03,0xC0,0xAB,0xEF,0xCD,0xAB,0x78,0x56,0x34,0x12,0xEF,0xCD,0xAB,0xEF,0xCD,0xAB);

static void px_set_static_random_addr(void) {
    uint8_t rnd[6] = {0};
    esp_read_mac(rnd, ESP_MAC_BASE);
    rnd[5] |= 0xC0;
    ble_hs_id_set_rnd(rnd);
    px_own_addr_type = BLE_OWN_ADDR_RANDOM;
}


static bool px_enqueue_prov_cmd(const px_prov_cmd_t* cmd){
    if (!px_prov_queue || !cmd) return false;
    BaseType_t ok = xQueueSend(px_prov_queue, cmd, 0);
    return ok == pdTRUE;
}

// ---------- Provisioning worker ----------
static esp_err_t px_worker_scan_wifi(int reqId){
    if (px_scan_cache_valid_for_req(reqId)) {
        px_replay_scan_results_for_req(reqId);
        px_txn_set_state(PX_TXN_SCAN_DONE, reqId, "", "");
        return ESP_OK;
    }

    px_wifi_stack_ensure_ready();
    px_wifi_disconnect_now();

    px_status_publish_stage(PX_TXN_SCAN_STARTED, reqId);

    wifi_scan_config_t scan_cfg = {0};
    scan_cfg.show_hidden = false;

    esp_err_t rc = esp_wifi_scan_start(&scan_cfg, true);
    if (rc != ESP_OK) {
      px_status_publish_error("WIFI_SCAN_START_FAILED", "Could not start Wi-Fi scan", reqId);
      return rc;
    }

    uint16_t count = PX_WIFI_SCAN_MAX_RESULTS;
    wifi_ap_record_t aps[PX_WIFI_SCAN_MAX_RESULTS];
    memset(aps, 0, sizeof(aps));

    rc = esp_wifi_scan_get_ap_records(&count, aps);
    if (rc != ESP_OK) {
      px_status_publish_error("WIFI_SCAN_READ_FAILED", "Could not read Wi-Fi scan results", reqId);
      return rc;
    }

    px_scan_cache.valid = true;
    px_scan_cache.req_id = reqId;
    px_scan_cache.count = count;
    memset(px_scan_cache.aps, 0, sizeof(px_scan_cache.aps));
    memcpy(px_scan_cache.aps, aps, sizeof(wifi_ap_record_t) * count);

    for (uint16_t i = 0; i < count; i++) {
      if (aps[i].ssid[0] == '\0') continue;
      px_status_publish_scan_item(reqId, &aps[i]);
      vTaskDelay(pdMS_TO_TICKS(PX_SCAN_RESULT_REPLAY_DELAY_MS));
    }

    px_scan_cache.completed_ms = px_nowMs();
    px_status_publish_stage(PX_TXN_SCAN_DONE, reqId);
    return ESP_OK;
}

static void px_worker_process_set_wifi(const px_prov_cmd_t* cmd){
    if (!cmd) return;

    if (cmd->ssid[0] == '\0') {
      px_status_publish_error("MISSING_SSID", "SSID is missing", cmd->req_id);
      return;
    }

    if (px_same_active_request(cmd)) {
        px_status_publish_current();
        return;
    }

    if (px_active_req_id == cmd->req_id &&
        (!px_same_active_request(cmd))) {
        px_status_publish_error("REQ_ID_CONFLICT", "Same reqId used with different payload", cmd->req_id);
        return;
    }

    strlcpy(px_active_ssid, cmd->ssid, sizeof(px_active_ssid));
    strlcpy(px_active_pass, cmd->pass, sizeof(px_active_pass));
    px_waiting_commit = false;
    px_waiting_commit_resume_ms = 0;

    px_status_publish_stage(PX_TXN_WIFI_PAYLOAD_RECEIVED, cmd->req_id);
    px_status_publish_stage(PX_TXN_WIFI_JSON_VALID, cmd->req_id);
    px_status_publish_stage(PX_TXN_SAVING_WIFI, cmd->req_id);

    esp_err_t e1 = nvs_set_str(px_nvs, PX_KEY_SSID, cmd->ssid);
    esp_err_t e2 = nvs_set_str(px_nvs, PX_KEY_PASS, cmd->pass);
    esp_err_t e3 = nvs_set_u8(px_nvs, PX_KEY_CFG_MODE, 0);
    esp_err_t e4 = nvs_commit(px_nvs);

    if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK || e4 != ESP_OK) {
        px_status_publish_error("NVS_WRITE_FAILED", "Could not save Wi-Fi data", cmd->req_id);
        return;
    }

    px_waiting_commit = true;
    px_waiting_commit_resume_ms = 0;
    px_status_publish_stage(PX_TXN_SAVED_WAITING_COMMIT, cmd->req_id);
}

static void px_worker_process_commit_wifi(const px_prov_cmd_t* cmd){
    if (!cmd) return;

    if (!px_waiting_commit || px_active_stage != PX_TXN_SAVED_WAITING_COMMIT) {
        if (px_active_req_id == cmd->req_id && px_active_stage == PX_TXN_RESTARTING) {
            px_status_publish_current();
            return;
        }
        px_status_publish_error("NO_PENDING_COMMIT", "No pending Wi-Fi save awaiting commit", cmd->req_id);
        return;
    }

    if (cmd->req_id != px_active_req_id) {
        px_status_publish_error("REQ_ID_MISMATCH", "Commit reqId does not match active transaction", cmd->req_id);
        return;
    }

    px_waiting_commit = false;
    px_waiting_commit_resume_ms = 0;
    px_status_publish_stage(PX_TXN_COMMIT_RECEIVED, cmd->req_id);
    vTaskDelay(pdMS_TO_TICKS(120));
    px_status_publish_stage(PX_TXN_RESTARTING, cmd->req_id);

    vTaskDelay(pdMS_TO_TICKS(700));
    esp_restart();
}

static void px_prov_worker_task(void* arg){
    (void)arg;
    px_prov_cmd_t cmd;

    while (1) {
        if (xQueueReceive(px_prov_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            switch (cmd.type) {
                case PX_PROV_CMD_SCAN_WIFI:
                    px_worker_scan_wifi(cmd.req_id);
                    break;
                case PX_PROV_CMD_SET_WIFI:
                    px_worker_process_set_wifi(&cmd);
                    break;
                case PX_PROV_CMD_COMMIT_WIFI:
                    px_worker_process_commit_wifi(&cmd);
                    break;
                case PX_PROV_CMD_RESEND_STATUS:
                    if (px_scan_cache_valid_for_req(cmd.req_id)) {
                        px_replay_scan_results_for_req(cmd.req_id);
                    } else {
                        px_status_publish_current();
                    }
                    break;
                default:
                    break;
            }
        }

        px_replay_status_if_needed();

        if (px_waiting_commit && px_active_stage == PX_TXN_SAVED_WAITING_COMMIT) {
            if (!px_ble_client_connected) {
                if (px_waiting_commit_resume_ms == 0) {
                    px_waiting_commit_resume_ms = px_nowMs();
                } else if ((px_nowMs() - px_waiting_commit_resume_ms) > PX_COMMIT_TIMEOUT_MS) {
                    px_waiting_commit = false;
                    px_waiting_commit_resume_ms = 0;
                    px_status_publish_error("COMMIT_TIMEOUT", "Timed out waiting for commit from app", px_active_req_id);
                }
            } else {
                px_waiting_commit_resume_ms = 0;
            }
        }

        if (px_scan_cache.valid &&
            (px_nowMs() - px_scan_cache.completed_ms) > PX_SCAN_CACHE_GRACE_MS) {
            px_scan_cache_clear();
        }
    }
}

// ---------- BLE provisioning JSON parser ----------
/*
 * ESP32-C3 note:
 * Do not use cJSON_Parse() for the BLE provisioning command path.
 * On the affected ESP32-C3/ESP-IDF build, the tiny payload
 * {"cmd":"scan_wifi","reqId":1} crashes inside newlib strtod(), which is
 * called by cJSON when it parses numeric values.  For BLE provisioning we only need a few
 * simple fields, so we parse them with a small safe extractor instead.
 */
static const char *px_ble_json_skip_ws(const char *p)
{
    while (p && (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')) p++;
    return p;
}

static const char *px_ble_json_find_value(const char *json, const char *key)
{
    if (!json || !key || !key[0]) return NULL;

    const char *p = json;
    size_t key_len = strlen(key);

    while (*p) {
        p = strchr(p, '"');
        if (!p) return NULL;
        p++;

        const char *kstart = p;
        bool esc = false;

        while (*p) {
            if (esc) {
                esc = false;
                p++;
                continue;
            }
            if (*p == '\\') {
                esc = true;
                p++;
                continue;
            }
            if (*p == '"') break;
            p++;
        }

        if (!*p) return NULL;

        size_t got_len = (size_t)(p - kstart);
        bool key_match = (got_len == key_len && strncmp(kstart, key, key_len) == 0);

        p++; /* after closing quote */
        p = px_ble_json_skip_ws(p);

        if (key_match && *p == ':') {
            p++;
            return px_ble_json_skip_ws(p);
        }

        /* Not the requested key. Continue scanning. */
    }

    return NULL;
}

static bool px_ble_json_get_string_one(const char *json, const char *key, char *out, size_t outlen)
{
    if (!out || outlen == 0) return false;
    out[0] = '\0';

    const char *p = px_ble_json_find_value(json, key);
    if (!p || *p != '"') return false;

    p++; /* after opening quote */
    size_t n = 0;
    bool esc = false;

    while (*p) {
        char c = *p++;

        if (esc) {
            esc = false;
            switch (c) {
                case 'n': c = '\n'; break;
                case 'r': c = '\r'; break;
                case 't': c = '\t'; break;
                case '"': c = '"'; break;
                case '\\': c = '\\'; break;
                case '/': c = '/'; break;
                default: break;
            }
        } else {
            if (c == '\\') {
                esc = true;
                continue;
            }
            if (c == '"') {
                out[n] = '\0';
                return true;
            }
        }

        if (n + 1 < outlen) {
            out[n++] = c;
        }
    }

    out[n] = '\0';
    return false;
}

static bool px_ble_json_get_string_any(const char *json,
                                       const char *k1,
                                       const char *k2,
                                       char *out,
                                       size_t outlen)
{
    if (k1 && px_ble_json_get_string_one(json, k1, out, outlen)) return true;
    if (k2 && px_ble_json_get_string_one(json, k2, out, outlen)) return true;
    if (out && outlen) out[0] = '\0';
    return false;
}

static int px_ble_json_get_req_id_no_cjson(const char *json)
{
    const char *p = px_ble_json_find_value(json, "reqId");
    if (!p) return -1;

    p = px_ble_json_skip_ws(p);

    if (*p == '"') {
        p++;
    }

    bool neg = false;
    if (*p == '-') {
        neg = true;
        p++;
    }

    int value = 0;
    bool any = false;

    while (*p >= '0' && *p <= '9') {
        any = true;
        if (value < 100000000) {
            value = (value * 10) + (*p - '0');
        }
        p++;
    }

    if (!any) return -1;
    return neg ? -value : value;
}

static void px_ble_parse_and_enqueue_prov_json(const char *json)
{
    if (!json || json[0] == '\0') {
        px_status_publish_error("EMPTY_PAYLOAD", "Provisioning payload is empty", -1);
        return;
    }

    px_prov_cmd_t cmd;
    memset(&cmd, 0, sizeof(cmd));
    cmd.req_id = px_ble_json_get_req_id_no_cjson(json);

    char cmd_name[32] = {0};
    bool has_cmd = px_ble_json_get_string_one(json, "cmd", cmd_name, sizeof(cmd_name));

    if (has_cmd && cmd_name[0]) {
        if (strcmp(cmd_name, "scan_wifi") == 0) {
            cmd.type = PX_PROV_CMD_SCAN_WIFI;
        } else if (strcmp(cmd_name, "commit_wifi") == 0) {
            cmd.type = PX_PROV_CMD_COMMIT_WIFI;
        } else if (strcmp(cmd_name, "resend_status") == 0 ||
                   strcmp(cmd_name, "get_status") == 0) {
            cmd.type = PX_PROV_CMD_RESEND_STATUS;
        } else if (strcmp(cmd_name, "set_wifi") == 0) {
            cmd.type = PX_PROV_CMD_SET_WIFI;

            if (!px_ble_json_get_string_any(json, "ssid", "SSID", cmd.ssid, sizeof(cmd.ssid))) {
                px_status_publish_error("MISSING_SSID", "SSID is missing", cmd.req_id);
                return;
            }

            px_ble_json_get_string_any(json, "password", "Password", cmd.pass, sizeof(cmd.pass));
        } else {
            px_status_publish_error("UNKNOWN_CMD", "Unknown provisioning command", cmd.req_id);
            return;
        }
    } else {
        /* Backward compatibility with older phone-side payloads:
         * {"ssid":"...","password":"..."} without a cmd field.
         */
        if (!px_ble_json_get_string_any(json, "ssid", "SSID", cmd.ssid, sizeof(cmd.ssid))) {
            px_status_publish_error("MISSING_SSID", "SSID is missing", cmd.req_id);
            return;
        }

        px_ble_json_get_string_any(json, "password", "Password", cmd.pass, sizeof(cmd.pass));
        cmd.type = PX_PROV_CMD_SET_WIFI;
    }

    ESP_LOGI(PX_TAG, "BLE provision command parsed without cJSON: cmd=%d reqId=%d ssid=%s",
             (int)cmd.type, cmd.req_id, cmd.ssid[0] ? cmd.ssid : "-");

    if (!px_enqueue_prov_cmd(&cmd)) {
        px_status_publish_error("BUSY", "Provisioning worker is busy", cmd.req_id);
    }
}

static void px_ble_json_worker_task(void *arg)
{
    (void)arg;
    px_ble_json_msg_t msg;

    while (1) {
        if (xQueueReceive(px_ble_json_queue, &msg, portMAX_DELAY) == pdTRUE) {
            msg.json[sizeof(msg.json) - 1] = '\0';
            px_ble_parse_and_enqueue_prov_json(msg.json);
        }
    }
}

static void px_ble_json_worker_start_once(void)
{
    if (!px_ble_json_queue) {
        px_ble_json_queue = xQueueCreate(6, sizeof(px_ble_json_msg_t));
        if (!px_ble_json_queue) {
            ESP_LOGE(PX_TAG, "Failed to create BLE JSON queue");
            abort();
        }
    }

    if (!px_ble_json_task_handle) {
        BaseType_t ok = xTaskCreatePinnedToCore(
            px_ble_json_worker_task,
            "px_ble_json_worker",
            8192,
            NULL,
            5,
            &px_ble_json_task_handle,
            tskNO_AFFINITY
        );

        if (ok != pdPASS) {
            ESP_LOGE(PX_TAG, "Failed to create BLE JSON worker task");
            px_ble_json_task_handle = NULL;
            abort();
        }
    }
}

static bool px_ble_queue_completed_json_from_accumulator(void)
{
    if (!px_ble_json_queue) {
        px_status_publish_error("BLE_QUEUE_NOT_READY", "BLE JSON queue is not ready", -1);
        return false;
    }

    px_ble_json_msg_t msg;
    memset(&msg, 0, sizeof(msg));

    if (px_wifi_len >= sizeof(msg.json)) {
        px_status_publish_error("PAYLOAD_OVERFLOW", "Provisioning payload overflow", -1);
        return false;
    }

    msg.len = (uint16_t)px_wifi_len;
    memcpy(msg.json, px_wifi_buf, px_wifi_len);
    msg.json[px_wifi_len] = '\0';

    if (xQueueSend(px_ble_json_queue, &msg, 0) != pdTRUE) {
        px_status_publish_error("BUSY", "BLE JSON worker is busy", -1);
        return false;
    }

    return true;
}

// ---------- GATT callback ----------
static int px_gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (!ctxt || !ctxt->chr || !ctxt->chr->uuid) {
        return BLE_ATT_ERR_UNLIKELY;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &PX_DEVINFO_UUID.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            cJSON* root = cJSON_CreateObject();
            if (!root) {
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }

            cJSON_AddStringToObject(root, "deviceSerialNumber", G.device_serial ? G.device_serial : "");
            cJSON_AddStringToObject(root, "authToken", G.auth_number ? G.auth_number : "");

            if (G.firmware_version) {
                cJSON_AddStringToObject(root, "firmwareVersion", G.firmware_version);
            }

            char* full = cJSON_PrintUnformatted(root);

            if (full) {
                size_t total = strlen(full);
                size_t off = ctxt->offset;
                if (off < total) {
                    int rc = os_mbuf_append(ctxt->om, full + off, total - off);
                    cJSON_free(full);
                    cJSON_Delete(root);
                    return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
                }
                cJSON_free(full);
            }

            cJSON_Delete(root);
            return 0;
        }

        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &PX_STATUS_UUID.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            px_status_build_current_txn();
            size_t total = strlen(px_status_json);
            size_t off = ctxt->offset;
            if (off < total) {
                int rc = os_mbuf_append(ctxt->om, px_status_json + off, total - off);
                return rc == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
            }
            return 0;
        }

        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &PX_WIFI_UUID.u) == 0) {
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
            if (!ctxt->om) {
                px_status_publish_error("EMPTY_PAYLOAD", "BLE write buffer is missing", -1);
                return BLE_ATT_ERR_UNLIKELY;
            }

            uint16_t plen = OS_MBUF_PKTLEN(ctxt->om);
            if (plen == 0) {
                return 0;
            }

            /*
             * ESP32-C3 safe BLE method:
             * Keep the working-code approach: copy each BLE chunk into a small
             * local buffer, append it into px_wifi_buf, wait until the full JSON
             * object is complete, then pass it to a normal FreeRTOS task.
             * No cJSON parsing is done inside the NimBLE GATT callback.
             */
            uint8_t tmp[256];
            if (plen > sizeof(tmp)) {
                px_status_publish_error("PAYLOAD_TOO_LARGE", "BLE payload chunk too large", -1);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            int rc = os_mbuf_copydata(ctxt->om, 0, plen, tmp);
            if (rc != 0) {
                px_status_publish_error("MBUF_COPY_FAILED", "BLE payload copy failed", -1);
                return BLE_ATT_ERR_UNLIKELY;
            }

            if (!px_wifi_accum_add(tmp, plen)) {
                px_wifi_accum_reset();
                px_status_publish_error("PAYLOAD_OVERFLOW", "Provisioning payload overflow", -1);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            if (!px_json_looks_complete(px_wifi_buf, px_wifi_len)) {
                return 0;
            }

            bool queued = px_ble_queue_completed_json_from_accumulator();
            px_wifi_accum_reset();

            return queued ? 0 : BLE_ATT_ERR_UNLIKELY;
        }

        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
}

// ---------- GATT DB ----------
static const struct ble_gatt_chr_def px_gatt_chars[] = {
    {
        .uuid = &PX_DEVINFO_UUID.u,
        .access_cb = px_gatt_access_cb,
        .flags = BLE_GATT_CHR_F_READ,
    },
    {
        .uuid = &PX_WIFI_UUID.u,
        .access_cb = px_gatt_access_cb,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
    },
    {
        .uuid = &PX_STATUS_UUID.u,
        .access_cb = px_gatt_access_cb,
        .val_handle = &px_status_val_handle,
        .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
    },
    { 0 }
};

static const struct ble_gatt_svc_def px_gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &PX_SERVICE_UUID.u,
        .characteristics = px_gatt_chars
    },
    { 0 }
};

// ---------- GAP ----------
static int px_gap_event(struct ble_gap_event *event, void *arg){
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            px_conn_handle = event->connect.conn_handle;
            px_status_subscribed = false;
            px_ble_client_connected = true;
            px_waiting_commit_resume_ms = 0;

            px_txn_set_state(PX_TXN_READY, -1, "", "");
            px_status_publish_current();

            struct ble_gap_upd_params params = {
                .itvl_min = 24,
                .itvl_max = 48,
                .latency  = 0,
                .supervision_timeout = 600,
                .min_ce_len = 0,
                .max_ce_len = 0,
            };
            ble_gap_update_params(px_conn_handle, &params);
        } else {
            px_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            px_status_subscribed = false;
            px_ble_client_connected = false;

            struct ble_gap_adv_params ap = {0};
            ap.conn_mode = BLE_GAP_CONN_MODE_UND;
            ap.disc_mode = BLE_GAP_DISC_MODE_GEN;
            ap.itvl_min = 0x00A0;
            ap.itvl_max = 0x00F0;
            ble_gap_adv_start(px_own_addr_type, NULL, BLE_HS_FOREVER, &ap, px_gap_event, NULL);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        px_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        px_status_subscribed = false;
        px_ble_client_connected = false;

        if (px_waiting_commit && px_active_stage == PX_TXN_SAVED_WAITING_COMMIT) {
            px_waiting_commit_resume_ms = px_nowMs();
        }

        px_txn_reset_idle();

        {
            struct ble_gap_adv_params ap = {0};
            ap.conn_mode = BLE_GAP_CONN_MODE_UND;
            ap.disc_mode = BLE_GAP_DISC_MODE_GEN;
            ap.itvl_min = 0x00A0;
            ap.itvl_max = 0x00F0;
            ble_gap_adv_start(px_own_addr_type, NULL, BLE_HS_FOREVER, &ap, px_gap_event, NULL);
        }
        return 0;

    case BLE_GAP_EVENT_SUBSCRIBE:
        if (event->subscribe.attr_handle == px_status_val_handle) {
            px_status_subscribed = event->subscribe.cur_notify;
            if (px_status_subscribed) {
                px_status_publish_current();
            }
        }
        return 0;

    default:
        return 0;
    }
}

static int px_adv_prepare_and_start(void){
    ble_gap_adv_stop();

    struct ble_hs_adv_fields adv = {0};
    adv.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv.name = (uint8_t*)px_devname;
    adv.name_len = strlen(px_devname);
    adv.name_is_complete = 1;
    int rc = ble_gap_adv_set_fields(&adv);
    if (rc != 0) return rc;

    struct ble_hs_adv_fields scan = {0};
    static ble_uuid128_t uuid_keep;
    uuid_keep = PX_SERVICE_UUID;
    scan.uuids128 = &uuid_keep;
    scan.num_uuids128 = 1;
    scan.uuids128_is_complete = 1;
    rc = ble_gap_adv_rsp_set_fields(&scan);
    if (rc != 0) return rc;

    struct ble_gap_adv_params ap = {0};
    ap.conn_mode = BLE_GAP_CONN_MODE_UND;
    ap.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ap.itvl_min = 0x00A0;
    ap.itvl_max = 0x00F0;

    return ble_gap_adv_start(px_own_addr_type, NULL, BLE_HS_FOREVER, &ap, px_gap_event, NULL);
}

static void px_ble_on_reset(int reason){
    ESP_LOGW(PX_TAG, "NimBLE reset reason=%d", reason);
}

static void px_ble_on_sync(void){
    ble_att_set_preferred_mtu(185);
    px_set_static_random_addr();
    px_txn_reset_idle();
    px_adv_prepare_and_start();
}

static void px_nimble_host_task(void *param){
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
    vTaskDelete(NULL);
}

static void px_build_ble_name_from_serial(const char* serial, char* out, size_t outlen){
    if (!out || outlen == 0) return;
    if (!serial) serial = "";
    while (*serial==' '||*serial=='\t'||*serial=='\r'||*serial=='\n') serial++;
    snprintf(out, outlen, "Pearlexa-%s", serial);
}

static void px_ble_start_provisioning(void){
    px_ble_json_worker_start_once();

    static bool bt_mem_released = false;
    if (!bt_mem_released) {
        esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
        bt_mem_released = true;
    }

    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_init();
    ble_svc_gatt_init();

    px_build_ble_name_from_serial(G.device_serial, px_devname, sizeof(px_devname));
    ble_svc_gap_device_name_set(px_devname);

    int rc = ble_gatts_count_cfg(px_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(PX_TAG, "ble_gatts_count_cfg rc=%d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(px_gatt_svcs);
    if (rc != 0) {
        ESP_LOGE(PX_TAG, "ble_gatts_add_svcs rc=%d", rc);
        return;
    }

    ble_hs_cfg.reset_cb = px_ble_on_reset;
    ble_hs_cfg.sync_cb = px_ble_on_sync;

    nimble_port_freertos_init(px_nimble_host_task);
}

// ---------- Service loops ----------
static void px_wifi_strength_publish_if_due(void){
    if (G.wifi_strength_period_ms <= 0) return;
    if (!px_mqtt_is_connected()) return;
    if (px_ota_update_in_progress_flag) return;

    uint64_t ms = px_nowMs();
    if (px_last_wifi_ms == 0) px_last_wifi_ms = ms;
    if (ms - px_last_wifi_ms < (uint64_t)G.wifi_strength_period_ms) return;
    px_last_wifi_ms = ms;

    int r = px_wifi_rssi_now();

    cJSON* root1 = cJSON_CreateObject();
    cJSON_AddStringToObject(root1, "Signal_Strength", px_signal_label_from_rssi(r));
    char* out1 = cJSON_PrintUnformatted(root1);
    if (out1) {
        esp_mqtt_client_publish(px_mqtt, px_topic_tx, out1, 0, 1, true);
        cJSON_free(out1);
    }
    cJSON_Delete(root1);

    cJSON* root2 = cJSON_CreateObject();
    cJSON_AddNumberToObject(root2, "WiFi_Strength_RSSI", r);
    char* out2 = cJSON_PrintUnformatted(root2);
    if (out2) {
        esp_mqtt_client_publish(px_mqtt, px_topic_tx, out2, 0, 1, true);
        cJSON_free(out2);
    }
    cJSON_Delete(root2);
}

static void px_wifi_buffer_housekeeping(void){
    if (px_wifi_len == 0) return;
    uint64_t ms = px_nowMs();
    if (ms - px_wifi_last_ms > PX_WIFI_ASSEMBLY_TIMEOUT_MS) {
        px_wifi_accum_reset();
        px_status_publish_error("ASSEMBLY_TIMEOUT", "Provisioning payload timed out", -1);
    }
}

static void px_service_task(void* arg)
{
    (void)arg;

    while (1) {
        if (px_factory_identity_mode) {
            px_factory_identity_reconnect_if_needed();
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        if (!px_is_ble_mode_flag) {

            /*
             * Important:
             * Do not allow MQTT / DNS / OTA work while Wi-Fi is disconnected.
             * If Wi-Fi failed once at boot, this task will keep retrying.
             */
            if (!px_wifi_is_connected()) {
                px_wifi_reconnect_if_needed();

                /*
                 * Wi-Fi is not IP-ready. Stop MQTT reconnect attempts completely
                 * until GOT_IP. Otherwise esp-mqtt may try DNS/TLS during AP
                 * association and produce misleading MQTT errors such as
                 * "Host is unreachable" or "No PING_RESP".
                 */
                px_mqtt_suspend_until_wifi_ready();

                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            px_mqtt_suspended_for_wifi = false;
            px_mqtt_fault_deferred_check();

            if (px_ota_update_in_progress_flag) {
                /*
                 * Real OTA update has started. Keep Part A quiet until OTA restarts the device.
                 */
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }

            /*
             * Fast normal boot:
             * The gate application does not need to wait for MQTT to be online.
             * Start Part B as soon as Wi-Fi is up. MQTT commands will arrive later
             * through the already-registered callback after MQTT connects.
             * For OTA pending-verify boot, keep the old safety rule and delay app
             * start until rollback validation passes.
             */
#if PX_FAST_APP_START_AFTER_WIFI
            if (!px_ota_pending_verify && px_wifi_is_connected()) {
                px_app_start_if_allowed();
            }
#endif

            /*
             * Keep MQTT connected/reconnected.
             * OTA checking happens only after MQTT is connected and stable.
             */
            if (!px_mqtt) {
                px_current_transport = PX_TRANS_WSS;
                px_mqtt_try_once_wss();
            } else {
                bool connected = (px_ws_state == PX_ST_CONNECTED);
                bool connecting = (px_ws_state == PX_ST_CONNECTING);

                /*
                 * WSS-only reliability:
                 * esp-mqtt now owns reconnects. Do not destroy/recreate the client
                 * on every WebSocket EOF, because that causes repeated TLS/WSS
                 * handshakes and makes ESP32 look unstable. Only force-recreate if
                 * the client is stuck for a very long time.
                 */
                if (!connected && !connecting) {
                    px_ws_state = PX_ST_CONNECTING;
                    px_attempt_start_us = px_nowUs();
                }

                if (connecting &&
                    (px_nowUs() - px_attempt_start_us) > (uint64_t)PX_ATTEMPT_TIMEOUT_MS * 1000ULL &&
                    px_ok_to_flip()) {
                    ESP_LOGW(PX_TAG, "MQTT WSS appears stuck, recreating client once");
                    px_restart_wss_transport();
                }
            }

            /*
             * If this is a newly booted OTA image, confirm it only after
             * Wi-Fi + MQTT are stable for at least 10 seconds.
             */
            px_ota_confirm_running_app_if_stable();

            /*
             * In normal boot, Part A can start the product application after MQTT is connected.
             * In OTA pending-verify boot, Part A delays app start until validation passes.
             */
            if (!px_ota_pending_verify && px_mqtt_is_connected()) {
                px_app_start_if_allowed();
            }

            /*
             * Version check also happens only after Wi-Fi + MQTT are stable.
             */
            px_ota_auto_check_if_due();

            px_wifi_strength_publish_if_due();


        } else {
            px_wifi_buffer_housekeeping();
            px_replay_status_if_needed();

            if (px_scan_cache.valid &&
                (px_nowMs() - px_scan_cache.completed_ms) > PX_SCAN_CACHE_GRACE_MS) {
                px_scan_cache_clear();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ---------- Public API ----------
void px_publish_json(const char* json){
    px_publish_json_internal(json);
}

bool px_ota_update_is_in_progress(void){
    return px_ota_update_in_progress_flag;
}

bool px_ota_first_boot_validation_pending(void){
    return px_ota_pending_verify;
}

bool px_app_is_started(void){
    return px_app_started_flag;
}

void px_set_app_start_callback(px_app_void_cb_t cb){
    px_app_start_cb = cb;
}

void px_set_app_stop_callback(px_app_void_cb_t cb){
    px_app_stop_cb = cb;
}

void px_set_ota_prepare_callback(px_app_bool_cb_t cb){
    px_app_prepare_ota_cb = cb;
}

void px_set_app_self_test_callback(px_app_bool_cb_t cb){
    px_app_self_test_cb = cb;
}

void px_set_message_callback(px_msg_cb_t cb){
    px_user_msg_cb = cb;
}

bool px_is_config_mode(void){
    return px_is_ble_mode_flag || px_factory_identity_mode;
}

bool px_boot_is_config_mode(void){
    if (px_factory_identity_mode) return true;

    bool configMode = px_nvs_get_bool(PX_KEY_CFG_MODE, false);
    bool cfgLock = px_nvs_get_bool(PX_KEY_CFG_LOCK, false);

    char ssid[64] = {0};
    px_nvs_get_strz(PX_KEY_SSID, ssid, sizeof(ssid));

    if (cfgLock) return false;
    if (configMode) return true;
    if (ssid[0] == '\0') return true;
    return false;
}

void px_force_ble_mode_and_restart(void){
    px_nvs_set_bool(PX_KEY_CFG_MODE, true);
    px_nvs_set_bool(PX_KEY_CFG_LOCK, false);
    esp_restart();
}

void px_init(const px_cfg_t* cfg){
    if (!cfg) abort();

    memset(&G, 0, sizeof(G));
    G = *cfg;


    px_ota_pending_verify = px_ota_running_app_is_pending_verify();
    px_nvs_init_open();

    if (!G.mqtt_host || !G.mqtt_wss_uri) {
        ESP_LOGE(PX_TAG, "MQTT endpoint missing in px_cfg_t");
        abort();
    }

    px_identity_ready = px_identity_load_or_bind_from_nvs();
    px_factory_identity_mode = !px_identity_ready;

    if (px_factory_identity_mode) {
        /* Identity setup mode must not use MQTT topics, MQTT auth, BLE Wi-Fi provisioning, or Part B. */
        px_app_start_allowed = false;
        strlcpy(px_hostname, "Pearlexa-Factory", sizeof(px_hostname));
    }

    if (G.ota_check_on_boot || G.ota_periodic_check_ms > 0) {
        if (!G.firmware_version || !G.ota_version_url) {
            ESP_LOGW(PX_TAG, "OTA auto-check enabled but firmware_version/ota_version_url missing");
        } else {
            ESP_LOGI(PX_TAG, "OTA configured: current=%s version_url=%s",
                     G.firmware_version, G.ota_version_url);
        }
    }

    if (px_identity_ready) {
        snprintf(px_topic_tx, sizeof(px_topic_tx), "%s/TX", G.device_serial);
        snprintf(px_topic_rx, sizeof(px_topic_rx), "%s/RX", G.device_serial);
        snprintf(px_topic_status, sizeof(px_topic_status), "%s/status", G.device_serial);

        px_build_hostname_from_serial(G.device_serial, px_hostname, sizeof(px_hostname));
        px_build_mqtt_client_id();
    } else {
        px_topic_tx[0] = 0;
        px_topic_rx[0] = 0;
        px_topic_status[0] = 0;
        px_mqtt_client_id[0] = 0;
        strlcpy(px_hostname, "Pearlexa-Factory", sizeof(px_hostname));
    }

    int restartCount = px_nvs_get_int(PX_KEY_RST_COUNT, 0) + 1;
    px_nvs_set_int(PX_KEY_RST_COUNT, restartCount);

    if (!px_rst_clear_timer) {
        const esp_timer_create_args_t timer_args = {
            .callback = &px_rst_clear_timer_cb,
            .arg = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "px_rst_clear"
        };
        ESP_ERROR_CHECK(esp_timer_create(&timer_args, &px_rst_clear_timer));
    }
    esp_timer_stop(px_rst_clear_timer);
    ESP_ERROR_CHECK(esp_timer_start_once(px_rst_clear_timer, 5000000ULL));

    if (restartCount >= 10) {
        px_nvs_set_bool(PX_KEY_CFG_MODE, true);
        px_nvs_set_bool(PX_KEY_CFG_LOCK, false);
        px_nvs_set_int(PX_KEY_RST_COUNT, 0);
    }

    /* WSS-only for both ESP32 and ESP32-C3. */
    px_current_transport = PX_TRANS_WSS;
    px_ws_state = PX_ST_DISCONNECTED;
    px_tls_state = PX_ST_DISCONNECTED;
    px_attempt_start_us = 0;
    px_last_flip_us = 0;
    px_conn_handle = BLE_HS_CONN_HANDLE_NONE;
    px_status_subscribed = false;
    px_ble_client_connected = false;
    px_status_val_handle = 0;
    px_ota_task_running = false;
    px_ota_update_in_progress_flag = false;
    px_ota_checked_once = false;
    px_ota_manual_request = false;
    px_ota_last_check_ms = 0;
    px_ota_online_stable_since_ms = 0;
                    px_wifi_associated_ms = 0;
    px_wifi_last_wait_ip_log_ms = 0;
    px_mqtt_transport_fault_count = 0;
    px_mqtt_transport_fault_window_ms = 0;
    px_mqtt_last_connected_ms = 0;
    px_mqtt_last_fault_ms = 0;
        px_mqtt_suspended_for_wifi = false;
    px_mqtt_fault_pending_until_ms = 0;
    px_mqtt_fault_pending_started_ms = 0;
    px_app_started_flag = false;
    px_app_start_allowed = (!px_ota_pending_verify && !px_factory_identity_mode);

    if (!px_ota_pending_verify) {
        px_ota_phase_t old_phase = px_ota_get_phase();
        if (old_phase == PX_OTA_PHASE_REBOOTING_TO_NEW_APP ||
            old_phase == PX_OTA_PHASE_FIRST_BOOT_VALIDATION) {
            ESP_LOGW(PX_TAG, "OTA rollback/recovery detected. Previous target did not validate.");
            px_ota_set_phase(PX_OTA_PHASE_ROLLED_BACK);
            px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "ROLLED_BACK_OR_INTERRUPTED");
        } else if (old_phase == PX_OTA_PHASE_DOWNLOADING) {
            ESP_LOGW(PX_TAG, "OTA interrupted during download/flash. Continuing current valid firmware.");
            px_ota_set_phase(PX_OTA_PHASE_FAILED);
            px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "INTERRUPTED_DURING_DOWNLOAD");
        }
    } else {
        px_ota_set_phase(PX_OTA_PHASE_FIRST_BOOT_VALIDATION);
    }
    px_last_wifi_reconnect_ms = 0;
    px_wifi_accum_reset();
    px_txn_reset_idle();
    px_scan_cache_clear();

    if (!px_prov_queue) {
        px_prov_queue = xQueueCreate(8, sizeof(px_prov_cmd_t));
        if (!px_prov_queue) {
            ESP_LOGE(PX_TAG, "Failed to create provisioning queue");
            abort();
        }
    }
}

void px_start(void){
    if (px_factory_identity_mode) {
        px_is_ble_mode_flag = false;
        px_factory_identity_start();

        xTaskCreatePinnedToCore(
            px_service_task,
            "px_service_task",
            6144,
            NULL,
            5,
            NULL,
            tskNO_AFFINITY
        );
        return;
    }

    px_is_ble_mode_flag = px_boot_is_config_mode();

    xTaskCreatePinnedToCore(
        px_prov_worker_task,
        "px_prov_worker",
        8192,
        NULL,
        5,
        NULL,
        tskNO_AFFINITY
    );

    if (px_is_ble_mode_flag) {
        px_ble_start_provisioning();
    } else {
        px_wifi_stack_ensure_ready();
        px_wifi_init_and_connect();
        if (px_is_ble_mode_flag) {
            px_ble_start_provisioning();
        }
    }

    xTaskCreatePinnedToCore(
        px_service_task,
        "px_service_task",
        6144,
        NULL,
        5,
        NULL,
        tskNO_AFFINITY
    );
}
























// ---------------------------------------------------------------------------------
