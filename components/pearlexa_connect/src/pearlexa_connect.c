// =================================================================================
// Pearlexa Connect reusable ESP-IDF component.
// Universal connectivity/provisioning/MQTT/OTA layer.
// Device-specific behavior belongs in the application main.c.
// =================================================================================

#include "pearlexa_connect.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

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
#include "esp_wifi.h"

#include "mqtt_client.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
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
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

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

#define PX_KEEPALIVE_SEC               30
#define PX_WIFI_CONNECT_TIMEOUT_MS  30000
#define PX_WIFI_RECONNECT_PERIOD_MS 5000
#define PX_ATTEMPT_TIMEOUT_MS         4000
#define PX_ATTEMPT_COOLDOWN_MS         300
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

#define PX_WIFI_ROAM_SCAN_MAX_RESULTS        20
#define PX_WIFI_ROAM_SCAN_PERIOD_MS       60000
#define PX_WIFI_ROAM_WEAK_RSSI              -75
#define PX_WIFI_ROAM_MARGIN_DB               12
#define PX_WIFI_ROAM_MIN_SWITCH_GAP_MS  300000
#define PX_WIFI_RECOVERY_SCAN_GAP_MS      15000
#define PX_MQTT_WIFI_FAULT_SUPPRESS_MS     8000

/*
 * Product-grade AP scoring / bad-AP memory.
 * This prevents the ESP32 from repeatedly selecting a strong but unstable AP.
 */
#define PX_WIFI_ROAM_AP_HISTORY_MAX          8
#define PX_WIFI_ROAM_BLACKLIST_MS       300000
#define PX_WIFI_ROAM_BAD_FAIL_LIMIT           2
#define PX_WIFI_ROAM_FAIL_PENALTY            18
#define PX_WIFI_ROAM_BEACON_PENALTY          25
#define PX_WIFI_ROAM_SUCCESS_BONUS            4
#define PX_WIFI_ROAM_CURRENT_AP_BONUS         8
#define PX_WIFI_ROAM_BLACKLIST_FALLBACK_MS 30000

typedef enum { PX_TRANS_WSS=0, PX_TRANS_TLS=1 } px_transport_t;
typedef enum { PX_ST_DISCONNECTED=0, PX_ST_CONNECTING=1, PX_ST_CONNECTED=2 } px_conn_state_t;

typedef enum {
    PX_NET_FAULT_NONE = 0,
    PX_NET_FAULT_WIFI_DISCONNECT,
    PX_NET_FAULT_WIFI_BEACON_TIMEOUT,
    PX_NET_FAULT_WIFI_ROAMING,
    PX_NET_FAULT_MQTT_TRANSPORT,
} px_net_fault_t;

typedef struct {
    bool valid;
    uint8_t bssid[6];
    uint8_t channel;
    int last_rssi;
    int fail_count;
    int success_count;
    int beacon_timeout_count;
    uint64_t last_seen_ms;
    uint64_t last_connected_ms;
    uint64_t blacklist_until_ms;
} px_roam_ap_history_t;

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

static char px_topic_tx[64], px_topic_rx[64], px_topic_status[64];

static px_transport_t px_current_transport = PX_TRANS_WSS;
static px_conn_state_t px_ws_state = PX_ST_DISCONNECTED, px_tls_state = PX_ST_DISCONNECTED;
static uint64_t px_attempt_start_us = 0, px_last_flip_us = 0;

static esp_mqtt_client_handle_t px_mqtt = NULL;
static bool px_is_ble_mode_flag = false;
static px_msg_cb_t px_user_msg_cb = NULL;

static uint64_t px_last_wifi_ms = 0;
static uint64_t px_last_wifi_reconnect_ms = 0;
static uint64_t px_last_roam_scan_ms = 0;
static uint64_t px_last_roam_switch_ms = 0;
static volatile bool px_wifi_roam_in_progress = false;
static volatile bool px_wifi_recovery_in_progress = false;
static int32_t px_last_wifi_disconnect_reason = -1;
static uint64_t px_last_wifi_disconnect_ms = 0;
static uint64_t px_last_wifi_recovery_scan_ms = 0;
static uint64_t px_mqtt_wifi_fault_suppress_until_ms = 0;
static px_net_fault_t px_last_net_fault = PX_NET_FAULT_NONE;

static px_roam_ap_history_t px_roam_ap_history[PX_WIFI_ROAM_AP_HISTORY_MAX];
static uint8_t px_current_bssid[6] = {0};
static bool px_current_bssid_valid = false;

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

// ---------- OTA state ----------
static volatile bool px_ota_task_running = false;
static volatile bool px_ota_update_in_progress_flag = false;
static volatile bool px_ota_checked_once = false;
static volatile bool px_ota_manual_request = false;

static bool px_ota_pending_verify = false;

static uint64_t px_ota_last_check_ms = 0;
static uint64_t px_ota_online_stable_since_ms = 0;

// ---------- Universal app lifecycle callbacks ----------
static px_app_void_cb_t px_app_start_cb = NULL;
static px_app_void_cb_t px_app_stop_cb = NULL;
static px_app_bool_cb_t px_app_prepare_ota_cb = NULL;
static px_app_bool_cb_t px_app_self_test_cb = NULL;
static volatile bool px_app_started_flag = false;
static volatile bool px_app_start_allowed = true;

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

static void px_wifi_roaming_check_if_due(void);
static bool px_wifi_select_best_ap_config(bool force_scan);

/* Roaming/AP reliability history helpers */
static void px_roam_history_note_connected(const wifi_ap_record_t *ap);
static void px_roam_history_note_disconnect(int32_t reason);
static int  px_roam_ap_score(const wifi_ap_record_t *ap,
                            const wifi_ap_record_t *current_ap,
                            bool is_current,
                            bool *is_blacklisted_out);
static bool px_roam_choose_best_from_scan(const wifi_ap_record_t *aps,
                                          uint16_t count,
                                          const char *ssid,
                                          const wifi_ap_record_t *current_ap,
                                          bool recovery_mode,
                                          bool allow_blacklisted,
                                          wifi_ap_record_t *best_out,
                                          int *best_score_out,
                                          int *current_score_out);
static const char *px_bssid_fmt(const uint8_t bssid[6], char *out, size_t outlen);

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
static void px_app_start_if_allowed(void)
{
    if (px_is_ble_mode_flag) return;
    if (!px_app_start_allowed) return;
    if (px_ota_update_in_progress_flag) return;
    if (px_ota_pending_verify) return;       // Do not start product logic before OTA validation.
    if (px_app_started_flag) return;

    if (px_app_start_cb) {
        ESP_LOGI(PX_TAG, "Starting product application via Part B callback");
        px_app_start_cb();
        px_app_started_flag = true;
    }
}

static void px_app_stop_if_started(void)
{
    if (!px_app_started_flag) return;

    ESP_LOGI(PX_TAG, "Stopping product application via Part B callback");
    if (px_app_stop_cb) {
        px_app_stop_cb();
    }
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

static bool px_ota_metadata_allowed(cJSON *root, const char *server_version)
{
    (void)server_version;

    if (!root) return true;

    cJSON *hw = cJSON_GetObjectItem(root, "hardware");
    if (cJSON_IsString(hw) && hw->valuestring && G.hardware_model && G.hardware_model[0]) {
        if (strcmp(hw->valuestring, G.hardware_model) != 0) {
            ESP_LOGW(PX_TAG, "OTA rejected: hardware mismatch server=%s device=%s", hw->valuestring, G.hardware_model);
            return false;
        }
    }

    cJSON *model = cJSON_GetObjectItem(root, "model");
    if (cJSON_IsString(model) && model->valuestring && G.device_type && G.device_type[0]) {
        if (strcmp(model->valuestring, G.device_type) != 0) {
            ESP_LOGW(PX_TAG, "OTA rejected: model mismatch server=%s device=%s", model->valuestring, G.device_type);
            return false;
        }
    }

    cJSON *minv = cJSON_GetObjectItem(root, "min_current_version");
    if (cJSON_IsString(minv) && minv->valuestring && G.firmware_version) {
        if (px_version_compare(G.firmware_version, minv->valuestring) < 0) {
            ESP_LOGW(PX_TAG, "OTA rejected: current version %s is lower than minimum %s", G.firmware_version, minv->valuestring);
            return false;
        }
    }

    cJSON *rollout = cJSON_GetObjectItem(root, "rollout");
    if (cJSON_IsNumber(rollout)) {
        int percent = rollout->valueint;
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

    int rc = ble_gatts_notify_custom(
        px_conn_handle,
        px_status_val_handle,
        ble_hs_mbuf_from_flat(px_status_json, strlen(px_status_json))
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

static void px_wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT);
        px_ota_online_stable_since_ms = 0;

        uint64_t now = px_nowMs();
        px_last_wifi_disconnect_ms = now;
        px_mqtt_wifi_fault_suppress_until_ms = now + PX_MQTT_WIFI_FAULT_SUPPRESS_MS;

        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)data;
        px_last_wifi_disconnect_reason = disc ? disc->reason : -1;

        /*
         * Learn from AP failures. If the current BSSID caused beacon timeouts or
         * repeated disconnects, future roaming/recovery scans will penalize it.
         */
        px_roam_history_note_disconnect(px_last_wifi_disconnect_reason);

        if (px_wifi_roam_in_progress) {
            px_last_net_fault = PX_NET_FAULT_WIFI_ROAMING;
            ESP_LOGW(PX_TAG, "Wi-Fi disconnected during intentional roaming, reason=%ld", (long)px_last_wifi_disconnect_reason);
        } else if (px_last_wifi_disconnect_reason == WIFI_REASON_BEACON_TIMEOUT) {
            px_last_net_fault = PX_NET_FAULT_WIFI_BEACON_TIMEOUT;
            ESP_LOGW(PX_TAG, "Wi-Fi root fault: beacon timeout / AP lost, reason=%ld", (long)px_last_wifi_disconnect_reason);
        } else {
            px_last_net_fault = PX_NET_FAULT_WIFI_DISCONNECT;
            ESP_LOGW(PX_TAG, "Wi-Fi root fault: disconnected, reason=%ld", (long)px_last_wifi_disconnect_reason);
        }

        /*
         * MQTT read errors immediately after this are only a symptom of Wi-Fi loss.
         * Stop/destroy MQTT here, then the service task will reconnect Wi-Fi first,
         * and only after Wi-Fi is back it will rebuild MQTT cleanly.
         */
        if (px_mqtt) {
            px_mqtt_stop_if_any();
        }
        px_ws_state = PX_ST_DISCONNECTED;
        px_tls_state = PX_ST_DISCONNECTED;

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(px_wifi_events(), PXWIFI_CONNECTED_BIT);
        px_last_wifi_reconnect_ms = 0;
        px_wifi_roam_in_progress = false;
        px_wifi_recovery_in_progress = false;

        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            px_roam_history_note_connected(&ap);
            ESP_LOGI(PX_TAG,
                     "Wi-Fi got IP. Connected AP RSSI=%d CH=%d BSSID=%02x:%02x:%02x:%02x:%02x:%02x last_reason=%ld",
                     ap.rssi, ap.primary,
                     ap.bssid[0], ap.bssid[1], ap.bssid[2],
                     ap.bssid[3], ap.bssid[4], ap.bssid[5],
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
    xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT);
    esp_wifi_disconnect();
}

static void px_wifi_reconnect_if_needed(void)
{
    if (px_is_ble_mode_flag) return;
    if (!px_wifi_started) return;
    if (px_wifi_is_connected()) return;

    uint64_t now = px_nowMs();

    if (px_last_wifi_reconnect_ms != 0 &&
        (now - px_last_wifi_reconnect_ms) < PX_WIFI_RECONNECT_PERIOD_MS) {
        return;
    }

    px_last_wifi_reconnect_ms = now;

    ESP_LOGW(PX_TAG, "Wi-Fi not connected -> recovery reconnect. last_reason=%ld net_fault=%d",
             (long)px_last_wifi_disconnect_reason, (int)px_last_net_fault);

    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(100));

    /*
     * If the previous loss was beacon timeout/AP loss, do not blindly reconnect
     * to a stale BSSID. Scan the same SSID and select the strongest AP first.
     * This makes roaming/recovery intelligent for homes with multiple APs.
     */
    bool should_reselect_ap = G.wifi_roaming_enable &&
        (px_last_wifi_disconnect_reason == WIFI_REASON_BEACON_TIMEOUT ||
         px_last_wifi_disconnect_reason == WIFI_REASON_NO_AP_FOUND ||
         px_last_net_fault == PX_NET_FAULT_WIFI_BEACON_TIMEOUT ||
         px_last_net_fault == PX_NET_FAULT_WIFI_ROAMING);

    if (should_reselect_ap && !px_wifi_recovery_in_progress) {
        px_wifi_recovery_in_progress = true;
        px_wifi_select_best_ap_config(false);
    }

    esp_err_t err = esp_wifi_connect();

    if (err == ESP_OK) {
        ESP_LOGI(PX_TAG, "esp_wifi_connect started");
    } else if (err == ESP_ERR_WIFI_CONN) {
        ESP_LOGW(PX_TAG, "Wi-Fi connection already in progress");
    } else if (err == ESP_ERR_WIFI_NOT_STARTED) {
        ESP_LOGW(PX_TAG, "Wi-Fi not started, starting Wi-Fi again");

        esp_err_t s = esp_wifi_start();
        if (s == ESP_OK || s == ESP_ERR_WIFI_CONN) {
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

    wifi_config_t w = {0};
    strlcpy((char*)w.sta.ssid, ssid, sizeof(w.sta.ssid));
    strlcpy((char*)w.sta.password, pass, sizeof(w.sta.password));
    w.sta.threshold.authmode = WIFI_AUTH_OPEN;
    w.sta.pmf_cfg.capable = true;
    w.sta.pmf_cfg.required = false;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &w));

    xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT);
    ESP_ERROR_CHECK(esp_wifi_connect());

    EventBits_t bits = xEventGroupWaitBits(
        px_wifi_events(),
        PXWIFI_CONNECTED_BIT,
        pdFALSE,
        pdFALSE,
        pdMS_TO_TICKS(PX_WIFI_CONNECT_TIMEOUT_MS)
    );

    if (!(bits & PXWIFI_CONNECTED_BIT)) {
        ESP_LOGW(PX_TAG, "Wi-Fi connect timeout");
    } else {
        ESP_LOGI(PX_TAG, "Wi-Fi connected");
    }
}

static int px_wifi_rssi_now(void){
    wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) return ap.rssi;
    return -127;
}
int px_wifi_rssi(void){ return px_wifi_rssi_now(); }

// ---------- MQTT ----------
static void px_mqtt_stop_if_any(void){
    if (px_mqtt){
        esp_mqtt_client_stop(px_mqtt);
        esp_mqtt_client_destroy(px_mqtt);
        px_mqtt = NULL;
    }
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
    if (!px_mqtt) return;
    esp_mqtt_client_publish(px_mqtt, px_topic_status, s, 0, 1, true);
}

static void px_publish_signal_strength_now(void){
    if (!px_mqtt) return;
    if (px_ota_update_in_progress_flag) return;
    int r = px_wifi_rssi_now();

    cJSON* root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "Signal_Strength", px_signal_label_from_rssi(r));
    char* out = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(px_mqtt, px_topic_tx, out, 0, 1, true);
    cJSON_free(out);
    cJSON_Delete(root);
}

static void px_publish_wifi_strength_rssi_now(void){
    if (!px_mqtt) return;
    if (px_ota_update_in_progress_flag) return;
    int r = px_wifi_rssi_now();

    cJSON* root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "WiFi_Strength_RSSI", r);
    char* out = cJSON_PrintUnformatted(root);
    esp_mqtt_client_publish(px_mqtt, px_topic_tx, out, 0, 1, true);
    cJSON_free(out);
    cJSON_Delete(root);
}

static void px_publish_json_raw(const char* json){
    if (!px_mqtt || !json) return;
    esp_mqtt_client_publish(px_mqtt, px_topic_tx, json, 0, 1, true);
}

static void px_publish_firmware_info_now(void)
{
    if (!px_mqtt) return;

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

    cJSON_AddStringToObject(root, "OTA_Mode",
                            (G.ota_check_on_boot || G.ota_periodic_check_ms > 0)
                                ? "auto_or_mqtt_controlled"
                                : "mqtt_controlled");

    px_ota_phase_t phase = px_ota_get_phase();
    const char *phase_str = "idle";
    switch (phase) {
        case PX_OTA_PHASE_CHECKING_VERSION: phase_str = "checking"; break;
        case PX_OTA_PHASE_UPDATE_AVAILABLE: phase_str = "update_available"; break;
        case PX_OTA_PHASE_PREPARE_SAFE_STOP: phase_str = "preparing_safe_stop"; break;
        case PX_OTA_PHASE_SAFE_STOP_CONFIRMED: phase_str = "safe_stop_confirmed"; break;
        case PX_OTA_PHASE_DOWNLOADING: phase_str = "downloading"; break;
        case PX_OTA_PHASE_REBOOTING_TO_NEW_APP: phase_str = "rebooting_to_new_app"; break;
        case PX_OTA_PHASE_FIRST_BOOT_VALIDATION: phase_str = "first_boot_validation"; break;
        case PX_OTA_PHASE_VALIDATED: phase_str = "validated"; break;
        case PX_OTA_PHASE_FAILED: phase_str = "failed"; break;
        case PX_OTA_PHASE_ROLLED_BACK: phase_str = "rolled_back"; break;
        case PX_OTA_PHASE_IDLE:
        default: phase_str = "idle"; break;
    }

    cJSON_AddStringToObject(root, "OTA_Status", phase_str);
    cJSON_AddBoolToObject(root, "OTA_Update_In_Progress", px_ota_update_in_progress_flag);
    cJSON_AddBoolToObject(root, "OTA_First_Boot_Validation_Pending", px_ota_pending_verify);

    char target[PX_OTA_TARGET_MAX] = {0};
    px_nvs_get_strz(PX_KEY_OTA_TARGET, target, sizeof(target));
    if (target[0]) {
        cJSON_AddStringToObject(root, "OTA_Target_Version", target);
    }

    int attempt = px_nvs_get_int(PX_KEY_OTA_ATTEMPT, 0);
    cJSON_AddNumberToObject(root, "OTA_Attempt_Count", attempt);

    char fail[64] = {0};
    px_nvs_get_strz(PX_KEY_OTA_FAIL, fail, sizeof(fail));
    if (fail[0]) {
        cJSON_AddStringToObject(root, "OTA_Last_Fail_Reason", fail);
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

static void px_publish_ota_status(const char *state, const char *server_version, const char *message) {
    cJSON *root = cJSON_CreateObject();
    if (!root) return;

    cJSON_AddStringToObject(root, "OTA_Status", state ? state : "unknown");
    cJSON_AddStringToObject(root, "Current_Version", G.firmware_version ? G.firmware_version : "unknown");
    if (server_version && server_version[0]) cJSON_AddStringToObject(root, "Server_Version", server_version);
    if (message && message[0]) cJSON_AddStringToObject(root, "OTA_Message", message);

    char *out = cJSON_PrintUnformatted(root);
    if (out) {
        px_publish_json_raw(out);
        cJSON_free(out);
    }
    cJSON_Delete(root);
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

    cJSON *root = cJSON_Parse(json);
    if (!root) return false;

    cJSON *ver = cJSON_GetObjectItem(root, "version");
    cJSON *url = cJSON_GetObjectItem(root, "bin_url");
    cJSON *force = cJSON_GetObjectItem(root, "force");

    if (!cJSON_IsString(url) || !url->valuestring) {
        url = cJSON_GetObjectItem(root, "firmware_url");
    }

    if (!cJSON_IsString(ver) || !ver->valuestring ||
        !cJSON_IsString(url) || !url->valuestring) {
        cJSON_Delete(root);
        return false;
    }

    strlcpy(version_out, ver->valuestring, version_out_len);
    strlcpy(bin_url_out, url->valuestring, bin_url_out_len);

    if (cJSON_IsBool(force)) {
        *force_out = cJSON_IsTrue(force);
    } else if (cJSON_IsNumber(force)) {
        *force_out = force->valueint != 0;
    }

    cJSON_Delete(root);
    return version_out[0] != 0 && px_url_is_https(bin_url_out);
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

    cJSON *meta_root = cJSON_Parse(json);
    if (!meta_root) {
        ESP_LOGE(PX_TAG, "OTA version JSON invalid: %s", json);
        px_ota_set_phase(PX_OTA_PHASE_FAILED);
        px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "VERSION_JSON_PARSE_FAILED");
        free(json);
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    if (!px_ota_parse_version_json(json,
                                   server_version, sizeof(server_version),
                                   bin_url, sizeof(bin_url),
                                   &force_update)) {
        ESP_LOGE(PX_TAG, "OTA version JSON missing version/bin_url: %s", json);
        cJSON_Delete(meta_root);
        free(json);
        px_ota_set_phase(PX_OTA_PHASE_FAILED);
        px_nvs_set_str_safe(PX_KEY_OTA_FAIL, "VERSION_JSON_INVALID");
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }
    free(json);

    int cmp = px_version_compare(server_version, G.firmware_version);

    ESP_LOGI(PX_TAG, "OTA current=%s server=%s force=%d",
             G.firmware_version, server_version, (int)force_update);

    if (!force_update && cmp <= 0) {
        ESP_LOGI(PX_TAG, "OTA: firmware is already up to date");
        px_publish_ota_status("up_to_date", server_version, "Firmware is already up to date");
        cJSON_Delete(meta_root);
        px_ota_set_phase(PX_OTA_PHASE_IDLE);
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }

    if (!px_ota_metadata_allowed(meta_root, server_version)) {
        px_publish_ota_status("skipped", server_version, "Firmware metadata did not match this device or rollout rule");
        cJSON_Delete(meta_root);
        px_ota_set_phase(PX_OTA_PHASE_IDLE);
        px_ota_task_running = false;
        vTaskDelete(NULL);
        return;
    }
    cJSON_Delete(meta_root);

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

static esp_err_t px_mqtt_event_handler_cb(esp_mqtt_event_handle_t e){
    switch (e->event_id){
    case MQTT_EVENT_CONNECTED:
    if (px_current_transport == PX_TRANS_WSS) {
        px_ws_state = PX_ST_CONNECTED;
    } else {
        px_tls_state = PX_ST_CONNECTED;
    }

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

    case MQTT_EVENT_DISCONNECTED:
    case MQTT_EVENT_ERROR:
        if (px_current_transport==PX_TRANS_WSS) px_ws_state = PX_ST_DISCONNECTED;
        else px_tls_state = PX_ST_DISCONNECTED;
        px_ota_online_stable_since_ms = 0;

        if (!px_wifi_is_connected() || px_nowMs() < px_mqtt_wifi_fault_suppress_until_ms) {
            ESP_LOGW(PX_TAG, "MQTT transport lost because Wi-Fi is down/recovering; root fault is Wi-Fi, last_reason=%ld",
                     (long)px_last_wifi_disconnect_reason);
        } else {
            px_last_net_fault = PX_NET_FAULT_MQTT_TRANSPORT;
            ESP_LOGW(PX_TAG, "MQTT root fault: transport disconnected while Wi-Fi is still connected");
        }
        break;

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

            cJSON* root = cJSON_Parse(payload);
            if (root){
                cJSON* reboot = cJSON_GetObjectItem(root, "Device_Reboot");
                if (cJSON_IsNumber(reboot) && reboot->valueint == 1){
                    cJSON_Delete(root);
                    free(topic); free(payload);
                    px_graceful_restart(false);
                    return ESP_OK;
                }

                cJSON* reset = cJSON_GetObjectItem(root, "Device_Reset");
                if (cJSON_IsNumber(reset) && reset->valueint == 1){
                    cJSON_Delete(root);
                    free(topic); free(payload);
                    px_graceful_restart(true);
                    return ESP_OK;
                }

                cJSON* fw_info = cJSON_GetObjectItem(root, "Firmware_Info");
                if ((cJSON_IsNumber(fw_info) && fw_info->valueint == 1) ||
                    (cJSON_IsBool(fw_info) && cJSON_IsTrue(fw_info))) {
                    ESP_LOGI(PX_TAG, "Firmware info requested by MQTT");
                    px_publish_firmware_info_now();
                    cJSON_Delete(root);
                    free(topic); free(payload);
                    return ESP_OK;
                }

                cJSON* fw = cJSON_GetObjectItem(root, "Firmware_Update");
                if ((cJSON_IsNumber(fw) && fw->valueint == 1) ||
                    (cJSON_IsBool(fw) && cJSON_IsTrue(fw))) {
                    ESP_LOGI(PX_TAG, "Manual OTA check requested by MQTT");
                    px_ota_check_now();
                    cJSON_Delete(root);
                    free(topic); free(payload);
                    return ESP_OK;
                }

                cJSON* ota_check = cJSON_GetObjectItem(root, "OTA_Check");
                if ((cJSON_IsNumber(ota_check) && ota_check->valueint == 1) ||
                    (cJSON_IsBool(ota_check) && cJSON_IsTrue(ota_check))) {
                    ESP_LOGI(PX_TAG, "Manual OTA check requested by MQTT");
                    px_ota_check_now();
                    cJSON_Delete(root);
                    free(topic); free(payload);
                    return ESP_OK;
                }

                cJSON_Delete(root);
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
        ESP_LOGW(PX_TAG, "MQTT WSS skipped: Wi-Fi not connected");
        return;
    }

    px_mqtt_stop_if_any();
    esp_mqtt_client_config_t cfg = {0};
    cfg.broker.address.uri = G.mqtt_wss_uri;
    cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.session.keepalive = PX_KEEPALIVE_SEC;
    cfg.network.timeout_ms = 10000;
    cfg.network.disable_auto_reconnect = true;
    cfg.credentials.client_id = G.device_serial;
    cfg.credentials.username  = G.device_serial;
    cfg.credentials.authentication.password = G.auth_number;

    static const char will_payload[]="offline";
    cfg.session.last_will.topic  = px_topic_status;
    cfg.session.last_will.msg    = will_payload;
    cfg.session.last_will.qos    = 1;
    cfg.session.last_will.retain = true;

    px_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(px_mqtt, MQTT_EVENT_ANY, px_mqtt_event_handler, NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(px_mqtt));
    px_ws_state = PX_ST_CONNECTING;
    px_attempt_start_us = px_nowUs();
}

static void px_mqtt_try_once_tls(void){
    if (!px_wifi_is_connected()) {
        ESP_LOGW(PX_TAG, "MQTT TLS skipped: Wi-Fi not connected");
        return;
    }

    px_mqtt_stop_if_any();
    char uri[128];
    snprintf(uri, sizeof(uri), "mqtts://%s:%d", G.mqtt_host, G.mqtt_tls_port);

    esp_mqtt_client_config_t cfg = {0};
    cfg.broker.address.uri = uri;
    cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    cfg.session.keepalive = PX_KEEPALIVE_SEC;
    cfg.network.timeout_ms = 10000;
    cfg.network.disable_auto_reconnect = true;
    cfg.credentials.client_id = G.device_serial;
    cfg.credentials.username  = G.device_serial;
    cfg.credentials.authentication.password = G.auth_number;

    static const char will_payload[]="offline";
    cfg.session.last_will.topic  = px_topic_status;
    cfg.session.last_will.msg    = will_payload;
    cfg.session.last_will.qos    = 1;
    cfg.session.last_will.retain = true;

    px_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(px_mqtt, MQTT_EVENT_ANY, px_mqtt_event_handler, NULL);
    ESP_ERROR_CHECK(esp_mqtt_client_start(px_mqtt));
    px_tls_state = PX_ST_CONNECTING;
    px_attempt_start_us = px_nowUs();
}

static void px_flip_transport(void){
    px_last_flip_us = px_nowUs();
    if (px_current_transport==PX_TRANS_WSS){
        px_mqtt_stop_if_any();
        px_ws_state = PX_ST_DISCONNECTED;
        px_current_transport = PX_TRANS_TLS;
        px_tls_state = PX_ST_DISCONNECTED;
        px_mqtt_try_once_tls();
    } else {
        px_mqtt_stop_if_any();
        px_tls_state = PX_ST_DISCONNECTED;
        px_current_transport = PX_TRANS_WSS;
        px_ws_state = PX_ST_DISCONNECTED;
        px_mqtt_try_once_wss();
    }
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

// ---------- Provisioning parsing ----------
static bool px_extract_string_any(cJSON* root, const char* k1, const char* k2, char* out, size_t outlen){
    if (!root || !out || outlen == 0) return false;
    cJSON* a = k1 ? cJSON_GetObjectItem(root, k1) : NULL;
    cJSON* b = k2 ? cJSON_GetObjectItem(root, k2) : NULL;

    if (cJSON_IsString(a) && a->valuestring) {
        strlcpy(out, a->valuestring, outlen);
        return true;
    }
    if (cJSON_IsString(b) && b->valuestring) {
        strlcpy(out, b->valuestring, outlen);
        return true;
    }
    return false;
}

static int px_extract_req_id(cJSON* root){
    if (!root) return -1;
    cJSON* reqId = cJSON_GetObjectItem(root, "reqId");
    if (cJSON_IsNumber(reqId)) return reqId->valueint;
    return -1;
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

// ---------- GATT callback ----------
static int px_gatt_access_cb(uint16_t conn_handle, uint16_t attr_handle,
                             struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void)conn_handle;
    (void)attr_handle;
    (void)arg;

    if (!ctxt || !ctxt->chr || !ctxt->chr->uuid) return BLE_ATT_ERR_UNLIKELY;

    if (ble_uuid_cmp(ctxt->chr->uuid, &PX_DEVINFO_UUID.u) == 0){
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR){
            cJSON* root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "deviceSerialNumber", G.device_serial ? G.device_serial : "");
            cJSON_AddStringToObject(root, "authToken", G.auth_number ? G.auth_number : "");
            if (G.firmware_version) cJSON_AddStringToObject(root, "firmwareVersion", G.firmware_version);
            char* full = cJSON_PrintUnformatted(root);

            size_t total = strlen(full);
            size_t off = ctxt->offset;
            if (off < total) os_mbuf_append(ctxt->om, full + off, total - off);

            cJSON_free(full);
            cJSON_Delete(root);
            return 0;
        }
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &PX_STATUS_UUID.u) == 0){
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR){
            px_status_build_current_txn();
            size_t total = strlen(px_status_json);
            size_t off = ctxt->offset;
            if (off < total) os_mbuf_append(ctxt->om, px_status_json + off, total - off);
            return 0;
        }
        return BLE_ATT_ERR_REQ_NOT_SUPPORTED;
    }

    if (ble_uuid_cmp(ctxt->chr->uuid, &PX_WIFI_UUID.u) == 0){
        if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR){
            uint16_t plen = OS_MBUF_PKTLEN(ctxt->om);
            if (plen == 0) return 0;

            uint8_t tmp[256];
            if (plen > sizeof(tmp)) {
                px_status_publish_error("PAYLOAD_TOO_LARGE", "BLE payload chunk too large", -1);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            os_mbuf_copydata(ctxt->om, 0, plen, tmp);

            if (!px_wifi_accum_add(tmp, plen)) {
                px_wifi_accum_reset();
                px_status_publish_error("PAYLOAD_OVERFLOW", "Provisioning payload overflow", -1);
                return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            }

            if (!px_json_looks_complete(px_wifi_buf, px_wifi_len)) {
                return 0;
            }

            cJSON* root = cJSON_Parse(px_wifi_buf);
            if (!root){
                px_status_publish_error("INVALID_JSON", "Provisioning JSON parse failed", -1);
                px_wifi_accum_reset();
                return BLE_ATT_ERR_UNLIKELY;
            }

            px_prov_cmd_t cmd;
            memset(&cmd, 0, sizeof(cmd));
            cmd.req_id = px_extract_req_id(root);

            cJSON* jcmd = cJSON_GetObjectItem(root, "cmd");

            if (cJSON_IsString(jcmd) && jcmd->valuestring) {
                if (strcmp(jcmd->valuestring, "scan_wifi") == 0) {
                    cmd.type = PX_PROV_CMD_SCAN_WIFI;
                } else if (strcmp(jcmd->valuestring, "commit_wifi") == 0) {
                    cmd.type = PX_PROV_CMD_COMMIT_WIFI;
                } else if (strcmp(jcmd->valuestring, "resend_status") == 0 ||
                           strcmp(jcmd->valuestring, "get_status") == 0) {
                    cmd.type = PX_PROV_CMD_RESEND_STATUS;
                } else if (strcmp(jcmd->valuestring, "set_wifi") == 0) {
                    cmd.type = PX_PROV_CMD_SET_WIFI;

                    if (!px_extract_string_any(root, "ssid", "SSID", cmd.ssid, sizeof(cmd.ssid))) {
                        cJSON_Delete(root);
                        px_wifi_accum_reset();
                        px_status_publish_error("MISSING_SSID", "SSID is missing", cmd.req_id);
                        return 0;
                    }

                    px_extract_string_any(root, "password", "Password", cmd.pass, sizeof(cmd.pass));
                } else {
                    cJSON_Delete(root);
                    px_wifi_accum_reset();
                    px_status_publish_error("UNKNOWN_CMD", "Unknown provisioning command", cmd.req_id);
                    return BLE_ATT_ERR_UNLIKELY;
                }
            } else {
                cmd.type = PX_PROV_CMD_SET_WIFI;

                if (!px_extract_string_any(root, "ssid", "SSID", cmd.ssid, sizeof(cmd.ssid))) {
                    cJSON_Delete(root);
                    px_wifi_accum_reset();
                    px_status_publish_error("MISSING_SSID", "SSID is missing", cmd.req_id);
                    return 0;
                }

                px_extract_string_any(root, "password", "Password", cmd.pass, sizeof(cmd.pass));
            }

            cJSON_Delete(root);
            px_wifi_accum_reset();

            if (!px_enqueue_prov_cmd(&cmd)) {
                px_status_publish_error("BUSY", "Provisioning worker is busy", cmd.req_id);
            }

            return 0;
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

// ---------- Wi-Fi roaming ----------
static int px_roam_scan_period_ms(void)
{
    return G.wifi_roam_scan_period_ms > 0 ? G.wifi_roam_scan_period_ms : PX_WIFI_ROAM_SCAN_PERIOD_MS;
}

static int px_roam_weak_rssi(void)
{
    return G.wifi_roam_weak_rssi != 0 ? G.wifi_roam_weak_rssi : PX_WIFI_ROAM_WEAK_RSSI;
}

static int px_roam_margin_db(void)
{
    return G.wifi_roam_margin_db > 0 ? G.wifi_roam_margin_db : PX_WIFI_ROAM_MARGIN_DB;
}

static int px_roam_min_switch_gap_ms(void)
{
    return G.wifi_roam_min_switch_gap_ms > 0 ? G.wifi_roam_min_switch_gap_ms : PX_WIFI_ROAM_MIN_SWITCH_GAP_MS;
}

static bool px_bssid_equal(const uint8_t a[6], const uint8_t b[6])
{
    return memcmp(a, b, 6) == 0;
}

static const char *px_bssid_fmt(const uint8_t bssid[6], char *out, size_t out_len)
{
    if (!out || out_len < 18) return "00:00:00:00:00:00";
    snprintf(out, out_len, "%02x:%02x:%02x:%02x:%02x:%02x",
             bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
    return out;
}

static px_roam_ap_history_t *px_roam_history_find(const uint8_t bssid[6])
{
    for (int i = 0; i < PX_WIFI_ROAM_AP_HISTORY_MAX; i++) {
        if (px_roam_ap_history[i].valid &&
            px_bssid_equal(px_roam_ap_history[i].bssid, bssid)) {
            return &px_roam_ap_history[i];
        }
    }
    return NULL;
}

static px_roam_ap_history_t *px_roam_history_get_or_create(const uint8_t bssid[6])
{
    px_roam_ap_history_t *h = px_roam_history_find(bssid);
    if (h) return h;

    int slot = -1;
    uint64_t oldest = UINT64_MAX;

    for (int i = 0; i < PX_WIFI_ROAM_AP_HISTORY_MAX; i++) {
        if (!px_roam_ap_history[i].valid) {
            slot = i;
            break;
        }
        uint64_t t = px_roam_ap_history[i].last_seen_ms;
        if (t < oldest) {
            oldest = t;
            slot = i;
        }
    }

    if (slot < 0) slot = 0;

    memset(&px_roam_ap_history[slot], 0, sizeof(px_roam_ap_history[slot]));
    px_roam_ap_history[slot].valid = true;
    memcpy(px_roam_ap_history[slot].bssid, bssid, 6);
    return &px_roam_ap_history[slot];
}

static void px_roam_history_note_seen(const wifi_ap_record_t *ap)
{
    if (!ap) return;
    px_roam_ap_history_t *h = px_roam_history_get_or_create(ap->bssid);
    if (!h) return;

    h->last_rssi = ap->rssi;
    h->channel = ap->primary;
    h->last_seen_ms = px_nowMs();
}

static void px_roam_history_note_connected(const wifi_ap_record_t *ap)
{
    if (!ap) return;

    px_roam_ap_history_t *h = px_roam_history_get_or_create(ap->bssid);
    if (!h) return;

    uint64_t now = px_nowMs();

    h->last_rssi = ap->rssi;
    h->channel = ap->primary;
    h->last_seen_ms = now;
    h->last_connected_ms = now;

    if (h->success_count < 20) h->success_count++;

    /* A successful connection slowly heals old penalties. */
    if (h->fail_count > 0) h->fail_count--;
    if (h->beacon_timeout_count > 0) h->beacon_timeout_count--;

    h->blacklist_until_ms = 0;

    memcpy(px_current_bssid, ap->bssid, 6);
    px_current_bssid_valid = true;

    char b[24];
    ESP_LOGI(PX_TAG,
             "Roam history: connected BSSID=%s success=%d fail=%d beacon=%d rssi=%d",
             px_bssid_fmt(ap->bssid, b, sizeof(b)),
             h->success_count, h->fail_count, h->beacon_timeout_count, ap->rssi);
}

static void px_roam_history_note_disconnect(int32_t reason)
{
    if (!px_current_bssid_valid) return;

    px_roam_ap_history_t *h = px_roam_history_get_or_create(px_current_bssid);
    if (!h) {
        px_current_bssid_valid = false;
        return;
    }

    uint64_t now = px_nowMs();
    bool hard_failure = false;

    h->last_seen_ms = now;

    if (reason == WIFI_REASON_BEACON_TIMEOUT) {
        h->beacon_timeout_count++;
        h->fail_count++;
        hard_failure = true;
    } else if (reason == WIFI_REASON_NO_AP_FOUND) {
        h->fail_count++;
        hard_failure = true;
    } else {
        /*
         * General disconnect. Count it lightly but still learn from repeated failures.
         * We avoid depending on many reason-code macros here so this remains portable
         * across ESP-IDF versions.
         */
        h->fail_count++;
    }

    if (h->fail_count > 50) h->fail_count = 50;
    if (h->beacon_timeout_count > 50) h->beacon_timeout_count = 50;

    if (hard_failure || h->fail_count >= PX_WIFI_ROAM_BAD_FAIL_LIMIT) {
        h->blacklist_until_ms = now + PX_WIFI_ROAM_BLACKLIST_MS;
    }

    char b[24];
    ESP_LOGW(PX_TAG,
             "Roam history: AP fault BSSID=%s reason=%ld fail=%d beacon=%d blacklist_ms=%lu",
             px_bssid_fmt(px_current_bssid, b, sizeof(b)),
             (long)reason,
             h->fail_count,
             h->beacon_timeout_count,
             (unsigned long)(h->blacklist_until_ms > now ? (h->blacklist_until_ms - now) : 0));

    px_current_bssid_valid = false;
}

static int px_roam_ap_score(const wifi_ap_record_t *ap,
                            const wifi_ap_record_t *cur_ap,
                            bool allow_blacklisted,
                            bool *rejected_blacklist)
{
    if (rejected_blacklist) *rejected_blacklist = false;
    if (!ap) return -9999;

    uint64_t now = px_nowMs();
    px_roam_history_note_seen(ap);
    px_roam_ap_history_t *h = px_roam_history_find(ap->bssid);

    if (h && h->blacklist_until_ms > now && !allow_blacklisted) {
        if (rejected_blacklist) *rejected_blacklist = true;
        return -9999;
    }

    int score = ap->rssi;  /* RSSI is negative; less negative is better. */

    if (h) {
        score -= h->fail_count * PX_WIFI_ROAM_FAIL_PENALTY;
        score -= h->beacon_timeout_count * PX_WIFI_ROAM_BEACON_PENALTY;

        int bonus = h->success_count * PX_WIFI_ROAM_SUCCESS_BONUS;
        if (bonus > 20) bonus = 20;
        score += bonus;

        if (h->blacklist_until_ms > now && allow_blacklisted) {
            /* Fallback mode only: still strongly penalize a blacklisted AP. */
            score -= 60;
        }
    }

    if (cur_ap && px_bssid_equal(ap->bssid, cur_ap->bssid)) {
        score += PX_WIFI_ROAM_CURRENT_AP_BONUS;
    }

    return score;
}

static bool px_roam_choose_best_from_scan(const wifi_ap_record_t *aps,
                                          uint16_t count,
                                          const char *ssid,
                                          const wifi_ap_record_t *cur_ap,
                                          bool include_current,
                                          bool allow_blacklisted,
                                          wifi_ap_record_t *best_out,
                                          int *best_score_out,
                                          int *current_score_out)
{
    if (!aps || !ssid || !best_out) return false;

    bool found = false;
    int best_score = -9999;
    int current_score = -9999;
    uint64_t now = px_nowMs();

    for (uint16_t i = 0; i < count; i++) {
        if (aps[i].ssid[0] == '\0') continue;
        if (strcmp((const char *)aps[i].ssid, ssid) != 0) continue;

        bool is_current = cur_ap && px_bssid_equal(aps[i].bssid, cur_ap->bssid);
        if (is_current && !include_current) {
            bool rejected = false;
            current_score = px_roam_ap_score(&aps[i], cur_ap, true, &rejected);
            continue;
        }

        bool rejected_blacklist = false;
        int score = px_roam_ap_score(&aps[i], cur_ap, allow_blacklisted, &rejected_blacklist);

        char b[24];
        px_roam_ap_history_t *h = px_roam_history_find(aps[i].bssid);
        ESP_LOGI(PX_TAG,
                 "Roam scored AP: RSSI=%d SCORE=%d CH=%d BSSID=%s fail=%d beacon=%d success=%d blacklisted=%d",
                 aps[i].rssi,
                 score,
                 aps[i].primary,
                 px_bssid_fmt(aps[i].bssid, b, sizeof(b)),
                 h ? h->fail_count : 0,
                 h ? h->beacon_timeout_count : 0,
                 h ? h->success_count : 0,
                 (h && h->blacklist_until_ms > now) ? 1 : 0);

        if (rejected_blacklist) continue;

        if (!found || score > best_score) {
            *best_out = aps[i];
            best_score = score;
            found = true;
        }

        if (is_current) current_score = score;
    }

    if (best_score_out) *best_score_out = best_score;
    if (current_score_out) *current_score_out = current_score;
    return found;
}

static bool px_wifi_select_best_ap_config(bool force_scan)
{
    char ssid[64] = {0};
    char pass[64] = {0};
    px_nvs_get_strz(PX_KEY_SSID, ssid, sizeof(ssid));
    px_nvs_get_strz(PX_KEY_PASS, pass, sizeof(pass));
    if (ssid[0] == '\0') return false;

    uint64_t now = px_nowMs();
    if (!force_scan && px_last_wifi_recovery_scan_ms != 0 &&
        (now - px_last_wifi_recovery_scan_ms) < PX_WIFI_RECOVERY_SCAN_GAP_MS) {
        return false;
    }
    px_last_wifi_recovery_scan_ms = now;

    wifi_scan_config_t scan_cfg = {0};
    scan_cfg.ssid = (uint8_t *)ssid;
    scan_cfg.bssid = NULL;
    scan_cfg.channel = 0;
    scan_cfg.show_hidden = false;

    ESP_LOGI(PX_TAG, "Wi-Fi recovery scan with AP scoring: SSID=%s", ssid);

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        ESP_LOGW(PX_TAG, "Wi-Fi recovery scan failed: %s", esp_err_to_name(err));
        return false;
    }

    uint16_t count = PX_WIFI_ROAM_SCAN_MAX_RESULTS;
    wifi_ap_record_t aps[PX_WIFI_ROAM_SCAN_MAX_RESULTS];
    memset(aps, 0, sizeof(aps));

    err = esp_wifi_scan_get_ap_records(&count, aps);
    if (err != ESP_OK) {
        ESP_LOGW(PX_TAG, "Wi-Fi recovery scan read failed: %s", esp_err_to_name(err));
        return false;
    }

    wifi_ap_record_t best_ap;
    memset(&best_ap, 0, sizeof(best_ap));
    int best_score = -9999;

    bool found = px_roam_choose_best_from_scan(aps, count, ssid, NULL, true, false,
                                               &best_ap, &best_score, NULL);

    if (!found) {
        uint64_t since_disconnect = px_last_wifi_disconnect_ms ? (now - px_last_wifi_disconnect_ms) : 0;
        if (since_disconnect >= PX_WIFI_ROAM_BLACKLIST_FALLBACK_MS) {
            ESP_LOGW(PX_TAG, "Wi-Fi recovery: all APs are blacklisted or rejected; using fallback scoring");
            found = px_roam_choose_best_from_scan(aps, count, ssid, NULL, true, true,
                                                  &best_ap, &best_score, NULL);
        }
    }

    wifi_config_t w = {0};
    strlcpy((char *)w.sta.ssid, ssid, sizeof(w.sta.ssid));
    strlcpy((char *)w.sta.password, pass, sizeof(w.sta.password));
    w.sta.threshold.authmode = WIFI_AUTH_OPEN;
    w.sta.pmf_cfg.capable = true;
    w.sta.pmf_cfg.required = false;

    if (found) {
        w.sta.bssid_set = true;
        memcpy(w.sta.bssid, best_ap.bssid, 6);
        w.sta.channel = best_ap.primary;
        char b[24];
        ESP_LOGI(PX_TAG,
                 "Wi-Fi recovery selected AP by score: score=%d RSSI=%d CH=%d BSSID=%s",
                 best_score, best_ap.rssi, best_ap.primary,
                 px_bssid_fmt(best_ap.bssid, b, sizeof(b)));
    } else {
        /* No scan result found. Clear BSSID lock so the Wi-Fi driver can choose any AP. */
        w.sta.bssid_set = false;
        w.sta.channel = 0;
        ESP_LOGW(PX_TAG, "Wi-Fi recovery found no usable AP; clearing BSSID lock and using normal SSID reconnect");
    }

    err = esp_wifi_set_config(WIFI_IF_STA, &w);
    if (err != ESP_OK) {
        ESP_LOGW(PX_TAG, "Wi-Fi recovery set_config failed: %s", esp_err_to_name(err));
        return false;
    }

    return true;
}

static void px_wifi_roaming_check_if_due(void)
{
    if (!G.wifi_roaming_enable) return;
    if (px_is_ble_mode_flag) return;
    if (px_ota_pending_verify) return;
    if (px_ota_task_running || px_ota_update_in_progress_flag || px_ota_manual_request) return;
    if (px_wifi_roam_in_progress) return;
    if (!px_wifi_is_connected()) return;

    /* Keep roaming conservative. */
    if (!px_mqtt_is_connected()) return;

    uint64_t now = px_nowMs();
    int scan_period = px_roam_scan_period_ms();
    if (scan_period < 30000) scan_period = 30000;

    if (px_last_roam_scan_ms != 0 &&
        (now - px_last_roam_scan_ms) < (uint64_t)scan_period) {
        return;
    }
    px_last_roam_scan_ms = now;

    wifi_ap_record_t cur_ap;
    if (esp_wifi_sta_get_ap_info(&cur_ap) != ESP_OK) return;

    px_roam_history_note_connected(&cur_ap);

    int weak_rssi = px_roam_weak_rssi();
    int margin_db = px_roam_margin_db();
    int min_gap_ms = px_roam_min_switch_gap_ms();

    /* RSSI is negative. Example: -62 is better than -82. */
    if (cur_ap.rssi > weak_rssi) {
        return;
    }

    if (px_last_roam_switch_ms != 0 &&
        (now - px_last_roam_switch_ms) < (uint64_t)min_gap_ms) {
        return;
    }

    char ssid[64] = {0};
    char pass[64] = {0};
    px_nvs_get_strz(PX_KEY_SSID, ssid, sizeof(ssid));
    px_nvs_get_strz(PX_KEY_PASS, pass, sizeof(pass));
    if (ssid[0] == '\0') return;

    px_wifi_roam_in_progress = true;

    ESP_LOGI(PX_TAG,
             "Wi-Fi roaming score scan: current SSID=%s RSSI=%d threshold=%d margin=%d",
             ssid, cur_ap.rssi, weak_rssi, margin_db);

    wifi_scan_config_t scan_cfg = {0};
    scan_cfg.ssid = (uint8_t *)ssid;
    scan_cfg.bssid = NULL;
    scan_cfg.channel = 0;
    scan_cfg.show_hidden = false;

    esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
    if (err != ESP_OK) {
        ESP_LOGW(PX_TAG, "Wi-Fi roaming scan failed: %s", esp_err_to_name(err));
        px_wifi_roam_in_progress = false;
        return;
    }

    uint16_t count = PX_WIFI_ROAM_SCAN_MAX_RESULTS;
    wifi_ap_record_t aps[PX_WIFI_ROAM_SCAN_MAX_RESULTS];
    memset(aps, 0, sizeof(aps));

    err = esp_wifi_scan_get_ap_records(&count, aps);
    if (err != ESP_OK) {
        ESP_LOGW(PX_TAG, "Wi-Fi roaming scan read failed: %s", esp_err_to_name(err));
        px_wifi_roam_in_progress = false;
        return;
    }

    wifi_ap_record_t best_ap;
    memset(&best_ap, 0, sizeof(best_ap));
    int best_score = -9999;
    int current_score = -9999;

    bool found = px_roam_choose_best_from_scan(aps, count, ssid, &cur_ap, false, false,
                                               &best_ap, &best_score, &current_score);

    if (!found) {
        ESP_LOGI(PX_TAG, "Wi-Fi roaming: no non-blacklisted alternative AP found");
        px_wifi_roam_in_progress = false;
        return;
    }

    if (current_score <= -9000) {
        current_score = px_roam_ap_score(&cur_ap, &cur_ap, true, NULL);
    }

    int rssi_gain = best_ap.rssi - cur_ap.rssi;

/*
 * Normal score-based roaming:
 * This respects AP reliability history, blacklist, success count, and fail count.
 */
bool score_says_roam = best_score >= (current_score + margin_db);

/*
 * Strong RSSI override:
 * If another AP is much stronger, do not let the current AP success bonus
 * block roaming forever.
 *
 * Example:
 * Current AP = -76 dBm
 * Best AP    = -57 dBm
 * RSSI gain  = 19 dB
 * Result     = allow roaming
 */
bool strong_rssi_override = false;

if (best_ap.rssi > cur_ap.rssi) {
    if (rssi_gain >= 15 && best_ap.rssi >= -70) {
        strong_rssi_override = true;
    }
}

if (!score_says_roam && !strong_rssi_override) {
    char b[24];
    ESP_LOGI(PX_TAG,
             "Wi-Fi roaming: staying on current AP. current_score=%d best_score=%d margin=%d rssi_gain=%d best=%s rssi=%d",
             current_score,
             best_score,
             margin_db,
             rssi_gain,
             px_bssid_fmt(best_ap.bssid, b, sizeof(b)),
             best_ap.rssi);

    px_wifi_roam_in_progress = false;
    return;
}

    char best_b[24], cur_b[24];
    ESP_LOGW(PX_TAG,
             "Wi-Fi roaming: switching AP by reliability score. current=%s RSSI=%d score=%d -> best=%s RSSI=%d score=%d CH=%d",
             px_bssid_fmt(cur_ap.bssid, cur_b, sizeof(cur_b)), cur_ap.rssi, current_score,
             px_bssid_fmt(best_ap.bssid, best_b, sizeof(best_b)), best_ap.rssi, best_score, best_ap.primary);

    wifi_config_t w = {0};
    strlcpy((char *)w.sta.ssid, ssid, sizeof(w.sta.ssid));
    strlcpy((char *)w.sta.password, pass, sizeof(w.sta.password));
    w.sta.threshold.authmode = WIFI_AUTH_OPEN;
    w.sta.pmf_cfg.capable = true;
    w.sta.pmf_cfg.required = false;
    w.sta.bssid_set = true;
    memcpy(w.sta.bssid, best_ap.bssid, 6);
    w.sta.channel = best_ap.primary;

    /* MQTT will reconnect cleanly after Wi-Fi comes back. */
    if (px_mqtt) {
        px_mqtt_stop_if_any();
        px_ws_state = PX_ST_DISCONNECTED;
        px_tls_state = PX_ST_DISCONNECTED;
    }

    px_ota_online_stable_since_ms = 0;
    xEventGroupClearBits(px_wifi_events(), PXWIFI_CONNECTED_BIT);

    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(250));

    err = esp_wifi_set_config(WIFI_IF_STA, &w);
    if (err != ESP_OK) {
        ESP_LOGW(PX_TAG, "Wi-Fi roaming set_config failed: %s", esp_err_to_name(err));
        px_wifi_roam_in_progress = false;
        return;
    }

    err = esp_wifi_connect();
    if (err == ESP_OK || err == ESP_ERR_WIFI_CONN) {
        px_last_roam_switch_ms = px_nowMs();
        px_wifi_recovery_in_progress = true;
        ESP_LOGI(PX_TAG, "Wi-Fi roaming reconnect started");
        /* Keep px_wifi_roam_in_progress true until GOT_IP, so Wi-Fi/MQTT faults are classified correctly. */
        return;
    } else {
        ESP_LOGW(PX_TAG, "Wi-Fi roaming reconnect failed: %s", esp_err_to_name(err));
    }

    px_wifi_roam_in_progress = false;
    px_wifi_recovery_in_progress = false;
}

// ---------- Service loops ----------
static void px_wifi_strength_publish_if_due(void){
    if (G.wifi_strength_period_ms <= 0) return;
    if (!px_mqtt) return;
    if (px_ota_update_in_progress_flag) return;

    uint64_t ms = px_nowMs();
    if (px_last_wifi_ms == 0) px_last_wifi_ms = ms;
    if (ms - px_last_wifi_ms < (uint64_t)G.wifi_strength_period_ms) return;
    px_last_wifi_ms = ms;

    int r = px_wifi_rssi_now();

    cJSON* root1 = cJSON_CreateObject();
    cJSON_AddStringToObject(root1, "Signal_Strength", px_signal_label_from_rssi(r));
    char* out1 = cJSON_PrintUnformatted(root1);
    esp_mqtt_client_publish(px_mqtt, px_topic_tx, out1, 0, 1, true);
    cJSON_free(out1);
    cJSON_Delete(root1);

    cJSON* root2 = cJSON_CreateObject();
    cJSON_AddNumberToObject(root2, "WiFi_Strength_RSSI", r);
    char* out2 = cJSON_PrintUnformatted(root2);
    esp_mqtt_client_publish(px_mqtt, px_topic_tx, out2, 0, 1, true);
    cJSON_free(out2);
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
        if (!px_is_ble_mode_flag) {

            /*
             * Important:
             * Do not allow MQTT / DNS / OTA work while Wi-Fi is disconnected.
             * If Wi-Fi failed once at boot, this task will keep retrying.
             */
            if (!px_wifi_is_connected()) {
                px_wifi_reconnect_if_needed();

                if (px_mqtt) {
                    px_mqtt_stop_if_any();
                    px_ws_state = PX_ST_DISCONNECTED;
                    px_tls_state = PX_ST_DISCONNECTED;
                }

                vTaskDelay(pdMS_TO_TICKS(500));
                continue;
            }

            if (px_ota_update_in_progress_flag) {
                /*
                 * Real OTA update has started. Keep Part A quiet until OTA restarts the device.
                 */
                vTaskDelay(pdMS_TO_TICKS(250));
                continue;
            }

            /*
             * First keep MQTT connected/reconnected.
             * OTA checking happens only after MQTT is connected and stable.
             */
            if (px_mqtt) {
                bool connected =
                    (px_current_transport == PX_TRANS_WSS && px_ws_state == PX_ST_CONNECTED) ||
                    (px_current_transport == PX_TRANS_TLS && px_tls_state == PX_ST_CONNECTED);

                bool connecting =
                    (px_current_transport == PX_TRANS_WSS && px_ws_state == PX_ST_CONNECTING) ||
                    (px_current_transport == PX_TRANS_TLS && px_tls_state == PX_ST_CONNECTING);

                if (connecting) {
                    if ((px_nowUs() - px_attempt_start_us) > (uint64_t)PX_ATTEMPT_TIMEOUT_MS * 1000ULL) {
                        if (px_ok_to_flip()) {
                            px_flip_transport();
                        }
                    }
                } else if (!connected) {
                    if (px_ok_to_flip()) {
                        if (px_current_transport == PX_TRANS_WSS) {
                            px_mqtt_try_once_wss();
                        } else {
                            px_mqtt_try_once_tls();
                        }
                    }
                }
            } else {
                if (px_current_transport == PX_TRANS_WSS) {
                    px_mqtt_try_once_wss();
                } else {
                    px_mqtt_try_once_tls();
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
            px_wifi_roaming_check_if_due();

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
    return px_is_ble_mode_flag;
}

bool px_boot_is_config_mode(void){
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

    if (!G.device_serial || !G.auth_number || !G.mqtt_host || !G.mqtt_wss_uri) {
        abort();
    }

    if (G.ota_check_on_boot || G.ota_periodic_check_ms > 0) {
        if (!G.firmware_version || !G.ota_version_url) {
            ESP_LOGW(PX_TAG, "OTA auto-check enabled but firmware_version/ota_version_url missing");
        } else {
            ESP_LOGI(PX_TAG, "OTA configured: current=%s version_url=%s",
                     G.firmware_version, G.ota_version_url);
        }
    }

    snprintf(px_topic_tx, sizeof(px_topic_tx), "%s/TX", G.device_serial);
    snprintf(px_topic_rx, sizeof(px_topic_rx), "%s/RX", G.device_serial);
    snprintf(px_topic_status, sizeof(px_topic_status), "%s/status", G.device_serial);

    px_build_hostname_from_serial(G.device_serial, px_hostname, sizeof(px_hostname));

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
    px_last_roam_scan_ms = 0;
    px_last_roam_switch_ms = 0;
    px_wifi_roam_in_progress = false;
    px_app_started_flag = false;
    px_app_start_allowed = !px_ota_pending_verify;

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
