#ifndef PEARLEXA_CONNECT_H
#define PEARLEXA_CONNECT_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---------- Library Config ----------
typedef struct {
    const char* device_serial;
    const char* auth_number;

    const char* mqtt_host;
    int         mqtt_tls_port;
    const char* mqtt_wss_uri;

    int wifi_strength_period_ms;

    // ---------- Optional Wi-Fi roaming config ----------
    // Universal feature for devices installed near multiple APs with the same SSID/password.
    // Disabled by default. Enable from Part B if needed.
    bool wifi_roaming_enable;
    int  wifi_roam_scan_period_ms;       // 0 = default 60000 ms
    int  wifi_roam_weak_rssi;            // 0 = default -75 dBm
    int  wifi_roam_margin_db;            // 0 = default 12 dB better AP required
    int  wifi_roam_min_switch_gap_ms;    // 0 = default 300000 ms

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

#ifdef __cplusplus
}
#endif

#endif // PEARLEXA_CONNECT_H
