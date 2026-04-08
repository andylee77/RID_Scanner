/**
 * @file ble_scanner.c
 * @brief ESP-IDF Bluedroid BLE Scanner Implementation
 * 
 * Implements BLE scanning using ESP-IDF Bluedroid stack with support for:
 * - BLE 4.x legacy advertising (31-byte payloads) via esp_ble_gap_start_scanning()
 * - BLE 5.0 extended advertising (up to 255-byte payloads) via esp_ble_gap_start_ext_scan()
 * 
 * Remote ID Service UUID: 0xFFFA (ASTM F3411)
 * Remote ID Message Type: 0x0D
 * 
 * CONFIGURATION (via menuconfig):
 * - CONFIG_BT_BLE_42_FEATURES_SUPPORTED: Use legacy BLE 4.2 scanning APIs
 *   - Uses esp_ble_gap_set_scan_params() / esp_ble_gap_start_scanning()
 *   - Receives ESP_GAP_BLE_SCAN_RESULT_EVT events
 *   - BLE 4.2 broadcasts rotate through individual messages (Basic ID, Location, etc.)
 * 
 * - CONFIG_BT_BLE_50_FEATURES_SUPPORTED: Use extended BLE 5.0 scanning APIs
 *   - Uses esp_ble_gap_set_ext_scan_params() / esp_ble_gap_start_ext_scan()
 *   - Receives ESP_GAP_BLE_EXT_ADV_REPORT_EVT events
 *   - BLE 5.0 broadcasts can send full message packs in single advertisement
 */

#include "ble_scanner.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_defs.h"
#include "esp_gap_ble_api.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "BLE_SCANNER";

// Remote ID BLE identifiers
#define ODID_SERVICE_UUID_16    0xFFFA  // ASTM F3411 Service Data UUID
#define ODID_MFG_CODE           0x0200  // Intel Mfg Code (draft spec placeholder)
#define ODID_MESSAGE_TYPE       0x0D    // ODID Application Code

// Scanner state
static bool s_scanner_initialized = false;
static bool s_scanner_running = false;
static ble_scanner_config_t s_config;

// Statistics
static uint32_t s_total_advertisements = 0;
static uint32_t s_odid_packets = 0;
static uint32_t s_ble42_packets = 0;   // BLE 4.x legacy ODID packets
static uint32_t s_ble50_packets = 0;   // BLE 5.0 extended ODID packets

#if CONFIG_BT_BLE_42_FEATURES_SUPPORTED
// Legacy scan parameters (BLE 4.x) - only when 4.2 features are enabled
static esp_ble_scan_params_t s_scan_params = {
    .scan_type = BLE_SCAN_TYPE_PASSIVE,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_interval = 160,  // 100ms (160 * 0.625ms)
    .scan_window = 80,     // 50ms (80 * 0.625ms)
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE
};
#endif

#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
// Extended scan parameters (BLE 5.0)
static esp_ble_ext_scan_params_t s_ext_scan_params = {
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
    .scan_duplicate = BLE_SCAN_DUPLICATE_ENABLE,
    .cfg_mask = ESP_BLE_GAP_EXT_SCAN_CFG_UNCODE_MASK | ESP_BLE_GAP_EXT_SCAN_CFG_CODE_MASK,
    .uncoded_cfg = {
        .scan_type = BLE_SCAN_TYPE_PASSIVE,
        .scan_interval = 160,
        .scan_window = 80
    },
    .coded_cfg = {
        .scan_type = BLE_SCAN_TYPE_PASSIVE,
        .scan_interval = 160,
        .scan_window = 80
    }
};
#endif

/**
 * @brief Parse advertisement data to find ODID data
 * 
 * Supports two ODID BLE advertisement formats:
 * 
 * 1. ASTM F3411 Service Data (finalized standard):
 *    [length][0x16][0xFA][0xFF][0x0D][counter][data...]
 *    - 0x16 = Service Data AD Type (16-bit UUID)
 *    - 0xFFFA = Remote ID Service UUID (little-endian: 0xFA, 0xFF)
 *    - 0x0D = ODID Application Code
 *    - counter = Message counter byte
 *    - data = 25-byte ODID message(s)
 * 
 * 2. Draft Spec Manufacturer Specific (legacy/older transmitters):
 *    [length][0xFF][0x00][0x02][0x0D][counter][data...]
 *    - 0xFF = Manufacturer Specific AD Type
 *    - 0x0200 = Intel Mfg Code placeholder (little-endian: 0x00, 0x02)
 *    - 0x0D = ODID Application Code
 *    - counter = Message counter byte
 *    - data = 25-byte ODID message(s)
 * 
 * Per Open Drone ID BLE Broadcast Spec v0.64.3, Section 5.2.
 * 
 * @param adv_data Advertisement data buffer
 * @param adv_len Length of advertisement data
 * @param odid_data Output: pointer to ODID message data start (after app code + counter)
 * @param odid_len Output: length of ODID message data
 * @return true if ODID data found
 */
static bool parse_odid_from_adv(const uint8_t *adv_data, size_t adv_len,
                                 const uint8_t **odid_data, size_t *odid_len)
{
    if (adv_data == NULL || adv_len < 6) {
        return false;
    }
    
    size_t offset = 0;
    while (offset < adv_len - 1) {
        uint8_t field_len = adv_data[offset];
        if (field_len == 0 || offset + field_len >= adv_len) {
            break;
        }
        
        uint8_t ad_type = adv_data[offset + 1];
        
        // Format 1: ASTM F3411 Service Data - 16-bit UUID (AD Type 0x16)
        // Layout: [len][0x16][UUID_lo=0xFA][UUID_hi=0xFF][0x0D][counter][ODID msg...]
        if (ad_type == 0x16 && field_len >= 4) {
            uint16_t uuid = adv_data[offset + 2] | (adv_data[offset + 3] << 8);
            if (uuid == ODID_SERVICE_UUID_16) {
                if (field_len >= 5 && adv_data[offset + 4] == ODID_MESSAGE_TYPE) {
                    // Skip: ad_type(1) + uuid(2) + app_code(1) + counter(1) = 5
                    *odid_data = &adv_data[offset + 6];
                    *odid_len = field_len - 5;
                    return true;
                }
            }
        }
        
        // Format 2: Draft Spec Manufacturer Specific (AD Type 0xFF)
        // Layout: [len][0xFF][MfgCode_lo=0x00][MfgCode_hi=0x02][0x0D][counter][ODID msg...]
        // Mfg Code 0x0200 = Intel (placeholder from draft spec)
        if (ad_type == 0xFF && field_len >= 4) {
            uint16_t mfg_code = adv_data[offset + 2] | (adv_data[offset + 3] << 8);
            if (mfg_code == ODID_MFG_CODE) {
                if (field_len >= 5 && adv_data[offset + 4] == ODID_MESSAGE_TYPE) {
                    // Skip: ad_type(1) + mfg_code(2) + app_code(1) + counter(1) = 5
                    *odid_data = &adv_data[offset + 6];
                    *odid_len = field_len - 5;
                    return true;
                }
            }
        }
        
        offset += field_len + 1;
    }
    
    return false;
}

/**
 * @brief Process a BLE advertisement for ODID data
 */
static void process_advertisement(const uint8_t *mac, int rssi, 
                                   const uint8_t *adv_data, size_t adv_len,
                                   bool is_extended)
{
    s_total_advertisements++;
    
    const uint8_t *odid_data = NULL;
    size_t odid_len = 0;
    
    if (parse_odid_from_adv(adv_data, adv_len, &odid_data, &odid_len)) {
        s_odid_packets++;
        if (is_extended) {
            s_ble50_packets++;
        } else {
            s_ble42_packets++;
        }
        
        // Invoke callback if registered
        if (s_config.callback != NULL) {
            s_config.callback(mac, rssi, odid_data, odid_len, is_extended);
        }
    }
}

/**
 * @brief GAP event handler for BLE events
 */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
#if CONFIG_BT_BLE_42_FEATURES_SUPPORTED
        case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
            if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Legacy scan parameters set");
            } else {
                ESP_LOGE(TAG, "Failed to set scan parameters: %d", param->scan_param_cmpl.status);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
            if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGD(TAG, "Legacy scan started");  // Debug level - too noisy for INFO
                s_scanner_running = true;
            } else {
                ESP_LOGE(TAG, "Failed to start scan: %d", param->scan_start_cmpl.status);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
            if (param->scan_stop_cmpl.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGD(TAG, "Scan stopped");  // Debug level - too noisy for INFO
                s_scanner_running = false;
            } else {
                ESP_LOGE(TAG, "Failed to stop scan: %d", param->scan_stop_cmpl.status);
            }
            break;
            
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            // Handle different scan result events
            switch (param->scan_rst.search_evt) {
                case ESP_GAP_SEARCH_INQ_RES_EVT:
                    // Legacy advertisement (BLE 4.x)
                    process_advertisement(
                        param->scan_rst.bda,
                        param->scan_rst.rssi,
                        param->scan_rst.ble_adv,
                        param->scan_rst.adv_data_len,
                        false  // Not extended
                    );
                    break;
                    
                case ESP_GAP_SEARCH_INQ_CMPL_EVT:
                    // Scan completed (timed scan finished)
                    ESP_LOGD(TAG, "Scan completed (timed duration elapsed)");
                    s_scanner_running = false;
                    break;
                    
                default:
                    break;
            }
            break;
#endif

#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
        case ESP_GAP_BLE_SET_EXT_SCAN_PARAMS_COMPLETE_EVT:
            if (param->set_ext_scan_params.status == ESP_BT_STATUS_SUCCESS) {
                ESP_LOGI(TAG, "Extended scan parameters set");
            } else {
                ESP_LOGE(TAG, "Failed to set extended scan parameters: %d", 
                         param->set_ext_scan_params.status);
            }
            break;
            
        case ESP_GAP_BLE_EXT_SCAN_START_COMPLETE_EVT:
            if (param->ext_scan_start.status == ESP_BT_STATUS_SUCCESS) {
               // ESP_LOGI(TAG, "Extended scan started");
                s_scanner_running = true;
            } else {
                ESP_LOGE(TAG, "Failed to start extended scan: %d", 
                         param->ext_scan_start.status);
            }
            break;
            
        case ESP_GAP_BLE_EXT_SCAN_STOP_COMPLETE_EVT:
            if (param->ext_scan_stop.status == ESP_BT_STATUS_SUCCESS) {
              //  ESP_LOGI(TAG, "Extended scan stopped");
                s_scanner_running = false;
            } else {
                ESP_LOGE(TAG, "Failed to stop extended scan: %d", 
                         param->ext_scan_stop.status);
            }
            break;
            
        case ESP_GAP_BLE_EXT_ADV_REPORT_EVT:
            // Extended scan can receive both legacy and extended advertisements
            // Check event_type bit 4 (0x10) to determine if legacy advertising
            {
                esp_ble_gap_ext_adv_report_t *ext_adv = &param->ext_adv_report.params;
                // Bit 4 (0x10) of event_type indicates legacy advertising PDU
                bool is_legacy = (ext_adv->event_type & 0x10) != 0;
                process_advertisement(
                    ext_adv->addr,
                    ext_adv->rssi,
                    ext_adv->adv_data,
                    ext_adv->adv_data_len,
                    !is_legacy  // is_extended = NOT legacy
                );
            }
            break;
            
        // Handle legacy advertising for backward compatibility with BLE 4.2 devices
        case ESP_GAP_BLE_SCAN_RESULT_EVT:
            if (param->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_RES_EVT) {
                process_advertisement(
                    param->scan_rst.bda,
                    param->scan_rst.rssi,
                    param->scan_rst.ble_adv,
                    param->scan_rst.adv_data_len,
                    false  // Legacy advertising
                );
            }
            break;
#endif
            
        default:
            ESP_LOGD(TAG, "Unhandled GAP event: %d", event);
            break;
    }
}

int ble_scanner_init(const ble_scanner_config_t *config)
{
    if (s_scanner_initialized) {
        ESP_LOGW(TAG, "Scanner already initialized");
        return 0;
    }
    
    if (config == NULL) {
        ESP_LOGE(TAG, "Invalid configuration");
        return -1;
    }
    
    // Store configuration
    memcpy(&s_config, config, sizeof(ble_scanner_config_t));
    
    // Release BLE memory from classic BT (we only use BLE)
    esp_err_t ret = esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Failed to release classic BT memory: %s", esp_err_to_name(ret));
        // Continue anyway - might already be released
    }
    
    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize BT controller: %s", esp_err_to_name(ret));
        return -2;
    }
    
    // Enable BT controller in BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable BT controller: %s", esp_err_to_name(ret));
        esp_bt_controller_deinit();
        return -3;
    }
    
    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize Bluedroid: %s", esp_err_to_name(ret));
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return -4;
    }
    
    // Enable Bluedroid stack
    ret = esp_bluedroid_enable();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable Bluedroid: %s", esp_err_to_name(ret));
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return -5;
    }
    
    // Register GAP callback
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register GAP callback: %s", esp_err_to_name(ret));
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        esp_bt_controller_disable();
        esp_bt_controller_deinit();
        return -6;
    }

#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    // Always use extended scan when BLE 5.0 is supported
    // Configure extended scan parameters
    s_ext_scan_params.scan_duplicate = config->filter_duplicates ? 
                                       BLE_SCAN_DUPLICATE_ENABLE : BLE_SCAN_DUPLICATE_DISABLE;
    s_ext_scan_params.uncoded_cfg.scan_type = config->active_scan ? 
                                               BLE_SCAN_TYPE_ACTIVE : BLE_SCAN_TYPE_PASSIVE;
    s_ext_scan_params.uncoded_cfg.scan_interval = config->scan_interval;
    s_ext_scan_params.uncoded_cfg.scan_window = config->scan_window;
    s_ext_scan_params.coded_cfg.scan_type = s_ext_scan_params.uncoded_cfg.scan_type;
    s_ext_scan_params.coded_cfg.scan_interval = config->scan_interval;
    s_ext_scan_params.coded_cfg.scan_window = config->scan_window;
    
    ret = esp_ble_gap_set_ext_scan_params(&s_ext_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set extended scan params: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "BLE 5.0 extended scanning configured");
    
#elif CONFIG_BT_BLE_42_FEATURES_SUPPORTED
    // Configure legacy scan parameters
    s_scan_params.scan_type = config->active_scan ? BLE_SCAN_TYPE_ACTIVE : BLE_SCAN_TYPE_PASSIVE;
    s_scan_params.scan_interval = config->scan_interval;
    s_scan_params.scan_window = config->scan_window;
    s_scan_params.scan_duplicate = config->filter_duplicates ? 
                                   BLE_SCAN_DUPLICATE_ENABLE : BLE_SCAN_DUPLICATE_DISABLE;
    
    ret = esp_ble_gap_set_scan_params(&s_scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set scan parameters: %s", esp_err_to_name(ret));
    }
    ESP_LOGI(TAG, "BLE 4.2 legacy scanning configured");
    
#else
    #error "Neither BLE 4.2 nor BLE 5.0 features are supported!"
#endif
    
    s_scanner_initialized = true;
    s_total_advertisements = 0;
    s_odid_packets = 0;
    s_ble42_packets = 0;
    s_ble50_packets = 0;
    
    ESP_LOGI(TAG, "BLE scanner initialized");
    return 0;
}

int ble_scanner_start(uint32_t duration_ms)
{
    if (!s_scanner_initialized) {
        ESP_LOGE(TAG, "Scanner not initialized");
        return -1;
    }
    
    if (s_scanner_running) {
        ESP_LOGW(TAG, "Scanner already running");
        return 0;
    }
    
    esp_err_t ret;
    
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    // Start extended scan (BLE 5.0)
    // Duration is in 10ms units, 0 = scan until explicitly stopped
    uint16_t duration = (duration_ms > 0) ? (duration_ms / 10) : 0;
    ret = esp_ble_gap_start_ext_scan(duration, 0);  // period = 0 (continuous)
    
#elif CONFIG_BT_BLE_42_FEATURES_SUPPORTED
    // Start legacy scan
    uint32_t duration_sec = (duration_ms > 0) ? (duration_ms / 1000) : 0;
    ret = esp_ble_gap_start_scanning(duration_sec);
    
#else
    #error "Neither BLE 4.2 nor BLE 5.0 features are supported!"
#endif
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start scan: %s", esp_err_to_name(ret));
        return -2;
    }
    
    return 0;
}

int ble_scanner_stop(void)
{
    if (!s_scanner_initialized) {
        return -1;
    }
    
    if (!s_scanner_running) {
        return 0;
    }
    
    esp_err_t ret;
    
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    ret = esp_ble_gap_stop_ext_scan();
    
#elif CONFIG_BT_BLE_42_FEATURES_SUPPORTED
    ret = esp_ble_gap_stop_scanning();
    
#else
    #error "Neither BLE 4.2 nor BLE 5.0 features are supported!"
#endif
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop scan: %s", esp_err_to_name(ret));
        return -2;
    }
    
    return 0;
}

void ble_scanner_deinit(void)
{
    if (!s_scanner_initialized) {
        return;
    }
    
    if (s_scanner_running) {
        ble_scanner_stop();
    }
    
    esp_bluedroid_disable();
    esp_bluedroid_deinit();
    esp_bt_controller_disable();
    esp_bt_controller_deinit();
    
    s_scanner_initialized = false;
    s_scanner_running = false;
    
    ESP_LOGI(TAG, "BLE scanner deinitialized");
}

bool ble_scanner_is_running(void)
{
    return s_scanner_running;
}

void ble_scanner_get_stats(uint32_t *total_adv, uint32_t *odid_packets, uint32_t *ble42_packets, uint32_t *ble50_packets)
{
    if (total_adv) *total_adv = s_total_advertisements;
    if (odid_packets) *odid_packets = s_odid_packets;
    if (ble42_packets) *ble42_packets = s_ble42_packets;
    if (ble50_packets) *ble50_packets = s_ble50_packets;
}
