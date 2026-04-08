/**
 * @file ble_scanner.h
 * @brief ESP-IDF Bluedroid BLE Scanner for Remote ID Detection
 * 
 * This module implements BLE scanning using the ESP-IDF Bluedroid stack.
 * 
 * Configuration (via menuconfig/sdkconfig):
 *   - CONFIG_BT_BLE_42_FEATURES_SUPPORTED: Uses BLE 4.2 legacy scanning (31-byte payloads)
 *   - CONFIG_BT_BLE_50_FEATURES_SUPPORTED: Uses BLE 5.0 extended scanning (255-byte payloads)
 * 
 * For BLE 4.2 legacy mode, the scanner receives individual ODID messages that are
 * rotated by the broadcaster (Basic ID, Location, System, Self ID, Operator ID).
 * Multiple scan cycles are needed to capture all message types.
 */

#ifndef BLE_SCANNER_H
#define BLE_SCANNER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BLE detection callback function type
 * 
 * @param mac 6-byte MAC address of the detected device
 * @param rssi Signal strength in dBm
 * @param odid_data Pointer to ODID message data
 * @param odid_len Length of ODID data
 * @param is_extended true if from BLE 5.0 extended advertising
 */
typedef void (*ble_odid_callback_t)(const uint8_t *mac, int rssi, 
                                     const uint8_t *odid_data, size_t odid_len,
                                     bool is_extended);

/**
 * @brief BLE Scanner configuration
 */
typedef struct {
    uint16_t scan_interval;      // Scan interval in 0.625ms units (default: 160 = 100ms)
    uint16_t scan_window;        // Scan window in 0.625ms units (default: 80 = 50ms)
    bool active_scan;            // true for active scan, false for passive
    bool filter_duplicates;      // true to filter duplicate advertisements
    bool enable_extended;        // true to enable BLE 5.0 extended advertising scan
    ble_odid_callback_t callback; // Callback for ODID detections
} ble_scanner_config_t;

/**
 * @brief Default scanner configuration
 */
#define BLE_SCANNER_CONFIG_DEFAULT() { \
    .scan_interval = 160,              \
    .scan_window = 80,                 \
    .active_scan = false,              \
    .filter_duplicates = true,         \
    .enable_extended = true,           \
    .callback = NULL                   \
}

/**
 * @brief Initialize the BLE scanner
 * 
 * Initializes the Bluedroid stack and configures BLE scanning.
 * Must be called before starting scans.
 * 
 * @param config Scanner configuration
 * @return 0 on success, negative error code on failure
 */
int ble_scanner_init(const ble_scanner_config_t *config);

/**
 * @brief Start BLE scanning
 * 
 * Begins scanning for BLE advertisements containing Remote ID data.
 * Supports both legacy and extended advertising.
 * 
 * @param duration_ms Scan duration in milliseconds (0 = continuous)
 * @return 0 on success, negative error code on failure
 */
int ble_scanner_start(uint32_t duration_ms);

/**
 * @brief Stop BLE scanning
 * 
 * @return 0 on success, negative error code on failure
 */
int ble_scanner_stop(void);

/**
 * @brief Deinitialize the BLE scanner
 * 
 * Releases all BLE resources.
 */
void ble_scanner_deinit(void);

/**
 * @brief Check if BLE scanner is running
 * 
 * @return true if scanning is active
 */
bool ble_scanner_is_running(void);

/**
 * @brief Get BLE scanner statistics
 * 
 * @param total_adv Total advertisements received
 * @param odid_packets Total ODID packets detected
 * @param ble42_packets BLE 4.x legacy advertising ODID packets
 * @param ble50_packets BLE 5.0 extended advertising ODID packets
 */
void ble_scanner_get_stats(uint32_t *total_adv, uint32_t *odid_packets, uint32_t *ble42_packets, uint32_t *ble50_packets);

#ifdef __cplusplus
}
#endif

#endif // BLE_SCANNER_H
