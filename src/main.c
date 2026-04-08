/**
 * @file main.c
 * @brief ESP32-S3 Remote ID Scanner (ESP-IDF)
 *
 * This program implements a WiFi and BLE Remote ID (RID) scanner using ESP-IDF.
 * It scans for drone identification messages transmitted via WiFi (NAN Action Frames)
 * and BLE (legacy + extended advertising) according to the ASTM F3411 Remote ID standard.
 *
 * Key features:
 * - WiFi promiscuous mode scanning for ODID NAN frames and beacons
 * - BLE scanning with BLE 5.0 extended advertising support
 * - Real-time UAV tracking with location, altitude, and operator information
 * - Dual UART output (USB console + GPIO17 for aggregator)
 * - Channel hopping to improve detection across WiFi frequencies
 * - JSON output for integration with monitoring systems
 *
 * Hardware: ESP32-S3 with WiFi/BLE capabilities
 * Based on ASTM F3411 Remote ID Standard
 */

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "opendroneid.h"    
#include "odid_wifi.h"
#include "ble_scanner.h"

static const char *TAG = "RID_SCANNER";

// ============================================================================
// CONFIGURATION
// ============================================================================

#define ENABLE_BLE 1
#define ENABLE_WIFI 1

// UART1 Configuration (GPIO17 TX for aggregator)
#define UART1_TX_PIN    GPIO_NUM_17
#define UART1_RX_PIN    GPIO_NUM_16  // Not used, but required for config
#define UART1_BAUD      115200
#define UART1_PORT      UART_NUM_1

// Scanner settings
#define MAX_UAVS        64
#define CHANNEL_HOP_MS  500
// BLE 4.2 rotates through 5 message types at ~100ms intervals, need longer scan windows
#define BLE_SCAN_MS_SHORT   500    // Short scan when idle (1 sec to catch rotation)
#define BLE_SCAN_MS_LONG    1000    // Extended scan when ODID detected (capture all chunks)
#define BLE_PAUSE_MS        100     // Brief pause between scans
#define BLE_EXTEND_TIMEOUT  1000   // Time to keep extended scanning after last detection
#define STATUS_INTERVAL_MS  60000
#define BLE_OUTPUT_INTERVAL_MS 1000  // Rate-limit BLE JSON output to ~1Hz per drone

// Packet type definitions for interface field
#define PKT_TYPE_WIFI_BEACON    1   // WiFi Beacon frame
#define PKT_TYPE_WIFI_NAN       2   // WiFi NAN (Neighbor Awareness Networking) action frame
#define PKT_TYPE_BLE4_LEGACY    3   // BLE 4.x Legacy advertising
#define PKT_TYPE_BLE5_EXTENDED  4   // BLE 5.0 Extended advertising

// ============================================================================
// DATA STRUCTURES
// ============================================================================

typedef struct {
    uint8_t  mac[6];
    int      rssi;
    uint8_t  channel;
    uint8_t  rate;
    int8_t   noise_floor;
    uint32_t last_seen;
    uint8_t  interface;  // PKT_TYPE_WIFI_BEACON, PKT_TYPE_WIFI_NAN, PKT_TYPE_BLE4_LEGACY, PKT_TYPE_BLE5_EXTENDED
    char     op_id[ODID_ID_SIZE + 1];
    char     uav_id[ODID_ID_SIZE + 1];
    double   lat_d;
    double   long_d;
    double   base_lat_d;
    double   base_long_d;
    int      altitude_msl;
    int      height_agl;
    int      speed;
    int      heading;
    int      flag;
    uint32_t packet_count;
    uint8_t  id_type;
    uint8_t  ua_type;
    float    speed_vertical;
    uint8_t  status;
    uint16_t area_count;
    uint16_t area_radius;
    float    area_ceiling;
    float    area_floor;
    uint32_t last_output_ms;  // Timestamp of last JSON output (for rate-limiting)
} id_data_t;

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

static id_data_t uavs[MAX_UAVS] = {0};
static ODID_UAS_Data UAS_data;
static volatile uint32_t g_last_detection_ms = 0;
static volatile uint32_t g_last_ble_odid_ms = 0;   // Timestamp of last BLE ODID detection
static QueueHandle_t print_queue = NULL;
static uint32_t last_status_ms = 0;

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

static uint32_t millis(void) {
    return (uint32_t)(esp_timer_get_time() / 1000ULL);
}

/**
 * @brief Find or allocate a UAV tracking slot
 */
static id_data_t* next_uav(uint8_t* mac, uint8_t interface) {
    // First pass: Look for existing MAC + interface combination
    for (int i = 0; i < MAX_UAVS; i++) {
        if (memcmp(uavs[i].mac, mac, 6) == 0 && uavs[i].interface == interface)
            return &uavs[i];
    }
    // Second pass: Find an empty slot
    for (int i = 0; i < MAX_UAVS; i++) {
        if (uavs[i].mac[0] == 0)
            return &uavs[i];
    }
    // No empty slots, return first slot (overwrite oldest)
    return &uavs[0];
}

// ============================================================================
// JSON OUTPUT
// ============================================================================

/**
 * @brief Send UAV data as JSON
 */
static void send_json(const id_data_t *UAV) {
    char mac_str[18];
    snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             UAV->mac[0], UAV->mac[1], UAV->mac[2],
             UAV->mac[3], UAV->mac[4], UAV->mac[5]);
    
    char json_msg[896];
    const char* interface_str = "Unknown";
    switch (UAV->interface) {
        case PKT_TYPE_WIFI_BEACON:   interface_str = "WiFi Beacon"; break;
        case PKT_TYPE_WIFI_NAN:      interface_str = "WiFi NAN"; break;
        case PKT_TYPE_BLE4_LEGACY:   interface_str = "BLE4 Legacy"; break;
        case PKT_TYPE_BLE5_EXTENDED: interface_str = "BLE5 Extended"; break;
        default:                     interface_str = "Unknown"; break;
    }

    snprintf(json_msg, sizeof(json_msg),
        "{\"type\":\"drone\",\"mac\":\"%s\",\"interface\":\"%s\",\"rssi\":%d,"
        "\"channel\":%u,\"rate\":%u,\"noise_floor\":%d,"
        "\"id_type\":%u,\"ua_type\":%u,\"basic_id\":\"%s\","
        "\"drone_lat\":%.6f,\"drone_long\":%.6f,\"drone_altitude\":%d,\"height_agl\":%d,"
        "\"speed_horizontal\":%.1f,\"speed_vertical\":%.1f,\"heading\":%d,\"status\":%u,"
        "\"pilot_lat\":%.6f,\"pilot_long\":%.6f,"
        "\"area_count\":%u,\"area_radius\":%u,\"area_ceiling\":%.1f,\"area_floor\":%.1f,"
        "\"operator_id\":\"%s\"}\n",
        mac_str, interface_str, UAV->rssi,
        UAV->channel, UAV->rate, UAV->noise_floor,
        UAV->id_type, UAV->ua_type, UAV->uav_id,
        UAV->lat_d, UAV->long_d, UAV->altitude_msl, UAV->height_agl,
        (float)UAV->speed, UAV->speed_vertical, UAV->heading, UAV->status,
        UAV->base_lat_d, UAV->base_long_d,
        UAV->area_count, UAV->area_radius, UAV->area_ceiling, UAV->area_floor,
        UAV->op_id);
    
    // Send to USB console
    printf("%s", json_msg);
    
    // Send to UART1 (aggregator)
    uart_write_bytes(UART1_PORT, json_msg, strlen(json_msg));
}

static void log_boot_msg(const char *msg) {
    char json[256];
    snprintf(json, sizeof(json), "{\"type\":\"boot\",\"msg\":\"%s\"}\n", msg);
    printf("%s", json);
    uart_write_bytes(UART1_PORT, json, strlen(json));
}

// ============================================================================
// WIFI SCANNING
// ============================================================================

#if ENABLE_WIFI

/**
 * @brief Convert PHY rate encoding to Mbps for non-HT (11b/g) packets
 * Rate encoding from ESP-IDF wifi_pkt_rx_ctrl_t documentation
 */
static uint8_t phy_rate_to_mbps(uint8_t rate_encoding) {
    switch (rate_encoding) {
        case 0x00: return 1;    // 1 Mbps (11b)
        case 0x01: return 2;    // 2 Mbps (11b)
        case 0x02: return 6;    // 5.5 Mbps (11b) - round to 6
        case 0x03: return 11;   // 11 Mbps (11b)
        case 0x0B: return 6;    // 6 Mbps (11a/g)
        case 0x0F: return 9;    // 9 Mbps (11a/g)
        case 0x0A: return 12;   // 12 Mbps (11a/g)
        case 0x0E: return 18;   // 18 Mbps (11a/g)
        case 0x09: return 24;   // 24 Mbps (11a/g)
        case 0x0D: return 36;   // 36 Mbps (11a/g)
        case 0x08: return 48;   // 48 Mbps (11a/g)
        case 0x0C: return 54;   // 54 Mbps (11a/g)
        default:   return rate_encoding; // Return raw value if unknown
    }
}

/**
 * @brief Get effective data rate from rx_ctrl based on sig_mode
 * @param rx_ctrl Pointer to wifi_pkt_rx_ctrl_t structure
 * @return Data rate in Mbps (approximate for HT/VHT)
 * 
 * sig_mode: 0=non-HT(11bg), 1=HT(11n), 3=VHT(11ac)
 * For non-HT: use rate field (PHY rate encoding)
 * For HT/VHT: use mcs field to estimate rate
 */
static uint8_t get_wifi_rate_mbps(const wifi_pkt_rx_ctrl_t *rx_ctrl) {
    if (rx_ctrl->sig_mode == 0) {
        // Non-HT (11b/g) - rate field is valid PHY rate encoding
        return phy_rate_to_mbps(rx_ctrl->rate);
    } else if (rx_ctrl->sig_mode == 1) {
        // HT (11n) - use MCS to estimate rate
        // MCS 0-7 single stream: ~6.5 to 65 Mbps (20MHz), ~13.5 to 135 Mbps (40MHz)
        // Simplified: MCS * 7 + 6 gives rough Mbps estimate for 20MHz
        uint8_t mcs = rx_ctrl->mcs;
        if (mcs <= 7) {
            return (mcs * 7) + 6;  // MCS0=6, MCS7=55 (approximate)
        } else if (mcs <= 15) {
            return ((mcs - 8) * 7) + 13;  // 2-stream estimate
        }
        return 65;  // Default HT rate
    } else if (rx_ctrl->sig_mode == 3) {
        // VHT (11ac) - use MCS to estimate rate
        // VHT MCS 0-9 with various spatial streams
        uint8_t mcs = rx_ctrl->mcs;
        if (mcs <= 9) {
            return (mcs * 10) + 7;  // Rough estimate: MCS0=7, MCS9=97
        }
        return 100;  // Default VHT rate
    }
    
    // Unknown sig_mode, return raw rate
    return rx_ctrl->rate;
}

// Debug counters for NAN vs Beacon reception analysis
static volatile uint32_t g_wifi_mgmt_count = 0;
static volatile uint32_t g_nan_dest_match = 0;
static volatile uint32_t g_nan_parse_ok = 0;
static volatile uint32_t g_nan_parse_fail = 0;
static volatile uint32_t g_beacon_count = 0;
static volatile uint32_t g_beacon_odid_count = 0;
static volatile uint32_t g_beacon_vendor_ie = 0;  // Any vendor IE (0xDD)
static volatile uint32_t g_last_debug_print = 0;

/**
 * @brief WiFi promiscuous mode callback
 */
static void wifi_sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    if (type != WIFI_PKT_MGMT) return;
    
    g_wifi_mgmt_count++;
    
    wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)buf;
    uint8_t *payload = packet->payload;
    int length = packet->rx_ctrl.sig_len;
    
    // NAN destination check
    static const uint8_t nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};
    if (memcmp(nan_dest, &payload[4], 6) == 0) {
        g_nan_dest_match++;
        int parse_result = odid_wifi_receive_message_pack_nan_action_frame(&UAS_data, NULL, payload, length);
        if (parse_result == 0) {
            g_nan_parse_ok++;
            g_last_detection_ms = millis();
            
            id_data_t UAV;
            memset(&UAV, 0, sizeof(UAV));
            memcpy(UAV.mac, &payload[10], 6);
            UAV.rssi = packet->rx_ctrl.rssi;
            UAV.channel = packet->rx_ctrl.channel;
            UAV.rate = get_wifi_rate_mbps(&packet->rx_ctrl);
            UAV.noise_floor = packet->rx_ctrl.noise_floor;
            UAV.last_seen = millis();
            UAV.interface = PKT_TYPE_WIFI_NAN;
            
            if (UAS_data.BasicIDValid[0]) {
                strncpy(UAV.uav_id, (char *)UAS_data.BasicID[0].UASID, ODID_ID_SIZE);
                UAV.id_type = (uint8_t)UAS_data.BasicID[0].IDType;
                UAV.ua_type = (uint8_t)UAS_data.BasicID[0].UAType;
            }
            if (UAS_data.LocationValid) {
                UAV.lat_d = UAS_data.Location.Latitude;
                UAV.long_d = UAS_data.Location.Longitude;
                UAV.altitude_msl = (int)UAS_data.Location.AltitudeGeo;
                UAV.height_agl = (int)UAS_data.Location.Height;
                UAV.speed = (int)UAS_data.Location.SpeedHorizontal;
                UAV.heading = (int)UAS_data.Location.Direction;
                UAV.speed_vertical = UAS_data.Location.SpeedVertical;
                UAV.status = (uint8_t)UAS_data.Location.Status;
            }
            if (UAS_data.SystemValid) {
                UAV.base_lat_d = UAS_data.System.OperatorLatitude;
                UAV.base_long_d = UAS_data.System.OperatorLongitude;
                UAV.area_count = UAS_data.System.AreaCount;
                UAV.area_radius = UAS_data.System.AreaRadius;
                UAV.area_ceiling = UAS_data.System.AreaCeiling;
                UAV.area_floor = UAS_data.System.AreaFloor;
            }
            if (UAS_data.OperatorIDValid) {
                strncpy(UAV.op_id, (char *)UAS_data.OperatorID.OperatorId, ODID_ID_SIZE);
            }
            
            id_data_t* storedUAV = next_uav(UAV.mac, PKT_TYPE_WIFI_NAN);
            uint32_t pc = storedUAV->packet_count + 1;
            *storedUAV = UAV;
            storedUAV->packet_count = pc;
            storedUAV->flag = 1;
            
            // Queue for output
            xQueueSendFromISR(print_queue, &UAV, NULL);
        } else {
            g_nan_parse_fail++;
        }
    }
    // Beacon frame check (0x80)
    else if (payload[0] == 0x80) {
        g_beacon_count++;
        int offset = 36;
        while (offset < length) {
            int typ = payload[offset];
            int len = payload[offset + 1];
            if (typ == 0xdd && len >= 4) {
                g_beacon_vendor_ie++;
            }
            if ((typ == 0xdd) &&
                (((payload[offset + 2] == 0x90 && payload[offset + 3] == 0x3a && payload[offset + 4] == 0xe6)) ||
                 ((payload[offset + 2] == 0xfa && payload[offset + 3] == 0x0b && payload[offset + 4] == 0xbc)))) {
                int j = offset + 7;
                if (j < length) {
                    g_beacon_odid_count++;
                    memset(&UAS_data, 0, sizeof(UAS_data));
                    odid_message_process_pack(&UAS_data, &payload[j], length - j);
                    
                    g_last_detection_ms = millis();
                    id_data_t UAV;
                    memset(&UAV, 0, sizeof(UAV));
                    memcpy(UAV.mac, &payload[10], 6);
                    UAV.rssi = packet->rx_ctrl.rssi;
                    UAV.channel = packet->rx_ctrl.channel;
                    UAV.rate = get_wifi_rate_mbps(&packet->rx_ctrl);
                    UAV.noise_floor = packet->rx_ctrl.noise_floor;
                    UAV.last_seen = millis();
                    UAV.interface = PKT_TYPE_WIFI_BEACON;
                    
                    if (UAS_data.BasicIDValid[0]) {
                        strncpy(UAV.uav_id, (char *)UAS_data.BasicID[0].UASID, ODID_ID_SIZE);
                        UAV.id_type = (uint8_t)UAS_data.BasicID[0].IDType;
                        UAV.ua_type = (uint8_t)UAS_data.BasicID[0].UAType;
                    }
                    if (UAS_data.LocationValid) {
                        UAV.lat_d = UAS_data.Location.Latitude;
                        UAV.long_d = UAS_data.Location.Longitude;
                        UAV.altitude_msl = (int)UAS_data.Location.AltitudeGeo;
                        UAV.height_agl = (int)UAS_data.Location.Height;
                        UAV.speed = (int)UAS_data.Location.SpeedHorizontal;
                        UAV.heading = (int)UAS_data.Location.Direction;
                        UAV.speed_vertical = UAS_data.Location.SpeedVertical;
                        UAV.status = (uint8_t)UAS_data.Location.Status;
                    }
                    if (UAS_data.SystemValid) {
                        UAV.base_lat_d = UAS_data.System.OperatorLatitude;
                        UAV.base_long_d = UAS_data.System.OperatorLongitude;
                        UAV.area_count = UAS_data.System.AreaCount;
                        UAV.area_radius = UAS_data.System.AreaRadius;
                        UAV.area_ceiling = UAS_data.System.AreaCeiling;
                        UAV.area_floor = UAS_data.System.AreaFloor;
                    }
                    if (UAS_data.OperatorIDValid) {
                        strncpy(UAV.op_id, (char *)UAS_data.OperatorID.OperatorId, ODID_ID_SIZE);
                    }
                    
                    id_data_t* storedUAV = next_uav(UAV.mac, PKT_TYPE_WIFI_BEACON);
                    uint32_t pc = storedUAV->packet_count + 1;
                    *storedUAV = UAV;
                    storedUAV->packet_count = pc;
                    storedUAV->flag = 1;
                    
                    xQueueSendFromISR(print_queue, &UAV, NULL);
                }
            }
            offset += len + 2;
        }
    }
}
#endif // ENABLE_WIFI

// ============================================================================
// BLE SCANNING
// ============================================================================

#if ENABLE_BLE
/**
 * @brief Check if UAV data has minimum required fields for output
 * @return true if uav_id and location (lat/long) are populated
 */
static bool uav_has_minimum_data(const id_data_t *UAV) {
    // Must have uav_id (non-empty string)
    if (UAV->uav_id[0] == '\0') return false;
    
    // Must have valid latitude and longitude (non-zero, within valid range)
    // Note: 0,0 is technically valid but extremely unlikely for a real drone
    if (UAV->lat_d == 0.0 && UAV->long_d == 0.0) return false;
    if (UAV->lat_d < -90.0 || UAV->lat_d > 90.0) return false;
    if (UAV->long_d < -180.0 || UAV->long_d > 180.0) return false;
    
    return true;
}

/**
 * @brief BLE ODID callback - processes Remote ID data from BLE advertisements
 */
static void ble_odid_callback(const uint8_t *mac, int rssi, 
                               const uint8_t *odid_data, size_t odid_len,
                               bool is_extended) {
    // BLE 4.2 legacy may truncate to 23 bytes due to 31-byte payload limit
    // Accept packets >= 23 bytes for compatibility
    if (odid_data == NULL || odid_len < 23) return;
    
    // Update BLE detection timestamp for adaptive scan timing
    g_last_ble_odid_ms = millis();
    
    // Determine packet type based on BLE advertising type
    uint8_t pkt_type = is_extended ? PKT_TYPE_BLE5_EXTENDED : PKT_TYPE_BLE4_LEGACY;
    
    id_data_t* UAV = next_uav((uint8_t*)mac, pkt_type);
    UAV->last_seen = millis();
    UAV->rssi = rssi;
    UAV->interface = pkt_type;
    UAV->channel = 0;
    memcpy(UAV->mac, mac, 6);
    
    uint8_t msg_type = odid_data[0] & 0xF0;
    switch (msg_type) {
        case 0x00: {
            ODID_BasicID_data basic;
            decodeBasicIDMessage(&basic, (ODID_BasicID_encoded*)odid_data);
            strncpy(UAV->uav_id, (char*)basic.UASID, ODID_ID_SIZE);
            UAV->id_type = (uint8_t)basic.IDType;
            UAV->ua_type = (uint8_t)basic.UAType;
            break;
        }
        case 0x10: {
            ODID_Location_data loc;
            decodeLocationMessage(&loc, (ODID_Location_encoded*)odid_data);
            UAV->lat_d = loc.Latitude;
            UAV->long_d = loc.Longitude;
            UAV->altitude_msl = (int)loc.AltitudeGeo;
            UAV->height_agl = (int)loc.Height;
            UAV->speed = (int)loc.SpeedHorizontal;
            UAV->heading = (int)loc.Direction;
            UAV->speed_vertical = loc.SpeedVertical;
            UAV->status = (uint8_t)loc.Status;
            break;
        }
        case 0x40: {
            ODID_System_data sys;
            decodeSystemMessage(&sys, (ODID_System_encoded*)odid_data);
            UAV->base_lat_d = sys.OperatorLatitude;
            UAV->base_long_d = sys.OperatorLongitude;
            UAV->area_count = sys.AreaCount;
            UAV->area_radius = sys.AreaRadius;
            UAV->area_ceiling = sys.AreaCeiling;
            UAV->area_floor = sys.AreaFloor;
            break;
        }
        case 0x30: {
            // Self ID message - contains description text
            ODID_SelfID_data selfid;
            decodeSelfIDMessage(&selfid, (ODID_SelfID_encoded*)odid_data);
            // Self ID data can be logged but typically not stored in UAV struct
            ESP_LOGD(TAG, "Self ID: %s", selfid.Desc);
            break;
        }
        case 0x50: {
            ODID_OperatorID_data op;
            decodeOperatorIDMessage(&op, (ODID_OperatorID_encoded*)odid_data);
            strncpy(UAV->op_id, (char*)op.OperatorId, ODID_ID_SIZE);
            break;
        }
        case 0xF0: {
            ODID_UAS_Data ble_uas_data;
            memset(&ble_uas_data, 0, sizeof(ble_uas_data));
            odid_message_process_pack(&ble_uas_data, (uint8_t*)odid_data, odid_len);
            
            if (ble_uas_data.BasicIDValid[0]) {
                strncpy(UAV->uav_id, (char*)ble_uas_data.BasicID[0].UASID, ODID_ID_SIZE);
                UAV->id_type = (uint8_t)ble_uas_data.BasicID[0].IDType;
                UAV->ua_type = (uint8_t)ble_uas_data.BasicID[0].UAType;
            }
            if (ble_uas_data.LocationValid) {
                UAV->lat_d = ble_uas_data.Location.Latitude;
                UAV->long_d = ble_uas_data.Location.Longitude;
                UAV->altitude_msl = (int)ble_uas_data.Location.AltitudeGeo;
                UAV->height_agl = (int)ble_uas_data.Location.Height;
                UAV->speed = (int)ble_uas_data.Location.SpeedHorizontal;
                UAV->heading = (int)ble_uas_data.Location.Direction;
                UAV->speed_vertical = ble_uas_data.Location.SpeedVertical;
                UAV->status = (uint8_t)ble_uas_data.Location.Status;
            }
            if (ble_uas_data.SystemValid) {
                UAV->base_lat_d = ble_uas_data.System.OperatorLatitude;
                UAV->base_long_d = ble_uas_data.System.OperatorLongitude;
                UAV->area_count = ble_uas_data.System.AreaCount;
                UAV->area_radius = ble_uas_data.System.AreaRadius;
                UAV->area_ceiling = ble_uas_data.System.AreaCeiling;
                UAV->area_floor = ble_uas_data.System.AreaFloor;
            }
            if (ble_uas_data.OperatorIDValid) {
                strncpy(UAV->op_id, (char*)ble_uas_data.OperatorID.OperatorId, ODID_ID_SIZE);
            }
            break;
        }
    }
    
    UAV->packet_count++;
    UAV->flag = 1;
    
    // Only queue for output if minimum required fields are present (uav_id + location)
    // AND rate-limit to ~1Hz per drone to match WiFi output rate
    uint32_t now = millis();
    if (uav_has_minimum_data(UAV) && (now - UAV->last_output_ms) >= BLE_OUTPUT_INTERVAL_MS) {
        UAV->last_output_ms = now;
        id_data_t tmp = *UAV;
        xQueueSend(print_queue, &tmp, 0);
    }
}
#endif // ENABLE_BLE

// ============================================================================
// TASKS
// ============================================================================

/**
 * @brief WiFi channel hopping task
 */
#if ENABLE_WIFI
static void channel_hopper_task(void *pvParameters) {
    const uint8_t channels[] = {6, 1, 11};
    int ch_idx = 0;
    
    while (1) {
        if (millis() - g_last_detection_ms > 3000) {
            ch_idx = (ch_idx + 1) % 5;
            esp_wifi_set_channel(channels[ch_idx], WIFI_SECOND_CHAN_NONE);
        }
        vTaskDelay(pdMS_TO_TICKS(CHANNEL_HOP_MS));
    }
}
#endif

/**
 * @brief BLE scanning task with adaptive scan duration
 * 
 * Uses short scan windows when idle, extends to longer scans when
 * ODID packets are detected to capture all BLE 4.x chunked messages.
 */
#if ENABLE_BLE
static void ble_scan_task(void *pvParameters) {
    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for system stabilization
    
    log_boot_msg("BLE: Starting ESP-IDF Bluedroid initialization...");
    
    ble_scanner_config_t config = BLE_SCANNER_CONFIG_DEFAULT();
    config.scan_interval = 160;
    config.scan_window = 80;
    config.active_scan = false;
    config.filter_duplicates = false;
    config.enable_extended = true;
    config.callback = ble_odid_callback;
    
    int ret = ble_scanner_init(&config);
    if (ret != 0) {
        ESP_LOGE(TAG, "BLE init failed: %d", ret);
        vTaskDelete(NULL);
        return;
    }
    
    log_boot_msg("BLE: Bluedroid initialized successfully");
    
#if CONFIG_BT_BLE_50_FEATURES_SUPPORTED
    log_boot_msg("BLE: BLE 5.0 Extended Advertising support enabled");
#else
    log_boot_msg("BLE: BLE 4.x Legacy mode only");
#endif
    
    vTaskDelay(pdMS_TO_TICKS(500));
    log_boot_msg("BLE: Scanner starting adaptive scan loop");
    
    while (1) {
        uint32_t now = millis();
        
        // Adaptive scan duration: extend when ODID recently detected
        // This allows capturing all chunked BLE 4.x messages
        uint32_t scan_duration;
        if ((now - g_last_ble_odid_ms) < BLE_EXTEND_TIMEOUT) {
            // Recent detection - use extended scan to capture all message chunks
            scan_duration = BLE_SCAN_MS_LONG;
        } else {
            // Idle - use short scan for power efficiency
            scan_duration = BLE_SCAN_MS_SHORT;
        }
        
        ble_scanner_start(scan_duration);
        vTaskDelay(pdMS_TO_TICKS(scan_duration));
        ble_scanner_stop();
        
        // Clear flags for BLE interfaces (both BLE4 Legacy and BLE5 Extended)
        for (int i = 0; i < MAX_UAVS; i++) {
            if (uavs[i].flag && 
                (uavs[i].interface == PKT_TYPE_BLE4_LEGACY || 
                 uavs[i].interface == PKT_TYPE_BLE5_EXTENDED)) {
                uavs[i].flag = 0;
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(BLE_PAUSE_MS));
    }
}
#endif

/**
 * @brief Printer task - outputs JSON data from queue
 */
static void printer_task(void *pvParameters) {
    id_data_t UAV;
    while (1) {
        if (xQueueReceive(print_queue, &UAV, portMAX_DELAY)) {
            send_json(&UAV);
        }
    }
}

/**
 * @brief Status task - periodic status messages and debug output
 */
static void status_task(void *pvParameters) {
    static uint32_t last_debug_ms = 0;
    while (1) {
        uint32_t now = millis();
        
        // Debug output every 60 seconds to analyze packet reception
        if (now - last_debug_ms > 60000) {
            last_debug_ms = now;
            
#if ENABLE_WIFI
            char debug_msg[384];
            snprintf(debug_msg, sizeof(debug_msg),
                "{\"type\":\"wifi_debug\",\"mgmt_total\":%lu,\"nan_dest_match\":%lu,"
                "\"nan_parse_ok\":%lu,\"nan_parse_fail\":%lu,"
                "\"beacon_total\":%lu,\"beacon_vendor_ie\":%lu,\"beacon_odid\":%lu}\n",
                (unsigned long)g_wifi_mgmt_count,
                (unsigned long)g_nan_dest_match,
                (unsigned long)g_nan_parse_ok,
                (unsigned long)g_nan_parse_fail,
                (unsigned long)g_beacon_count,
                (unsigned long)g_beacon_vendor_ie,
                (unsigned long)g_beacon_odid_count);
            printf("%s", debug_msg);
            uart_write_bytes(UART1_PORT, debug_msg, strlen(debug_msg));
#endif

#if ENABLE_BLE
            // BLE debug output
            uint32_t ble_total_adv, ble_odid_pkts, ble42_pkts, ble50_pkts;
            ble_scanner_get_stats(&ble_total_adv, &ble_odid_pkts, &ble42_pkts, &ble50_pkts);
            char ble_debug_msg[256];
            snprintf(ble_debug_msg, sizeof(ble_debug_msg),
                "{\"type\":\"ble_debug\",\"adv_total\":%lu,\"odid_total\":%lu,"
                "\"ble42_odid\":%lu,\"ble50_odid\":%lu}\n",
                (unsigned long)ble_total_adv,
                (unsigned long)ble_odid_pkts,
                (unsigned long)ble42_pkts,
                (unsigned long)ble50_pkts);
            printf("%s", ble_debug_msg);
            uart_write_bytes(UART1_PORT, ble_debug_msg, strlen(ble_debug_msg));
#endif
        }
        
        if (now - last_status_ms > STATUS_INTERVAL_MS) {
            last_status_ms = now;
#if ENABLE_BLE
            const char *msg = "{\"type\":\"status\",\"msg\":\"Device is active - WiFi + BLE scanning enabled\"}\n";
#else
            const char *msg = "{\"type\":\"status\",\"msg\":\"Device is active - WiFi only (BLE disabled)\"}\n";
#endif
            printf("%s", msg);
            uart_write_bytes(UART1_PORT, msg, strlen(msg));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// MAIN
// ============================================================================

void app_main(void) {
    // Initial delay for aggregator boot
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "ESP32-S3 RID Scanner (ESP-IDF)");
    ESP_LOGI(TAG, "========================================");
    
    // Initialize UART1 for aggregator output
    uart_config_t uart_config = {
        .baud_rate = UART1_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(UART1_PORT, 256, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART1_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART1_PORT, UART1_TX_PIN, UART1_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    log_boot_msg("ESP32-S3 RID Scanner Starting...");
    log_boot_msg("Serial1 (UART1): 115200 baud initialized (GPIO17 TX)");
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    log_boot_msg("NVS Flash: Initialized");
    
    // Create print queue
    print_queue = xQueueCreate(MAX_UAVS, sizeof(id_data_t));
    if (print_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create print queue!");
    }
    log_boot_msg("Queue: Print queue created");

#if ENABLE_WIFI
    // Initialize WiFi
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(wifi_sniffer_cb));
    ESP_ERROR_CHECK(esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE));
    
    log_boot_msg("WiFi: Promiscuous mode enabled on channel 6");
#endif

    // Initialize UAV tracking array
    memset(uavs, 0, sizeof(uavs));
    log_boot_msg("Memory: UAV tracking array initialized");

#if ENABLE_BLE
    log_boot_msg("BLE: Initialization deferred to scan task");
#else
    log_boot_msg("BLE: Disabled (compile-time flag)");
#endif

    // Create tasks
    xTaskCreatePinnedToCore(printer_task, "printer", 8192, NULL, 2, NULL, 1);
    log_boot_msg("Task: PrinterTask started on core 1");

#if ENABLE_WIFI
    xTaskCreatePinnedToCore(channel_hopper_task, "channel_hop", 4096, NULL, 1, NULL, 0);
    log_boot_msg("Task: ChannelHopper started on core 0");
#endif

#if ENABLE_BLE
    xTaskCreatePinnedToCore(ble_scan_task, "ble_scan", 10000, NULL, 1, NULL, 1);
    log_boot_msg("Task: BLEScanTask started on core 1");
#endif

    xTaskCreatePinnedToCore(status_task, "status", 4096, NULL, 1, NULL, 0);
    log_boot_msg("Task: StatusTask started on core 0");

#if ENABLE_BLE
    log_boot_msg("==== SCANNER READY - WiFi + BLE scanning active ====");
#else
    log_boot_msg("==== SCANNER READY - WiFi scanning active (BLE disabled) ====");
#endif
}
