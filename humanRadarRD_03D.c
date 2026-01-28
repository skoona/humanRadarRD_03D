// humanRadarRD_03D.c

#include "humanRadarRD_03D.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/idf_additions.h"
#include "freertos/projdefs.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#pragma pack(1)

// frame header / trailer
typedef union 
{
    uint32_t frameWord;
    uint8_t frameBytes[4];
} FrameMarkerType;
// for identifying the frame type and getting the length
typedef struct 
{
    FrameMarkerType header;
    uint16_t ifDataLength;
} FrameStart;

typedef struct 
{
    FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
    uint16_t ifDataLength;
    uint16_t commandWord;
    FrameMarkerType trailer; // 04 03 02 01 0x01020304
} REQframeCommand;
typedef struct 
{
    FrameMarkerType header;  // FD FC FB FA == 0xfafbfcfd
    uint16_t ifDataLength;   // 04 00 == 0x0004
    uint16_t commandWord;    // FF 00 == 0x00ff
    uint16_t commandValue;   // 01 00 == 0x0001
    FrameMarkerType trailer; // 04 03 02 01 == 0x01020304
} REQframeCommandWithValue;
typedef struct 
{
    FrameMarkerType header;  // FD FC FB FA == 0xfafbfcfd
    uint16_t ifDataLength;   // 08 00 == 0x0008
    uint16_t commandWord;    // 07 00 == 0x0007
    uint16_t commandPar;     // 0x0000 - 0x0004
    uint32_t commandValue;   //
    FrameMarkerType trailer; // 04 03 02 01 == 0x01020304
} REQframeCommandValueWithPar;
typedef struct 
{
    FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
    uint16_t ifDataLength;  // 04 00 == 0x0004
    uint16_t commandReply;
    uint16_t ackStatus;
    FrameMarkerType trailer; // 04 03 02 01 0x01020304
} ACKframeCommand;
typedef struct 
{
    FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
    uint16_t ifDataLength;  // 08 00 == 0x0004
    uint16_t commandReply;  // FF 01 == 0x01ff
    uint16_t ackStatus;
    uint16_t protocolVer;
    uint16_t bufferSize;
    FrameMarkerType trailer; // 04 03 02 01 0x01020304
} ACKframeCommandModeEnter;

typedef struct 
{
    FrameMarkerType header;  // FD FC FB FA == 0xfafbfcfd
    uint16_t ifDataLength;   // 08 00 == 0x0008
    uint16_t commandWord;    // 12 00 == 0x0012 - SET_MODE
    uint16_t parWord;        // 00 00 == 0x0000
    uint32_t parValue;       // 0x00 / 0x04 / 0x64
    FrameMarkerType trailer; // 04 03 02 01 == 0x01020304
} REQframeSetSystemMode;
/// <summary>
/// struct representing the firmware version of the radar sensor (contains the
/// firmware type, major, minor and bugfix parts of the version)
/// </summary>
// 0000 01 00 01000000
typedef struct {
	uint16_t type;	 // firmware type
	uint8_t minor;	 // minor version of the radar firmware
	uint8_t major;	 // major version of the radar firmware
	uint32_t bugfix; // bug fix version of the radar firmware
} FirmwareVersion;
typedef struct 
{
    FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
    uint16_t ifDataLength;
    uint16_t commandReply;
    uint16_t ackStatus;
	uint16_t versionBytes; // version bytes length
	uint16_t type;	 // firmware type
	uint8_t  major;	 // major version of the radar firmware
	uint8_t  minor;	 // minor version of the radar firmware
	uint32_t bugfix; // bug fix version of the radar firmware
	FrameMarkerType trailer; // 04 03 02 01 0x01020304
} ACKframeFirmwareVersion;

typedef struct 
{
    FrameMarkerType header; // FD FC FB FA 0xfafbfcfd
    uint16_t ifDataLength;  // 08 00 == 0x0008
    uint16_t commandReply;  // FF 01 == 0x01ff
    uint16_t ackStatus;     // 0 / 1
    uint32_t parValue;
    FrameMarkerType trailer; // 04 03 02 01 0x01020304
} ACKframeParameter;

#pragma pack(0)

// Config Begin Command
uint8_t Config_Begin_CMD[14] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
// Single-Target Detection Commands
uint8_t Single_Target_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x80, 0x00, 0x04, 0x03, 0x02, 0x01};
// Multi-Target Detection Command
uint8_t Multi_Target_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x90, 0x00, 0x04, 0x03, 0x02, 0x01};
// Version Number Command
uint8_t Version_Number_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x00, 0x00, 0x04, 0x03, 0x02, 0x01};
// Set baud Rate Command
uint8_t Set_Baud_rate_CMD[14] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x31, 0x00, 0x04, 0x00, 0x04, 0x03, 0x02, 0x01};
// Config End Command
uint8_t Config_End_CMD[12] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};

// Private function to update retention logic
static void radar_sensor_update_retention(radar_sensor_t *sensor);

// determine if a single target has moved
bool radar_sensor_hasTargetMoved(radar_target_t *currentTargets,
					radar_target_t *priorTargets, int targetId) {
	if (currentTargets[targetId].detected && priorTargets[targetId].detected) {
		float deltaX = currentTargets[targetId].x - priorTargets[targetId].x;
		float deltaY = currentTargets[targetId].y - priorTargets[targetId].y;
		float distanceMoved = sqrtf(deltaX * deltaX + deltaY * deltaY);
		if (distanceMoved >= 75.0f) // Movement threshold in mm
		{
			return true; // A target has moved beyond the threshold
		}
	} else if (currentTargets[targetId].detected !=
			   priorTargets[targetId].detected) {
		return true; // Target appearance/disappearance is considered movement
	}
	return false; // No significant movement detected
}
// Position description functions
void radar_sensor_update_position_description(radar_target_t *target)
{
    if (!target || !target->detected)
    {
        strcpy(target->position_description, "No target");
        return;
    }

    float x = target->x;
    float y = target->y;
    float distance_m = target->distance / 1000.0f; // Convert mm to meters

    // Determine primary direction
    const char *horizontal = "";
    const char *vertical = "";
    const char *distance_desc = "";

    // Horizontal direction (X-axis)
    if (abs((int)x) < 100)
    {
        horizontal = "Center";
    }
    else if (x > 0)
    {
        if (x > 1000)
            horizontal = "Far Left";
        else if (x > 500)
            horizontal = "Left";
        else
            horizontal = "Near Left";
    }
    else
    {
        if (x < -1000)
            horizontal = "Far Right";
        else if (x < -500)
            horizontal = "Right";
        else
            horizontal = "Near Right";
    }

    // Vertical direction (Y-axis)
    if (abs((int)y) < 100)
    {
        vertical = "Center";
    }
    else if (y > 0)
    {
        if (y > 2000)
            vertical = "Far Forward";
        else if (y > 1000)
            vertical = "Forward";
        else
            vertical = "Near Forward";
    }
    else
    {
        if (y < -2000)
            vertical = "Far Behind";
        else if (y < -1000)
            vertical = "Behind";
        else
            vertical = "Near Behind";
    }

    // Distance description
    if (distance_m < 0.5f)
    {
        distance_desc = "Very Close";
    }
    else if (distance_m < 1.0f)
    {
        distance_desc = "Close";
    }
    else if (distance_m < 2.0f)
    {
        distance_desc = "Medium";
    }
    else if (distance_m < 4.0f)
    {
        distance_desc = "Far";
    }
    else
    {
        distance_desc = "Very Far";
    }

    // Create comprehensive description
    if (strcmp(horizontal, "Center") == 0 && strcmp(vertical, "Center") == 0)
    {
        snprintf(target->position_description, 64, "Directly at sensor (%.1fm)",
                 distance_m);
    }
    else if (strcmp(horizontal, "Center") == 0)
    {
        snprintf(target->position_description, 64, "%s - %s (%.1fm)", vertical,
                 distance_desc, distance_m);
    }
    else if (strcmp(vertical, "Center") == 0)
    {
        snprintf(target->position_description, 64, "%s - %s (%.1fm)", horizontal,
                 distance_desc, distance_m);
    }
    else
    {
        snprintf(target->position_description, 64, "%s %s - %s (%.1fm)", horizontal,
                 vertical, distance_desc, distance_m);
    }
}

const char *radar_sensor_get_quadrant_name(float x, float y)
{
    if (x >= 0 && y >= 0)
        return "Front-Right (Q1)";
    if (x < 0 && y >= 0)
        return "Front-Left (Q2)";
    if (x < 0 && y < 0)
        return "Back-Left (Q3)";
    if (x >= 0 && y < 0)
        return "Back-Right (Q4)";
    return "Unknown";
}

const char *radar_sensor_get_direction_description(float x,
                                                   float y,
                                                   float distance)
{
    static char desc[32];
    const char *quadrant = radar_sensor_get_quadrant_name(x, y);
    snprintf(desc, 32, "%s (%.1fm)", quadrant, distance / 1000.0f);
    return desc;
}

esp_err_t radar_sensor_init(radar_sensor_t *sensor,
                            uart_port_t uart_port,
                            gpio_num_t rx_pin,
                            gpio_num_t tx_pin)
{
    if (!sensor)
    {
        return ESP_ERR_INVALID_ARG;
    }

    sensor->uart_port = uart_port;
    sensor->rx_pin = rx_pin;
    sensor->tx_pin = tx_pin;
    sensor->buffer_index = 0;
    sensor->parser_state = WAIT_AA;

    // Initialize target structures
    for (int targetId = 0; targetId < RADAR_MAX_TARGETS; targetId++) {
        sensor->target[targetId].targetId = targetId;
        sensor->target[targetId].detected = false;
        sensor->target[targetId].x = 0.0f;
        sensor->target[targetId].y = 0.0f;
        sensor->target[targetId].speed = 0.0f;
        sensor->target[targetId].distance = 0.0f;
        sensor->target[targetId].angle = 0.0f;
        strcpy(sensor->target[targetId].position_description, "No target");

        sensor->raw_target[targetId] = sensor->target[targetId]; // Initialize raw target same as filtered

        // Initialize retention system with default values
        sensor->retention[targetId].detection_retention_ms =
            RADAR_DEFAULT_DETECTION_RETENTION_MS;
        sensor->retention[targetId].absence_retention_ms = RADAR_DEFAULT_ABSENCE_RETENTION_MS;
        sensor->retention[targetId].last_detection_time = 0;
        sensor->retention[targetId].last_absence_time = 0;
        sensor->retention[targetId].raw_detected = false;
        sensor->retention[targetId].filtered_detected = false;
        sensor->retention[targetId].retention_enabled = true; // Enable by default
    }
    ESP_LOGI("mmWave",
             "Radar sensor initialized with retention: detection=%lu ms, "
             "absence=%lu ms",
             sensor->retention[0].detection_retention_ms,
             sensor->retention[0].absence_retention_ms);

    return ESP_OK;
}

esp_err_t radar_sensor_begin(radar_sensor_t *sensor, uint32_t baud_rate)
{
    if (!sensor)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    esp_err_t ret = uart_param_config(sensor->uart_port, &uart_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE("mmWave", "Failed to configure UART parameters");
        return ret;
    }

    ret = uart_set_pin(sensor->uart_port, sensor->tx_pin, sensor->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK)
    {
        ESP_LOGE("mmWave", "Failed to set UART pins");
        return ret;
    }

    // device buffer is typically 64 bytes
    ret = uart_driver_install(sensor->uart_port, 640, 640, 0, NULL, 0);
    if (ret != ESP_OK)
    {
        ESP_LOGE("mmWave", "Failed to install UART driver");
        return ret;
    }

    return ESP_OK;
}

bool radar_sensor_parse_data(radar_sensor_t *sensor,
                             const uint8_t *buf,
                             size_t len)
{
    if (!sensor || !buf || len != RADAR_FRAME_SIZE)
    {
        return false;
    }

    int target_offset = 0;
    for (int targetId = 0; targetId < RADAR_MAX_TARGETS; targetId++)
    {

        // Parse first 8 bytes for the first target
        int16_t raw_x = buf[target_offset] | (buf[target_offset + 1] << 8);
        target_offset += 2;
        int16_t raw_y = buf[target_offset] | (buf[target_offset + 1] << 8);
        target_offset += 2;
        int16_t raw_speed = buf[target_offset] | (buf[target_offset + 1] << 8);
        target_offset += 2;
        uint16_t raw_pixel_dist = buf[target_offset] | (buf[target_offset + 1] << 8);
        target_offset += 2;

        // Store raw target data (unfiltered)
        sensor->raw_target[targetId].detected =
            !(raw_x == 0 && raw_y == 0 && raw_speed == 0 && raw_pixel_dist == 0);

        // Parse signed values
        sensor->raw_target[targetId].x = ((raw_x & 0x8000) ? 1 : -1) * (raw_x & 0x7FFF);
        sensor->raw_target[targetId].y = ((raw_y & 0x8000) ? 1 : -1) * (raw_y & 0x7FFF);
        sensor->raw_target[targetId].speed = ((raw_speed & 0x8000) ? 1 : -1) * (raw_speed & 0x7FFF);

        if (sensor->raw_target[targetId].detected)
        {
            sensor->raw_target[targetId].distance =
                sqrtf(sensor->raw_target[targetId].x * sensor->raw_target[targetId].x +
                      sensor->raw_target[targetId].y * sensor->raw_target[targetId].y);

            // Angle calculation (convert radians to degrees, then flip)
            float angle_rad = atan2f(sensor->raw_target[targetId].y, sensor->raw_target[targetId].x) - (M_PI / 2.0f);
            float angle_deg = angle_rad * (180.0f / M_PI);
            sensor->raw_target[targetId].angle = -angle_deg; // align angle with x measurement positive/negative sign

            // Update position description
            radar_sensor_update_position_description(&sensor->raw_target[targetId]);
        }
        else
        {
            sensor->raw_target[targetId].distance = 0.0f;
            sensor->raw_target[targetId].angle = 0.0f;
            strcpy(sensor->raw_target[targetId].position_description, "No target");
        }

        // Update retention state
        sensor->retention[targetId].raw_detected = sensor->raw_target[targetId].detected;
    }

    return true;
}

/*
 * Data frame format (multi-target mode):
 * 0xAA 0xFF 0x03 0x00                          Header
 * 0x05 0x01 0x19 0x82 0x00 0x00 0x68 0x01      target1
 * 0xE3 0x81 0x33 0x88 0x20 0x80 0x68 0x01      target2
 * 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00      target3
 * 0x55 0xCC                                    Trailer
 *
 * Each target consists of 8 bytes:
 * Bytes 0-1: X position (little-endian int16_t, in mm)
 * Bytes 2-3: Y position (little-endian int16_t, in mm)
 * Bytes 4-5: Speed (little-endian int16_t, in mm/s)
 * Bytes 6-7: Pixel distance (little-endian uint16_t, in pixels)
 *
*/
bool radar_sensor_update(radar_sensor_t *sensor)
{
    if (!sensor)
    {
        return false;
    }

    bool data_updated = false;
    uint8_t byte_in;

    while (uart_read_bytes(sensor->uart_port, &byte_in, 1, 0) > 0)
    {
        switch (sensor->parser_state)
        {
        case WAIT_AA:
            if (byte_in == 0xAA)
            {
                sensor->parser_state = WAIT_FF;
            }
            break;

        case WAIT_FF:
            if (byte_in == 0xFF)
            {
                sensor->parser_state = WAIT_03;
            }
            else
            {
                sensor->parser_state = WAIT_AA;
            }
            break;

        case WAIT_03:
            if (byte_in == 0x03)
            {
                sensor->parser_state = WAIT_00;
            }
            else
            {
                sensor->parser_state = WAIT_AA;
            }
            break;

        case WAIT_00:
            if (byte_in == 0x00)
            {
                sensor->buffer_index = 0;
                sensor->parser_state = RECEIVE_FRAME;
            }
            else
            {
                sensor->parser_state = WAIT_AA;
            }
            break;

        case RECEIVE_FRAME:
            sensor->buffer[sensor->buffer_index++] = byte_in;
            if (sensor->buffer_index >= RADAR_FULL_FRAME_SIZE)
            {
                // Check tail bytes
                if (sensor->buffer[24] == 0x55 && sensor->buffer[25] == 0xCC)
                {
                    data_updated = radar_sensor_parse_data(sensor, sensor->buffer,
                                                           RADAR_FRAME_SIZE);
                }
                sensor->parser_state = WAIT_AA;
                sensor->buffer_index = 0;
            }
            break;
        }
    }

    // Always update retention logic, even if no new data
    radar_sensor_update_retention(sensor);

    return data_updated;
}

static void radar_sensor_update_retention(radar_sensor_t *sensor)
{
    TickType_t current_time = xTaskGetTickCount();

    for (int targetId = 0; targetId < RADAR_MAX_TARGETS; targetId++) {

        if (!sensor->retention[targetId].retention_enabled)
        {
            // If retention is disabled, just pass through raw data
            sensor->target[targetId] = sensor->raw_target[targetId];
            sensor->retention[targetId].filtered_detected = sensor->retention[targetId].raw_detected;
            return;
        }

        bool state_changed = false;

        // Update timing based on raw detection state
        if (sensor->retention[targetId].raw_detected)
        {
            sensor->retention[targetId].last_detection_time = current_time;
        }
        else
        {
            sensor->retention[targetId].last_absence_time = current_time;
        }

        // State machine for filtered detection
        if (sensor->retention[targetId].filtered_detected)
        {
            // Currently in "detected" state
            if (sensor->retention[targetId].raw_detected)
            {
                // Still detecting, stay in detected state
                // Copy the latest raw target data to filtered target
                sensor->target[targetId] = sensor->raw_target[targetId];
            }
            else
            {
                // No longer detecting raw target
                uint32_t time_since_detection =
                    (current_time - sensor->retention[targetId].last_detection_time) *
                    portTICK_PERIOD_MS;

                if (time_since_detection >= sensor->retention[targetId].detection_retention_ms)
                {
                    // Retention time exceeded, switch to not detected
                    sensor->retention[targetId].filtered_detected = false;
                    sensor->target[targetId].detected = false;
                    sensor->target[targetId].x = 0.0f;
                    sensor->target[targetId].y = 0.0f;
                    sensor->target[targetId].speed = 0.0f;
                    sensor->target[targetId].distance = 0.0f;
                    sensor->target[targetId].angle = 0.0f;
                    strcpy(sensor->target[targetId].position_description, "No target");
                    state_changed = true;

                    ESP_LOGI("mmWave", "Target lost after %lu ms retention",
                            time_since_detection);
                }
                // else: stay in detected state (retention active)
            }
        }
        else
        {
            // Currently in "not detected" state
            if (sensor->retention[targetId].raw_detected)
            {
                // Raw target detected
                uint32_t time_since_absence =
                    (current_time - sensor->retention[targetId].last_absence_time) *
                    portTICK_PERIOD_MS;

                if (time_since_absence >= sensor->retention[targetId].absence_retention_ms)
                {
                    // Absence retention time exceeded, confirm detection
                    sensor->retention[targetId].filtered_detected = true;
                    sensor->target[targetId] = sensor->raw_target[targetId];
                    state_changed = true;

                    ESP_LOGI("mmWave", "Target confirmed after %lu ms absence retention",
                            time_since_absence);
                }
                // else: stay in not detected state (absence retention active)
            }
            else
            {
                // Still no raw target, stay in not detected state
                // Keep target data as zeros (already set)
            }
        }

        if (state_changed)
        {
            ESP_LOGI("mmWave", "Retention state changed: %s -> %s",
                    sensor->retention[targetId].filtered_detected ? "NOT_DETECTED" : "DETECTED",
                    sensor->retention[targetId].filtered_detected ? "DETECTED" : "NOT_DETECTED");
        }
    }
}

int radar_sensor_get_target_count(radar_sensor_t *sensor)
{
    if (!sensor)
    {
        return 0;
    }

    int count = 0;
    for (int targetId = 0; targetId < RADAR_MAX_TARGETS; targetId++) {
        if (sensor->target[targetId].detected) {
            count = targetId + 1;
        }
    }
    return count;
}
int radar_sensor_get_raw_target_count(radar_sensor_t *sensor)
{
    if (!sensor)
    {
        return 0;
    }

    int count = 0;
    for (int targetId = 0; targetId < RADAR_MAX_TARGETS; targetId++)
    {
        if (sensor->raw_target[targetId].detected)
        {
            count = targetId+1;
        }
    }
    return count;
}

int radar_sensor_get_targets(radar_sensor_t *sensor, radar_target_t *targets)
{
    if (!sensor) {
        return 0;
    }
    memcpy(targets, sensor->target, sizeof(sensor->target));
    // Returns filtered target
    return radar_sensor_get_target_count(sensor);
}

int radar_sensor_get_raw_targets(radar_sensor_t *sensor, radar_target_t *targets)
{
    if (!sensor) {
        return 0;
    }
    memcpy(targets, sensor->raw_target, sizeof(sensor->raw_target));
    // Returns unfiltered raw target
    return radar_sensor_get_raw_target_count(sensor);
}

void radar_sensor_set_retention_times(radar_sensor_t *sensor,
                                      uint32_t detection_retention_ms,
                                      uint32_t absence_retention_ms)
{
    if (!sensor)
    {
        return;
    }

    for (int targetId = 0; targetId < RADAR_MAX_TARGETS; targetId++) {
    sensor->retention[targetId].detection_retention_ms = detection_retention_ms;
    sensor->retention[targetId].absence_retention_ms = absence_retention_ms;
    }

    ESP_LOGI("mmWave", "Retention times updated: detection=%lu ms, absence=%lu ms",
             detection_retention_ms, absence_retention_ms);
}

void radar_sensor_enable_retention(radar_sensor_t *sensor, bool enable)
{
    if (!sensor)
    {
        return;
    }

    for (int targetId = 0; targetId < RADAR_MAX_TARGETS; targetId++) {
        sensor->retention[targetId].retention_enabled = enable;

        if (!enable)
        {
            // If disabling retention, immediately sync filtered state with raw state
            sensor->target[targetId] = sensor->raw_target[targetId];
            sensor->retention[targetId].filtered_detected = sensor->retention[targetId].raw_detected;
        }
    }
    ESP_LOGI("mmWave", "Target retention %s", enable ? "enabled" : "disabled");
}

void radar_sensor_reset_retention(radar_sensor_t *sensor)
{
    if (!sensor)
    {
        return;
    }

    TickType_t current_time = xTaskGetTickCount();
    for (int targetId = 0; targetId < RADAR_MAX_TARGETS; targetId++) {
        sensor->retention[targetId].last_detection_time = current_time;
        sensor->retention[targetId].last_absence_time = current_time;
        sensor->retention[targetId].filtered_detected = sensor->retention[targetId].raw_detected;

        if (sensor->retention[targetId].raw_detected)
        {
            sensor->target[targetId] = sensor->raw_target[targetId];
        }
        else
        {
            sensor->target[targetId].detected = false;
            sensor->target[targetId].x = 0.0f;
            sensor->target[targetId].y = 0.0f;
            sensor->target[targetId].speed = 0.0f;
            sensor->target[targetId].distance = 0.0f;
            sensor->target[targetId].angle = 0.0f;
            strcpy(sensor->target[targetId].position_description, "No target");
        }
    }

    ESP_LOGI("mmWave", "Retention state reset");
}

bool radar_sensor_is_retention_active(radar_sensor_t *sensor)
{
    if (!sensor || !sensor->retention[0].retention_enabled)
    {
        return false;
    }

    // Retention is active if filtered state differs from raw state
    return (sensor->retention[0].filtered_detected !=
            sensor->retention[0].raw_detected);
}

uint32_t radar_sensor_get_time_since_last_detection(radar_sensor_t *sensor, int targetId)
{
    if (!sensor)
    {
        return 0;
    }

    TickType_t current_time = xTaskGetTickCount();
    return (current_time - sensor->retention[targetId].last_detection_time) *
           portTICK_PERIOD_MS;
}

uint32_t radar_sensor_get_time_since_last_absence(radar_sensor_t *sensor, int targetId)
{
    if (!sensor)
    {
        return 0;
    }

    TickType_t current_time = xTaskGetTickCount();
    return (current_time - sensor->retention[targetId].last_absence_time) *
           portTICK_PERIOD_MS;
}

void radar_sensor_deinit(radar_sensor_t *sensor)
{
    if (sensor)
    {
        uart_driver_delete(sensor->uart_port);
        ESP_LOGI("mmWave", "Radar sensor deinitialized");
    }
}

// ack_buffer_size should be at least expected ack frame size
esp_err_t Read_Command_Ack(radar_sensor_t *sensor, uint8_t *ack_buffer, size_t ack_buffer_size)
{
    if (!sensor || !ack_buffer || ack_buffer_size < RADAR_ACK_SIZE)
    {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total_bytes_read = 0;
    TickType_t start_time = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(2000); // 2 second timeout

    while (total_bytes_read < ack_buffer_size)
    {
        size_t bytes_read = uart_read_bytes(sensor->uart_port,
                                            (uint8_t *)(ack_buffer + total_bytes_read),
                                            ack_buffer_size - total_bytes_read,
                                            pdMS_TO_TICKS(100)); // 100 ms per read

        if (bytes_read > 0)
        {
            total_bytes_read += bytes_read;
        }

        // Check for timeout
        if ((xTaskGetTickCount() - start_time) > timeout_ticks)
        {
            ESP_LOGE("mmWave", "Timeout waiting for ACK");
            return ESP_ERR_TIMEOUT;
        }
    }

    ESP_LOGI("mmWave", "Received ACK (%d bytes)", total_bytes_read);
    ESP_LOG_BUFFER_HEX("UARTReceive", ack_buffer, total_bytes_read);
    return ESP_OK;
}

// Send command buffer and read ack buffer
// ack_buffer_size should be at least expected ack frame size
esp_err_t Send_Command(radar_sensor_t *sensor, uint8_t *cmd, size_t cmd_len, uint8_t *ack_buffer, size_t ack_buffer_size)
{
    if (!sensor || !cmd || cmd_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    
    uart_flush_input(CONFIG_UART_PORT); // Clear UART buffer before sending command

    int bytes_written = uart_write_bytes(sensor->uart_port, (const char *)cmd, cmd_len);
    if (bytes_written < 0) {
        ESP_LOGE("mmWave", "Failed to send command over UART");
        return ESP_FAIL;
    }

    ESP_LOGI("mmWave", "Sent command (%d bytes)", bytes_written);
    ESP_LOG_BUFFER_HEX("UARTSend", cmd, bytes_written);
    return Read_Command_Ack(sensor, ack_buffer, ack_buffer_size);
}

/* Configuration Commands
 * - set config mode        FDFCFBFA 0400 FF00 0100 04030201
 *    ack:                  FDFCFBFA 0800 FF01 0000 0100 4000 04030201 (18)
 *    fmt:    header,dLen,cWord,ack,protoVer,bufSz,end
 * - end config mode        FDFCFBFA 0200 FE00 04030201
 *    ack:                  FDFCFBFA 0400 FF01 0000 04030201 (14)
 *    fmt:    header,dLen,cWord,ack,end
 * - get version number     FDFCFBFA 0200 0000 04030201
 *    ack:                  FDFCFBFA 0E00 0001 0000 0080 0000010001000000 04030201 (24)
 *    fmt:    header,dLen,cWord,ack,vlen,vnum{8}, end
 * - set baud rate          FDFCFBFA 0400 3100 0400 04030201
 *    ack:                  FDFCFBFA 0400 3101 0000 04030201 (14)
 *    choices: 57600,  115200, 256000, 460800
 *             0x0004, 0x0005, 0x0007, 0x0008
 *     fmt:    header,dLen,cWord,ack,end
 * - set multi-target mode  FDFCFBFA 0200 9000 04030201
 *    ack:                  FDFCFBFA 0400 9001 0000 04030201 (14)
 *    fmt:     header,dLen,cWord,ack,end
 * - set single-target mode FDFCFBFA 0200 8000 04030201
 *    ack:                  FDFCFBFA 0400 8001 0000 04030201 (14)
 *    fmt:     header,dLen,cWord,ack,end
 */
esp_err_t radar_sensor_set_config_mode(radar_sensor_t *sensor, bool enable) {
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd[14];
    uint8_t ack[18];
    size_t cmd_len = 0, ack_len = 0;

    if (enable) {
        // Set config mode command
        memcpy(cmd, Config_Begin_CMD, sizeof(Config_Begin_CMD));
        cmd_len = sizeof(Config_Begin_CMD);
        ack_len = 18; // Expecting 18-byte ack
        sensor->configMode = true;
    } else {
        // End config mode command
        memcpy(cmd, Config_End_CMD, sizeof(Config_End_CMD));
        cmd_len = sizeof(Config_End_CMD);
        ack_len = 14; // Expecting 14-byte ack
        sensor->configMode = false;
    }

    esp_err_t ret = Send_Command(sensor, cmd, cmd_len, ack, ack_len);
    if (ret != ESP_OK) {
        ESP_LOGE("mmWave", "Failed to %s config mode", enable ? "enter" : "exit");
        return ret;
    }
/* zero based: 
 * -  8 = status (0x00 0x00 =success)
 * - 10 = Protocol Version (0x10 0x00)
 * - 12 = Buffer Size (0x40 0x00) = 64 bytes (little-endian)
*/
    // 9,8 = Status 0x00, 0x00
    if (ack[8] != 0x00 || ack[9] != 0x00)
    {
        ESP_LOGE("mmWave", "Error in ACK while changing command mode");
        return ESP_FAIL;
    }

    ESP_LOGI("mmWave", "%s config mode successful", enable ? "Entered" : "Exited");
    if (enable) {
        ACKframeCommandModeEnter *ackFrame = (ACKframeCommandModeEnter *)ack;
        ESP_LOGI("mmWave", "Protocol version:%d, Buffer size: %d", ackFrame->protocolVer, ackFrame->bufferSize);        
    }
    return ESP_OK;
}
esp_err_t radar_sensor_get_firmware_version(radar_sensor_t *sensor, char *outVersionString)
{
	if (!sensor || !outVersionString) {
		return ESP_ERR_INVALID_ARG;
	}

	bool was_in_config_mode = sensor->configMode;
    if (!sensor->configMode) {
        ESP_ERROR_CHECK(radar_sensor_set_config_mode(sensor, true));
    }

    uint8_t ack[24];
    esp_err_t ret = Send_Command(sensor, Version_Number_CMD, sizeof(Version_Number_CMD), ack, sizeof(ack));
    if (ret != ESP_OK) {
        ESP_LOGE("mmWave", "Failed to get firmware version");
        return ret;
    }

	if (!was_in_config_mode) {
        ESP_ERROR_CHECK(radar_sensor_set_config_mode(sensor, false));
    }

    // 9,8 = Status 0x00, 0x00
    ACKframeFirmwareVersion *ackFrame = (ACKframeFirmwareVersion *)ack;
    if (ackFrame->ackStatus != 0)
    {
        ESP_LOGE("mmWave", "Error in ACK while getting firmware version");
        return ESP_FAIL;
    }

	sprintf(outVersionString, "%d.%d.%ld", ackFrame->major, ackFrame->minor,
			(ackFrame->bugfix >= 32768 ? -1 : ackFrame->bugfix));

	ESP_LOGI("mmWave", outVersionString);
    
	return ESP_OK;
}
esp_err_t radar_sensor_set_baud_rate(radar_sensor_t *sensor, uint32_t baud_rate) {
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t baud_code;
    switch (baud_rate) {
        case 57600:
            baud_code = 0x0004;  // need to be reversed
            break;
        case 115200:
            baud_code = 0x0005;
            break;
        case 256000:
            baud_code = 0x0007;
            break;
        case 460800:
            baud_code = 0x0008;
            break;
        default:
            ESP_LOGE("mmWave", "Unsupported baud rate: %u", baud_rate);
            return ESP_ERR_INVALID_ARG;
    }

    bool was_in_config_mode = sensor->configMode;
    if (!sensor->configMode)
    {
        ESP_ERROR_CHECK(radar_sensor_set_config_mode(sensor, true));
    }

    uint8_t cmd[14];
    memcpy(cmd, Set_Baud_rate_CMD, sizeof(Set_Baud_rate_CMD));
    // Insert baud code into command (little-endian)
    cmd[8] = (uint8_t)(baud_code & 0xFF);
    cmd[9] = (uint8_t)((baud_code >> 8) & 0xFF);

    uint8_t ack[14];
    esp_err_t ret = Send_Command(sensor, cmd, sizeof(cmd), ack, sizeof(ack));
    if (ret != ESP_OK) {
        ESP_LOGE("mmWave", "Failed to set baud rate to %u", baud_rate);
        return ret;
    }
    if (!was_in_config_mode)
    {
        ESP_ERROR_CHECK(radar_sensor_set_config_mode(sensor, false));
    }

    // 9,8 = Status 0x00, 0x00 
    if (ack[8] != 0x00 || ack[9] != 0x00) {
        ESP_LOGE("mmWave", "Error in ACK while setting baud rate");
        return ESP_FAIL;
    }

    ESP_LOGI("mmWave", "Baud rate set to %u successfully", baud_rate);
    return ESP_OK;
}
esp_err_t radar_sensor_set_multi_target_mode(radar_sensor_t *sensor, bool enable) {
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t cmd[12];
    uint8_t ack[14];
    size_t cmd_len = sizeof(cmd);
    size_t ack_len = sizeof(ack);

    if (enable) {
        // Multi-target mode command
        memcpy(cmd, Multi_Target_CMD, sizeof(Multi_Target_CMD));
        sensor->multiTargetMode = true;
    } else {
        // Single-target mode command
        memcpy(cmd, Single_Target_CMD, sizeof(Single_Target_CMD));
        sensor->multiTargetMode = false;
    }

    bool was_in_config_mode = sensor->configMode;
    if (!sensor->configMode)
    {
        ESP_ERROR_CHECK(radar_sensor_set_config_mode(sensor, true));
    }

    esp_err_t ret = Send_Command(sensor, cmd, cmd_len, ack, ack_len);
    if (ret != ESP_OK) {
        ESP_LOGE("mmWave", "Failed to set %s-target mode", enable ? "multi" : "single");
        return ret;
    }

    if (!was_in_config_mode)
    {
        ESP_ERROR_CHECK(radar_sensor_set_config_mode(sensor, false));
    }

    // 9,8 = Status 0x00, 0x00
    if (ack[8] != 0x00 || ack[9] != 0x00)
    {
        ESP_LOGE("mmWave", "Error in ACK while setting target mode: %d,%d", ack[8], ack[9]);
        return ESP_FAIL;
    }

    ESP_LOGI("mmWave", "%s-target mode set successfully", enable ? "Multi" : "Single");
    return ESP_OK;
}