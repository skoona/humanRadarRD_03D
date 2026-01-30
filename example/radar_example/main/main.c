#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "humanRadarRD_03D.h"

void app_main()
{
    radar_sensor_t radar;
    radar_target_t targets[RADAR_MAX_TARGETS];
    radar_target_t priorTargets[RADAR_MAX_TARGETS];
    char versionString[64] = {0};

    // Initialize the radar sensor
    esp_err_t ret = radar_sensor_init(&radar, CONFIG_UART_PORT, CONFIG_UART_RX_GPIO, CONFIG_UART_TX_GPIO);
    if (ret != ESP_OK)
    {
        ESP_LOGE("MAIN", "Failed to initialize radar sensor");
        return;
    }

    // Start UART communication -- defaults to single target mode
    ret = radar_sensor_begin(&radar, CONFIG_UART_SPEED_BPS);
    if (ret != ESP_OK)
    {
        ESP_LOGE("MAIN", "Failed to start radar sensor");
        return;
    }

    // Configure radar
    // ESP_ERROR_CHECK(radar_sensor_set_config_mode(&radar, true));
    if (radar_sensor_set_config_mode(&radar, true) == ESP_OK) {
        radar_sensor_set_retention_times(&radar, 10000, 500);
        radar_sensor_get_firmware_version(&radar, versionString);
        ESP_LOGI("Radar", "Radar Firmware Version: %s", versionString);
        radar_sensor_set_config_mode(&radar, false);
    }

    ESP_LOGI("Radar", "Starting main loop.");

    // Main loop
    int target_count = 0;
    bool hasMoved = false;
    while (1)
    {
        if (radar_sensor_update(&radar))
        {
            // Save prior state
            memcpy(priorTargets, targets, sizeof(targets));
            // Save current state
            target_count = radar_sensor_get_targets(&radar, targets);
            for (int idx = 0; idx < RADAR_MAX_TARGETS; idx++)
            {
                hasMoved = radar_sensor_hasTargetMoved(targets, priorTargets, idx);
                if (hasMoved)
                {
                    ESP_LOGI("RD-03D", "[%d]:[%d] Target detected at (%.1f, %.1f) mm, distance: %.1f mm", idx, target_count, targets[idx].x, targets[idx].y, targets[idx].distance);
                    ESP_LOGI("RD-03D", "[%d]:[%d] Position: %s", idx, target_count, targets[idx].position_description);
                    ESP_LOGI("RD-03D", "[%d]:[%d] Angle: %.1f degrees, Distance: %.1f mm, Speed: %.1f mm/s", idx, target_count, targets[idx].angle, targets[idx].distance, targets[idx].speed);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(120));
    }
}