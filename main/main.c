#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "dht11.h"
#include "driver/adc.h"
#include "cJSON.h"
#include "freertos/semphr.h" // Include semaphore header

#define WATER_RELAY_GPIO GPIO_NUM_15
#define HEATER_RELAY_GPIO GPIO_NUM_6
#define FAN_RELAY_GPIO GPIO_NUM_7
#define MOISTURE_SENSOR_PIN ADC1_CHANNEL_4
#define DHT_TASK_DELAY_MS 2000
SemaphoreHandle_t printMutex; // Declare semaphore

// Function to map the sensor reading to a moisture range
// Function to map the sensor reading to a moisture range
int mapMoisture(float value, float fromLow, float fromHigh, int toLow, int toHigh) {
    return (int)(((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow) + toLow);
}

void sendSensorData(const char *sensorName, float value) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "sensor", sensorName);
    cJSON_AddNumberToObject(root, "value", value);

    char *jsonString = cJSON_Print(root);
    xSemaphoreTake(printMutex, portMAX_DELAY); // Take semaphore before printing
    printf("%s\n", jsonString);
    fflush(stdout); // Flush the standard output
    xSemaphoreGive(printMutex); // Give semaphore after printing

    cJSON_Delete(root);
    free(jsonString);
}
void readMoistureTask(void *pvParameter) {
    adc1_config_width(ADC_WIDTH_BIT_12); // ADC width configuration
    int lastMoistureLevel = -1; // Initialize with a value that won't match any valid level

    while (1) {
        int sensorValue = adc1_get_raw(MOISTURE_SENSOR_PIN); // Read sensor value

        // Check if the sensor value is within a valid range
        if (sensorValue < 0 || sensorValue > 4095) {
            xSemaphoreTake(printMutex, portMAX_DELAY);
            printf("Invalid moisture sensor reading: %d\n", sensorValue);
            fflush(stdout); // Flush the standard output
            xSemaphoreGive(printMutex);
            vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second before retrying
            continue;
        }

        float voltage = sensorValue * (3.3 / 4095.0); // Convert sensor value to voltage (assuming 3.3V)

        // Map the voltage to a moisture range (adjust these values according to your sensor)
        int moistureLevel = mapMoisture(voltage, 0.8, 3.0, 0, 100); // Example mapping for moisture range between 0.8V and 3.0V

        xSemaphoreTake(printMutex, portMAX_DELAY);
        
        // Only print if the current moisture level is different from the last one
        if (moistureLevel != lastMoistureLevel) {
            printf("Moisture Level: %d%%\n", moistureLevel);
            fflush(stdout); // Flush the standard output
            lastMoistureLevel = moistureLevel;
        }

        xSemaphoreGive(printMutex);

        sendSensorData("moisture", moistureLevel);

        // Control relays based on moisture level
        xSemaphoreTake(printMutex, portMAX_DELAY);
        if (moistureLevel < 30) {
            gpio_set_level(WATER_RELAY_GPIO, 1); // Turn on water relay
            printf("Turn on water relay\n");
        } else {
            gpio_set_level(WATER_RELAY_GPIO, 0); // Turn off water relay
            printf("Turn off water relay\n");
        }
        fflush(stdout); // Flush the standard output
        xSemaphoreGive(printMutex);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
    }
}

void dht11_task(void *pvParameter) {
    while (1) {
        int ret = readDHT11(); // Read data from DHT11 sensor
        if (ret == DHT_OK) {
            float temperature = getTemperature_dht11();

            xSemaphoreTake(printMutex, portMAX_DELAY);
            printf("Temperature: %.1f°C\n", temperature);
            fflush(stdout); // Flush the standard output
            xSemaphoreGive(printMutex);

            sendSensorData("temperature", temperature);

            // Control relays based on temperature thresholds
            xSemaphoreTake(printMutex, portMAX_DELAY);
            if (temperature < 20) {
                gpio_set_level(HEATER_RELAY_GPIO, 1); // Turn on heater relay
                printf("Turn on heater\n ");
            } else {
                gpio_set_level(HEATER_RELAY_GPIO, 0); // Turn off heater relay
                printf("Turn off heater\n");
            }

            if (temperature <= 10) {
                gpio_set_level(FAN_RELAY_GPIO, 1); // Turn off fan relay
                printf("Turn off fan\n");
            } else if (temperature > 22) {
                gpio_set_level(FAN_RELAY_GPIO, 0); // Turn on fan relay
                printf("Turning on fan\n");
            }
            fflush(stdout); // Flush the standard output
            xSemaphoreGive(printMutex);
        } else {
            xSemaphoreTake(printMutex, portMAX_DELAY);
            printf("Failed to read temperature data\n");
            fflush(stdout); // Flush the standard output
            xSemaphoreGive(printMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(DHT_TASK_DELAY_MS)); // Delay between sensor readings
    }
}
void app_main() {
    // Create semaphore for console print synchronization
    printMutex = xSemaphoreCreateMutex();

    // Create the moisture reading task
    xTaskCreate(&readMoistureTask, "readMoistureTask", 2048, NULL, 5, NULL);

    // Create the DHT11 task
    xTaskCreate(&dht11_task, "dht11_task", 2048, NULL, 4, NULL);

    // Initialize GPIOs for relays if not done already

    // Initialize DHT11 GPIO
    setDHT11gpio(GPIO_NUM_2);
}