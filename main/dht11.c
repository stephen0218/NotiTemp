// dht11.c
#include "dht11.h"
#include "driver/gpio.h"
#include "rom/ets_sys.h"
#include "esp_log.h"

int wait_for_state(dht11_t dht11, int state, int timeout_us)
{
    gpio_set_direction(dht11.dht11_pin, GPIO_MODE_INPUT);
    int count = 0;
    while (gpio_get_level(dht11.dht11_pin) != state) {
        if (count >= timeout_us) return -1;
        ets_delay_us(1);
        count++;
    }
    return count;
}

void hold_low(dht11_t dht11, int hold_time_us)
{
    gpio_set_direction(dht11.dht11_pin, GPIO_MODE_OUTPUT_OD); // Open-drain mode
    gpio_set_level(dht11.dht11_pin, 0);
    ets_delay_us(hold_time_us);
    gpio_set_level(dht11.dht11_pin, 1);
    ets_delay_us(30); // Wait for sensor to pull line low
    gpio_set_direction(dht11.dht11_pin, GPIO_MODE_INPUT);
}

int dht11_read(dht11_t *dht11, int connection_timeout)
{
    int waited = 0;
    uint8_t received_data[5] = {0, 0, 0, 0, 0};
    int timeout_counter = 0;

    while (timeout_counter < connection_timeout)
    {
        timeout_counter++;
        hold_low(*dht11, 18000); // Host start signal: pull low for 18ms
        
        // Wait for the sensor response (low)
        waited = wait_for_state(*dht11, 0, 80);
        if (waited == -1) {
            ESP_LOGE("DHT11", "Failed at phase 1 (no response)");
            ets_delay_us(20000); continue;
        }
        // Wait for sensor pulls high
        waited = wait_for_state(*dht11, 1, 80);
        if (waited == -1) {
            ESP_LOGE("DHT11", "Failed at phase 2 (no handshake)");
            ets_delay_us(20000); continue;
        }
        // Wait for sensor pulls low again
        waited = wait_for_state(*dht11, 0, 80);
        if (waited == -1) {
            ESP_LOGE("DHT11", "Failed at phase 3");
            ets_delay_us(20000); continue;
        }
        break;
    }
    if (timeout_counter == connection_timeout) return -1;

    // Read 5 bytes (40 bits)
    for (int i = 0; i < 5; i++) {
        for (int j = 7; j >= 0; j--) {
            // Wait for LOW
            if (wait_for_state(*dht11, 1, 55) == -1) return -1;
            // Measure HIGH time to distinguish 0/1
            int t = wait_for_state(*dht11, 0, 75);
            if (t == -1) return -1;
            received_data[i] |= ((t > 35) ? 1 : 0) << j;
        }
    }

    // Checksum
    int sum = (received_data[0] + received_data[1] + received_data[2] + received_data[3]) & 0xFF;
    if (sum != received_data[4]) {
        ESP_LOGE("DHT11", "Wrong checksum (got %d expected %d)", received_data[4], sum);
        return -1;
    }

    dht11->humidity = received_data[0] + received_data[1] * 0.1;
    dht11->temperature = received_data[2] + received_data[3] * 0.1;
    return 0;
}
