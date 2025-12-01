#ifndef _DHT_11
#define _DHT_11

#include <driver/gpio.h>
#include <stdio.h>
#include <string.h>
#include <rom/ets_sys.h>
#include "esp_log.h"

/**
 * @brief DHT11 data structure.
 * @var dht11_pin The GPIO pin connected to the DHT11 DATA/SIGNAL line.
 * @var temperature Last temperature reading (°C, float).
 * @var humidity Last humidity reading (%RH, float).
 */
typedef struct
{
    gpio_num_t dht11_pin;
    float temperature;
    float humidity;
} dht11_t;

/**
 * @brief Waits on pin until it reaches the specified state.
 * @param dht11 Struct containing pin info.
 * @param state GPIO level to wait for (0 or 1).
 * @param timeout_us Timeout, in microseconds.
 * @return Time waited (us), or -1 if timed out.
 */
int wait_for_state(dht11_t dht11, int state, int timeout_us);

/**
 * @brief Pulls pin low for specified microseconds to start DHT11 protocol.
 * @param dht11 Struct containing pin info.
 * @param hold_time_us Time to hold pin LOW (microseconds).
 */
void hold_low(dht11_t dht11, int hold_time_us);

/**
 * @brief Reads temperature and humidity from DHT11 sensor.
 * @param dht11 Pointer to dht11_t struct containing pin info.
 * @param connection_timeout Number of handshake attempts before giving up.
 * @return 0 if successful, -1 if failed (logs error).
 */
int dht11_read(dht11_t *dht11, int connection_timeout);

#endif
