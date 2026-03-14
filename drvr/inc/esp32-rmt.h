/**
 * @file rmt_driver.h
 * @date 20260312
 * @author mhoyer
 * @copyright mhoyer 2026
 *
 * @brief
 * ESP32 RMT driver interface
 */
#pragma once

#include <stdint.h>
#include <zephyr/device.h>

typedef enum {
    CHANNEL_0=0,
    CHANNEL_1=1,
    CHANNEL_2=2,
    CHANNEL_3=3,
    CHANNEL_4=4,
    CHANNEL_5=5,
    CHANNEL_6=6,
    CHANNEL_7=7
} rmt_channel_num_t;

typedef enum  {
    RMT_CHANNEL_EVENT_DATA_RECEIVED =   0,
    RMT_CHANNEL_EVENT_ERROR         =   1
} rmt_channel_event_t;

struct esp32_rmt_channel;

/**
 * Notes:
 * -> Parameters in DT:
 *  - RMT channel number
 *  - GPIO pin for RMT input/output
 *  - RMT clock source and divider
 *  - RMT memory block size
 *  - timeout for reception (cycles)
 *  - minimum pulse duration for reception (cycles)
 *  - maximum pulse duration for reception (cycles)
 *  - enable / disable filtering
 * 
 * -> In Functions / Driver:
 *  - callback for RMT events (reception complete)
 *  - expected datagram len for reception (supports datagram as well as single pulse mode)
 *  - transmit function with pulse timing parameters (duration, level, datagram length)
 *  - enable / disable RMT unit
 * 
 */
typedef void (*rmt_channel_callback_t)(struct esp32_rmt_channel *channel, 
    rmt_channel_event_t event, void *user_data);

struct esp32_rmt_channel *rmt_get_channel(const struct device *dev,
    rmt_channel_num_t channel_num);

void *rmt_get_channel_databuffer(struct esp32_rmt_channel *channel);

uint8_t rmt_get_channel_num(struct esp32_rmt_channel *channel);

void rmt_register_channel_callback(struct esp32_rmt_channel *channel, 
    rmt_channel_callback_t callback, void *user_data);

void rmt_start_reception(struct esp32_rmt_channel *channel);

void rmt_stop_reception(struct esp32_rmt_channel *channel);

void rmt_flush_received_data(struct esp32_rmt_channel *channel);
