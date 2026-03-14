/**
 * @file rmt_driver.c
 * @date 20260312
 * @author mhoyer
 * @copyright mhoyer 2026
 *
 * @brief
 * ESP32 RMT driver implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

#include "esp_private/periph_ctrl.h"
#include "soc/rmt_periph.h"
#include "soc/rmt_struct.h"
#include "hal/rmt_ll.h"
#include "soc/soc.h"
#include "soc/rmt_reg.h"
#include "esp_intr_alloc.h"

#include "esp32-rmt.h"

#define DT_DRV_COMPAT espressif_esp32_rmt

LOG_MODULE_REGISTER(esp32_rmt, LOG_LEVEL_ERR);

#define DT_RMT_MODULE_NODE_ID DT_INST(0, espressif_esp32_rmt)

#define RMT_NO_FILTERING        0

#define RMT_CHANNEL_MODE_RECV   0
#define RMT_CHANNEL_MODE_TRANS  1

#define RMT_CLOCK_APB           0
#define RMT_CLOCK_REF           1

typedef enum {
    RMT_CARRIER_SIGNAL_NONE =       0,
    RMT_CARRIER_SIGNAL_ON_HIGH =    1,
    RMT_CARRIER_SIGNAL_ON_LOW =     2
} tx_carrier_signal_t;

struct esp32_rmt_channel_config {
    /**
     * channel number
     */
	const uint8_t channel_num;

    /**
     * RMT_CHANNEL_MODE_RECV(0) or RMT_CHANNEL_MODE_TRANS(1)
     **/
    bool channel_mode;  

    /** 
     * Number of memory blocks to allocate for this channel.
     **/
    uint8_t mem_blocks;
 
    /**
     *  Clock source for the channel: RMT_CLOCK_APB(0) or RMT_CLOCK_REF(1) 
     **/
    bool clock_source;

    /** 
     * Clock divider for the channel, used to derive the RMT clock from the source clock. 
     **/
    uint8_t clock_divider;
    
    /** 
     * Idle threshold for reception, in RMT clock cycles. Used to detect the end of a datagram. 
     **/
    uint16_t rx_idle_threshold;
    
    /** 
     * Filter threshold for reception, in RMT clock cycles. Used to filter out noise and
     * short pulses. Set to RMT_NO_FILTERING (0) to disable filtering.
     **/
    uint8_t rx_filter_threshold;

    /** 
     * Transmission limit threshold, in number of symbols.
     * An interrupt event would be generated after transmitting this many symbols.
     **/
    uint8_t tx_limit;
    
    /** 
     * Whether to enable output on idle for transmission.
     * If true, the RMT output will be enabled during idle periods between symbols.
     **/   
    bool tx_output_enable_on_idle;

    /** 
     * Output level during idle periods for transmission, if output on idle is enabled.
     * If true, the RMT output will be high during idle periods; if false, it will be low.
     **/
    bool tx_output_level_on_idle;

    /** 
     * Whether to enable carrier modulation for transmission.
     * If true, the RMT will modulate the output signal with a carrier frequency defined by tx_carrier_high_period and tx_carrier_low_period.
     **/
    tx_carrier_signal_t tx_carrier_signal;
    
    /** 
     * High period of the carrier signal for transmission, in RMT clock cycles, if carrier modulation is enabled.
     **/
    uint16_t tx_carrier_high_period;
    
    /** 
     * Low period of the carrier signal for transmission, in RMT clock cycles, if carrier modulation is enabled.
     **/
    uint16_t tx_carrier_low_period;
    
};

struct esp32_rmt_config {
	const struct pinctrl_dev_config *pincfg;
    const uint32_t rmt_base_addr;
	const struct esp32_rmt_channel_config *channel_config;
	const int channel_len;
};


struct esp32_rmt_channel {
    // Add any channel-specific runtime data here, such as reception buffers, transmission state, etc.
    const struct esp32_rmt_channel_config *config;

    // Callback function for RMT events (e.g., reception complete, transmission complete, error, etc.)
    rmt_channel_callback_t callback;
    // User data pointer to be passed to the callback function
    void *user_data;

    //interrupt mask relevant for this channel
    uint32_t intr_mask;
};

#define CHANNEL_CONFIG(node_id)                                                            \
	{                                                                                      \
		.channel_num = DT_REG_ADDR(node_id),                                               \
        .channel_mode = DT_ENUM_IDX(node_id, mode),                                        \
        .mem_blocks = DT_PROP(node_id, memblocks),                                         \
        .clock_source = DT_ENUM_IDX(node_id, clk_source),                                  \
        .clock_divider = DT_PROP(node_id, clk_divider),                                    \
        .rx_idle_threshold = DT_PROP(node_id, rx_idle_threshold),                          \
        .rx_filter_threshold = DT_PROP(node_id, rx_filter_threshold),                      \
        .tx_limit = DT_PROP(node_id, tx_limit),                                            \
        .tx_output_enable_on_idle = DT_PROP(node_id, tx_output_enable_on_idle),            \
        .tx_output_level_on_idle = DT_PROP(node_id, tx_output_level_on_idle),              \
        .tx_carrier_signal = DT_ENUM_IDX(node_id, tx_carrier_signal),                      \
        .tx_carrier_high_period = DT_PROP(node_id, tx_carrier_high_period),                \
        .tx_carrier_low_period = DT_PROP(node_id, tx_carrier_low_period),                  \
	},

static struct esp32_rmt_channel_config channel_config[] = {
	DT_INST_FOREACH_CHILD(0, CHANNEL_CONFIG)};

PINCTRL_DT_INST_DEFINE(0);

static const struct esp32_rmt_config esp32_rmt_config = {
    	.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
        .rmt_base_addr = DT_REG_ADDR_RAW(DT_RMT_MODULE_NODE_ID),
	    .channel_config=channel_config,
	    .channel_len = ARRAY_SIZE(channel_config),
};

struct esp32_rmt_data {
    rmt_dev_t *rmt_dev;
    struct esp32_rmt_channel channel[ARRAY_SIZE(channel_config)]; 
};

static struct esp32_rmt_data esp32_rmt_data = {
    .rmt_dev = NULL,
};

#warning: implement thread safe access 

static intr_handle_t irg_handle=NULL; // Interrupt handle for RMT ISR

static inline void esp32_rmt_handle_recv_isr(struct esp32_rmt_channel *ch, uint32_t intr_status)
{

    if (intr_status & RMT_LL_EVENT_RX_DONE(ch->config->channel_num))
        ch->callback(ch, RMT_CHANNEL_EVENT_DATA_RECEIVED,ch->user_data);
    if (intr_status & RMT_LL_EVENT_RX_ERROR(ch->config->channel_num))
        ch->callback(ch, RMT_CHANNEL_EVENT_ERROR,ch->user_data);
}

static inline void esp32_rmt_handle_transv_isr(struct esp32_rmt_channel *ch, uint32_t intr_status)
{
    LOG_ERR("Transceiver functionality not yet implemented.");
    assert(false);   
}

void esp32_rmt_isr(void *arg)
{
    assert(esp32_rmt_data.rmt_dev != NULL);
    volatile uint32_t *intr_status_ptr=rmt_ll_get_interrupt_status_reg(esp32_rmt_data.rmt_dev);


    for (int i = 0; i < esp32_rmt_config.channel_len; i++) 
    {
        if ((esp32_rmt_data.channel[i].intr_mask & *intr_status_ptr) != 0 &&
            esp32_rmt_data.channel[i].callback != NULL)
        {
            // found channel with fired interrupts ...
            struct esp32_rmt_channel *ch=&esp32_rmt_data.channel[i];
            if (ch->config->channel_mode==RMT_CHANNEL_MODE_RECV)
                esp32_rmt_handle_recv_isr(ch, *intr_status_ptr);
            else
                esp32_rmt_handle_transv_isr(ch, *intr_status_ptr);
        }
    }

    rmt_ll_clear_interrupt_status(esp32_rmt_data.rmt_dev, *intr_status_ptr);
}

struct esp32_rmt_channel *rmt_get_channel(const struct device *dev,
    rmt_channel_num_t channel_num) 
{
    assert(esp32_rmt_data.rmt_dev != NULL); // Ensure that the RMT device has been initialized
    
    const struct esp32_rmt_config *config = dev->config;

    for (int i = 0; i < config->channel_len; i++) {
        if (config->channel_config[i].channel_num == channel_num) {
            return &esp32_rmt_data.channel[i];
        }
    }

    LOG_ERR("Channel %d not found in device configuration", channel_num);
    return NULL; // Channel not found
}

uint8_t rmt_get_channel_num(struct esp32_rmt_channel *channel)
{
    return channel->config->channel_num;
}


void *rmt_get_channel_databuffer(struct esp32_rmt_channel *channel)
{
    assert(esp32_rmt_data.rmt_dev != NULL); // Ensure that the RMT device has been initialized
    uint32_t addr=(uint32_t)esp32_rmt_data.rmt_dev + 0x800 + (channel->config->channel_num * 0x100);
    return (void *)(addr);
}

void rmt_start_reception(struct esp32_rmt_channel *channel)
{
    assert(esp32_rmt_data.rmt_dev != NULL); // Ensure that the RMT device has been initialized
    LOG_DBG("Starting RMT reception");
    rmt_ll_rx_enable(esp32_rmt_data.rmt_dev, channel->config->channel_num, true);
}

void rmt_stop_reception(struct esp32_rmt_channel *channel)
{
    assert(esp32_rmt_data.rmt_dev != NULL); // Ensure that the RMT device has been initialized
    LOG_DBG("Stopping RMT reception");
    rmt_ll_rx_enable(esp32_rmt_data.rmt_dev, channel->config->channel_num, false); 
}

void rmt_register_channel_callback(struct esp32_rmt_channel *channel, 
    rmt_channel_callback_t callback, void *user_data)
{
    channel->callback = callback;
    channel->user_data = user_data;
}

void rmt_flush_received_data(struct esp32_rmt_channel *channel)
{
    assert(esp32_rmt_data.rmt_dev != NULL); // Ensure that the RMT device has been initialized
    // Flush the received data buffer by resetting the memory pointer and setting memory owner to hardware for reception
    rmt_ll_rx_reset_pointer(esp32_rmt_data.rmt_dev, channel->config->channel_num);
    rmt_ll_rx_set_mem_owner(esp32_rmt_data.rmt_dev, channel->config->channel_num, RMT_LL_MEM_OWNER_HW);
}

// config the channel with reception specific parameters 
int esp32_rmt_config_channel_receiver_parameters(const struct esp32_rmt_channel_config *ch_config,
    struct esp32_rmt_channel *channel)
{
    channel->intr_mask = RMT_LL_EVENT_RX_DONE(ch_config->channel_num) | RMT_LL_EVENT_RX_ERROR(ch_config->channel_num);

    // receiver mode -> memory owner is hardware
    rmt_ll_rx_set_mem_owner(esp32_rmt_data.rmt_dev, ch_config->channel_num, RMT_LL_MEM_OWNER_HW);

    rmt_ll_rx_set_mem_blocks(esp32_rmt_data.rmt_dev, ch_config->channel_num, ch_config->mem_blocks);
    rmt_ll_rx_set_channel_clock_div(esp32_rmt_data.rmt_dev, ch_config->channel_num, ch_config->clock_divider);
    rmt_ll_rx_set_idle_thres(esp32_rmt_data.rmt_dev, ch_config->channel_num, ch_config->rx_idle_threshold);
    rmt_ll_rx_enable_filter(esp32_rmt_data.rmt_dev, ch_config->channel_num, ch_config->rx_filter_threshold != RMT_NO_FILTERING);
    rmt_ll_rx_set_filter_thres(esp32_rmt_data.rmt_dev, ch_config->channel_num, ch_config->rx_filter_threshold); 

    rmt_ll_enable_interrupt(esp32_rmt_data.rmt_dev, channel->intr_mask, true);

    return 0;
}

int esp32_rmt_config_channel_transmitter_parameters(const struct esp32_rmt_channel_config *ch_config, 
    struct esp32_rmt_channel *channel)
{
    // Configure the channel for transmission
/*    rmt_ll_tx_enable_loop(rmt_dev, ch_config->channel_num, true); // Disable loop mode for transmission 
    rmt_ll_enable_interrupt(rmt_dev, RMT_LL_EVENT_TX_DONE(ch_config->channel_num) | RMT_LL_EVENT_TX_ERROR(ch_config->channel_num) | 
        RMT_LL_EVENT_TX_LOOP_END(ch_config->channel_num) | RMT_LL_EVENT_TX_THRES(ch_config->channel_num), true); // Enable relevant TX interrupts for this channel
    rmt_ll_tx_set_mem_blocks(rmt_dev, ch_config->channel_num, 4); // Set memory blocks for transmission
    rmt_ll_tx_set_limit(rmt_dev, ch_config->channel_num, 4); // Set limit threshold for transmission
    rmt_ll_tx_enable_carrier_modulation(rmt_dev, ch_config->channel_num, true); // Enable carrier modulation for transmission
    rmt_ll_tx_fix_idle_level(rmt_dev, ch_config->channel_num, 1, true); // Set idle level for transmission
    rmt_ll_tx_enable_wrap(rmt_dev, ch_config->channel_num, true); // Enable wrap mode for transmission
*/
    // Transmission configuration is not implemented yet, as the current focus is on reception.
    // This function can be implemented in the future when transmission functionality is added.
    assert(false); 
    
    return 0;
}

static void rmt_log_dump_config(const struct esp32_rmt_config *config)
{
    LOG_DBG("RMT Configuration:");
    LOG_DBG("  RMT device: %p", (void*)config->rmt_base_addr);
    LOG_DBG("  Number of channels: %d", config->channel_len);
}

static void rmt_log_dump_ch_config(const struct esp32_rmt_channel_config *ch_config)
{
    LOG_DBG("Channel %d Configuration:", ch_config->channel_num);
    LOG_DBG("  Mode: %s", ch_config->channel_mode == RMT_CHANNEL_MODE_RECV ? "RX" : "TX");
    LOG_DBG("  Memory blocks: %d", ch_config->mem_blocks);
    LOG_DBG("  Clock source: %s", ch_config->clock_source == RMT_CLOCK_APB ? "APB" : "REF");
    LOG_DBG("  Clock divider: %d", ch_config->clock_divider);
    LOG_DBG("  RX idle threshold: %u", ch_config->rx_idle_threshold);
    LOG_DBG("  RX filter threshold: %d", ch_config->rx_filter_threshold);
    LOG_DBG("  TX limit: %d", ch_config->tx_limit);
    LOG_DBG("  TX output on idle: %s", ch_config->tx_output_enable_on_idle ? "enabled" : "disabled");
    LOG_DBG("  TX idle level: %s", ch_config->tx_output_level_on_idle ? "high" : "low");
    LOG_DBG("  TX carrier signal: %d", ch_config->tx_carrier_signal);
    LOG_DBG("  TX carrier high: %u, low: %u", ch_config->tx_carrier_high_period, ch_config->tx_carrier_low_period);
}

int esp32_rmt_init(const struct device *dev)
{
    const struct esp32_rmt_config *config = dev->config;
    int ret;

    // Initialize the RMT peripheral with the specified configuration parameters
    rmt_log_dump_config(config);

    // Initialize the RMT device pointer from the configuration
    esp32_rmt_data.rmt_dev = (rmt_dev_t *)config->rmt_base_addr; 

    // enable RMT peripheral module
    periph_module_enable(PERIPH_RMT_MODULE);
    // Reset RMT peripheral module
    periph_module_reset(PERIPH_RMT_MODULE);
    // Enable RMT peripheral clock
    rmt_ll_enable_periph_clock(esp32_rmt_data.rmt_dev, true); 
    // do not use fifo via abd access. Instead, access rmt memory directly, 
    // which is more efficient and faster. Only used for reception, for transmission, fifo mode is still used. 
    rmt_ll_enable_mem_access_nonfifo(esp32_rmt_data.rmt_dev, true); 

    // Connect the RMT interrupt to the ISR and enable it
    ret=esp_intr_alloc(ETS_RMT_INTR_SOURCE, 0, esp32_rmt_isr, NULL, &irg_handle);
    if (ret != ESP_OK) {
        LOG_ERR("Failed to allocate interrupt for RMT: %d", ret);
        return ret;
    }


    // Initialize all channels based on the channel configuration provided in the device config
    for (int i = 0; i < config->channel_len; i++) {
        const struct esp32_rmt_channel_config *ch_config = &config->channel_config[i];

        //initialize per channel data 
        esp32_rmt_data.channel[i].config = ch_config;
        esp32_rmt_data.channel[i].callback = NULL; // Initialize callback to NULL
        esp32_rmt_data.channel[i].user_data = NULL; // Initialize user data to NULL

        rmt_log_dump_ch_config(ch_config);

        //set channel mode independent parameters
        if (ch_config->clock_source == RMT_CLOCK_APB) {
            rmt_ll_set_group_clock_src(esp32_rmt_data.rmt_dev, ch_config->channel_num, RMT_CLK_SRC_APB, 1, 1, 0);
        } else if (ch_config->clock_source == RMT_CLOCK_REF) {
            rmt_ll_set_group_clock_src(esp32_rmt_data.rmt_dev, ch_config->channel_num, RMT_CLK_SRC_REF_TICK, 1, 1, 0);
        } else {
            LOG_ERR("Invalid clock source for channel %d", ch_config->channel_num);
            return -EINVAL;
        }

        // set channel mode specific parameters
        if (ch_config->channel_mode == RMT_CHANNEL_MODE_RECV) 
            ret=esp32_rmt_config_channel_receiver_parameters(ch_config, &esp32_rmt_data.channel[i]);
         else
            ret=esp32_rmt_config_channel_transmitter_parameters(ch_config, &esp32_rmt_data.channel[i]);
    }
 
    /* Select "default" state at initialization time */
    ret = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
    if (ret < 0) {
        LOG_ERR("Failed to apply pinctrl state: %d", ret);
        return ret;
    }
    LOG_DBG("Pinctrl state applied successfully");
    return 0;
}

DEVICE_DT_INST_DEFINE(0, &esp32_rmt_init, NULL, NULL,
    &esp32_rmt_config, POST_KERNEL,99, NULL);
