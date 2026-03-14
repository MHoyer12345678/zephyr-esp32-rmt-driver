#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "nec-ir-decoder.h"

static inline bool is_rmt_sample_withing_range(uint16_t sample, uint16_t min_cycles, uint16_t max_cycles)
{
    sample = sample & 0x7FFF; // Mask out the level bit to get the cycle count
    return sample >= min_cycles && sample <= max_cycles;
}

static inline bool extract_value_and_check_crc(const uint16_t raw, uint16_t *val_p, bool do_crc)
{
    if (do_crc) {
        uint8_t val = raw >> 8;
        uint8_t val_inv = raw & 0xFF;
        // Verify address checksum
        *val_p=val;
        return ((val ^ val_inv) == 0xFF);
    } else {
        *val_p = raw;
        return true;
    }
}

static inline bool extract_address_and_command_from_nec_datagram(uint32_t nec_datagram, 
    const ir_decoder_config_t *config, decoded_data_t *decoded_data)
{
    // NEC datagram format: address (8 bits) + ~address (8 bits) + command (8 bits) + ~command (8 bits)
    // depending on the config flags, the address and command can be either 8 or 16 bits, with or without checksum.

    //extract & check command
    if (!extract_value_and_check_crc(nec_datagram & 0xFFFF, &decoded_data->command, 
            config->decoder_flags & NEC_CMD_USE_CMD_CRC))
    {
        decoded_data->result=CRC_COMMAND_FAILED;
        return false;
    }
 
    //extract & check address
    nec_datagram >>= 16;
    if (!extract_value_and_check_crc(nec_datagram & 0xFFFF, &decoded_data->addr, 
            config->decoder_flags & NEC_ADDR_USE_ADDR_CRC))
    {
        decoded_data->result=CRC_ADDRESS_FAILED;
        return false;
    }

    decoded_data->result=DECODER_SUCCESS;
    return true;
}

bool nec_ir_decode(void *buffer, const ir_decoder_config_t *config, decoded_data_t *decoded_data)
{
    uint32_t nec_datagram=0;
    uint8_t cntr=0;
    uint16_t *raw_data=(uint16_t *)buffer;

    decoded_data->raw_data=0;

    // start with 9ms leading pulse, followed by 4.5ms space, 
    // then 2.25ms pulse for bit 1 and 1.125ms pulse for bit 0.
    if (is_rmt_sample_withing_range((raw_data[0] & 0x7FFF), config->min_leading_pulse_cycles,
            config->max_leading_pulse_cycles))
    {
        if (is_rmt_sample_withing_range((raw_data[1] & 0x7FFF), config->min_repeat_space_cycles,
                config->max_repeat_space_cycles)) 
        {   
            decoded_data->is_repeat = true;
            return true;
        }
        else if (!is_rmt_sample_withing_range((raw_data[1] & 0x7FFF), config->min_start_space_cycles, 
                    config->max_start_space_cycles)) 
        {   
            decoded_data->result=INVALID_START_PULSE_LEN;
            decoded_data->raw_data=raw_data[1] & 0x7FFF;
            return false; // Not a valid NEC start pulse, ignore
        }
    }
    else
    {
        decoded_data->result=INVALID_START_PULSE_LEN;
        decoded_data->raw_data=raw_data[0] & 0x7FFF;
        return false; // Not a valid NEC start pulse, ignore
    }
     
    while (cntr<64) // NEC datagram is 32 bits, each bit is represented by a pair of pulses (low + high), so we need to read 64 pulses in total 
    {
         // Get the total period of the bit (low + high)
        uint32_t cycles_period=(uint32_t)((raw_data[2+cntr] & 0x7FFF) + (raw_data[2+cntr+1] & 0x7FFF));
        nec_datagram <<= 1; // Shift the datagram to make room for the next bit

        if (is_rmt_sample_withing_range(cycles_period, config->min_databit_one_cycles, 
                config->max_databit_one_cycles)) 
            nec_datagram |= 1; // Bit is 1
        else if (is_rmt_sample_withing_range(cycles_period, config->min_databit_zero_cycles, 
                config->max_databit_zero_cycles)) 
            nec_datagram |= 0; // Bit is 0
        else 
        {
            decoded_data->result=INVALID_BIT_PULSE_LEN;
            decoded_data->raw_data=cycles_period;
            return false; // Invalid pulse width, ignore the datagram
        }
        cntr+=2;
    }

    decoded_data->is_repeat=false;
    decoded_data->raw_data=nec_datagram;

    return extract_address_and_command_from_nec_datagram(nec_datagram, config, decoded_data);
}
