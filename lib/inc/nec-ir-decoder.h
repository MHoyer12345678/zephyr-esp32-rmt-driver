#ifndef SRC_DECODERS_NEC_IR_DECODER_H
#define SRC_DECODERS_NEC_IR_DECODER_H

#include <stdint.h>
#include <stdbool.h>

//if set, address is an 8 bit value followed by its inverse,
//otherwise address is a 16 bit value without checksum
#define NEC_ADDR_USE_ADDR_CRC       0x1
//if set, command is an 8 bit value followed by its inverse,
//otherwise command is a 16 bit value without checksum
#define NEC_CMD_USE_CMD_CRC         0x2

typedef struct ir_decoder_config_t {
    uint8_t decoder_flags; // Flags to indicate the datagram format (e.g., NEC_ADDR_USE_ADDR_CRC, NEC_CMD_USE_CMD_CRC)
    uint16_t min_leading_pulse_cycles; // Minimum cycles for the leading pulse (e.g., 8700 for 9ms at 1.1us per cycle)
    uint16_t max_leading_pulse_cycles; // Maximum cycles for the leading pulse ( e.g., 9300 for 9ms at 1.1us per cycle)
    uint16_t min_start_space_cycles; // Minimum cycles for the space after leading pulse in normal datagrams (e.g., 4200 for 4.5ms at 1.1us per cycle)
    uint16_t max_start_space_cycles; // Maximum cycles for the space after leading pulse in
    uint16_t min_repeat_space_cycles; // Minimum cycles for the space after leading pulse in repeat codes (e.g., 2000 for 2.25ms at 1.1us per cycle)
    uint16_t max_repeat_space_cycles; // Maximum cycles for the space after leading pulse in repeat codes (e.g., 2500 for 2.25ms at 1.1us per cycle)
    uint16_t min_databit_one_cycles; // Minimum cycles for a databit 1 (e.g., 2000 for 2.25ms at 1.1us per cycle)
    uint16_t max_databit_one_cycles; // Maximum cycles for a databit 1 (e.g., 2500 for 2.25ms at 1.1us per cycle)
    uint16_t min_databit_zero_cycles; // Minimum cycles for a databit 0 (e.g., 875 for 1.125ms at 1.1us per cycle)
    uint16_t max_databit_zero_cycles; // Maximum cycles for a databit 0 (e.g., 1375 for 1.125ms at 1.1us per cycle)
} ir_decoder_config_t;

typedef enum
{
    DECODER_SUCCESS,
    INVALID_START_PULSE_LEN,
    INVALID_BIT_PULSE_LEN,
    CRC_COMMAND_FAILED,
    CRC_ADDRESS_FAILED 
} decoder_result_t;

typedef struct 
{
    /**
     * rawdata contains:
     *  - 32bit nec datagram (addr+crc+cmd+crc) in case of no errors or crc errors
     *  - the invalid pulse value in case of pulsewith failures
     **/
    uint32_t raw_data;

    /**
     * contains the status after decoding
     * - DECODER_SUCCESS in case addr and command could be decoded correctly (addr and command contain valid values)
     * - respective error codes in case of failures (addr & command are not touched)   
     */
    decoder_result_t result;
    
    /**
     * decoded addr if decoding was successful
     */
    uint16_t addr;
    
    /**
     * decoded addr if decoding was successful
     */
    uint16_t command;
    
    /**
     * set if a repeat code was received (addr & command not touched)
     */
    bool is_repeat;

} decoded_data_t;

/**
 * Decode NEC IR datagram from the raw RMT data buffer. Buffer should contain the pulse widths 
 * of the received signal, where each pulse width is represented as a 16-bit value with the 
 * level bit in the MSB and the cycle count in the lower 15 bits. The function checks for 
 * the valid NEC leading pulse, then decodes the subsequent pulses to reconstruct the datagram, 
 * verifies checksums if applicable, and extracts the address and command. The function also 
 * identifies repeat codes based on the timing of the pulses.
 * 
 * The datagram format is as follows:
 * - 9ms leading pulse
 * - 4.5ms space for normal datagram, or 2.25ms space for repeat datagram
 * - 32 bits of data, where each bit is represented by a pair of pulses: 2.25ms pulse for bit 1, 
 *      or 1.125ms pulse for bit 0.
 * 
 * The 32 bits are structured as follows:
 *  - Address (8 bits) + ~Address (8 bits) + Command (8 bits) + ~Command (8 bits) if NEC_ADDR_USE_ADDR_CRC and NEC_CMD_USE_CMD_CRC flags are set
 *  - Address (16 bits) + Command (16 bits) if NEC_ADDR_USE_ADDR_CRC and NEC_CMD_USE_CMD_CRC flags are not set
 *  - Address (16 bits) + Command (8 bits) + ~Command (8 bits) if NEC_ADDR_USE_ADDR_CRC is not set and NEC_CMD_USE_CMD_CRC is set
 *  - Address (8 bits) + ~Address (8 bits) + Command (16 bits) if NEC_ADDR_USE_ADDR_CRC is set and NEC_CMD_USE_CMD_CRC is not set
 * 
 * @param buffer Pointer to the raw RMT data buffer containing the received pulses
 * @param config Pointer to the decoder configuration structure containing the timing parameters and format flags
 * @param decoded_data Pointer to the data structure for the decoded data
 * @return true if decoding is successful, false if decoding fails
 */
bool nec_ir_decode(void *buffer, const ir_decoder_config_t *config, decoded_data_t *decoded_data);

#endif /* SRC_DECODERS_NEC_IR_DECODER_H */