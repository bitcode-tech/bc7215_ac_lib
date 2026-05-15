/*
 * bc7215_types.h
 * BC7215 Universal IR Encoder/Decoder Chip - Type Definitions
 *  
 * This header file defines the data structures used for communication with
 * the BC7215 chip. These structures handle:
 * - Variable and fixed-size data packets for IR transmission/reception
 * - Format packets containing timing and protocol information
 * - Signature and control flags for different IR protocols
 *
 * Dependencies:
 *   - bc7215_config.h for configuration constants
 *
 * Author:
 *   Bitcode
 *
 * Version:
 *   V4.0 July 2025
 *
 * License:
 *   MIT License
 */

#ifndef BC7215_TYPES_H
#define BC7215_TYPES_H

#include "bc7215_lib_config.h"

/* Conditionally include stdint.h or define uint8_t and uint16_t for C89 compatibility */
#if defined(__cplusplus) && __cplusplus >= 201103L
    #include <cstdint>
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 199901L
    #include <stdint.h>
#else
    #ifndef UINT8_T
    #define UINT8_T
    typedef unsigned char uint8_t;
    #endif
    #ifndef UINT16_T
    #define UINT16_T
    typedef unsigned short uint16_t;
    #endif
#endif

/* **************************************************** */
/* *** IR Data Packet Structure Definitions        *** */
/* **************************************************** */

/*
 * Variable-size IR data packet structure
 * 
 * This structure is used for IR data packets where the actual data length
 * varies. The 'data' array has only one element as a placeholder - the
 * actual size is determined by the bitLen field and allocated dynamically.
 * 
 * Typical usage: Allocated as pointer with variable size based on actual data length
 * 
 * Memory layout:
 * +----------+----------+----------+----------+-----+
 * | bitLen   | bitLen   | data[0]  | data[1]  | ... |
 * | (LSB)    | (MSB)    |          |          |     |
 * +----------+----------+----------+----------+-----+
 *   byte 0     byte 1     byte 2     byte 3    ...
 */
typedef struct bc7215DataVarPkt_t
{
    uint16_t bitLen;    /* Length of IR data in bits (0-4095) */
    uint8_t  data[1];   /* Variable-length data array (placeholder - actual size varies) */
} bc7215DataVarPkt_t;

/*
 * Fixed maximum-size IR data packet structure
 * 
 * This structure provides a fixed-size buffer that can accommodate the largest
 * possible IR data packet that the BC7215 can handle. The actual data length
 * is still specified by bitLen, but the buffer is pre-allocated to maximum size.
 * 
 * Typical usage: Stack allocation or as member variable when maximum size is acceptable
 * 
 * Memory layout:
 * +----------+----------+----------+----------+-----+----------+
 * | bitLen   | bitLen   | data[0]  | data[1]  | ... | data[MAX]|
 * | (LSB)    | (MSB)    |          |          |     |          |
 * +----------+----------+----------+----------+-----+----------+
 *   byte 0     byte 1     byte 2     byte 3    ...   byte MAX+1
 */
typedef struct bc7215DataMaxPkt_t
{
    uint16_t bitLen;                            /* Length of IR data in bits (0-4095) */
    uint8_t  data[BC7215_MAX_RX_DATA_SIZE];     /* Fixed-size data buffer (maximum capacity) */
} bc7215DataMaxPkt_t;

/*
 * IR Protocol Format Packet Structure
 * 
 * This structure defines the timing parameters and protocol characteristics
 * for IR transmission and reception. It contains:
 * - Protocol signature with control flags
 * - Detailed timing parameters for IR waveform generation
 * 
 * The format packet must be loaded before transmitting data to ensure
 * correct timing and carrier frequency settings.
 * 
 * Memory layout:
 * +----------+----------+----------+----------+-----+----------+
 * |signature | format[0]| format[1]| format[2]| ... |format[31]|
 * |   byte   |          |          |          |     |          |
 * +----------+----------+----------+----------+-----+----------+
 *   byte 0     byte 1     byte 2     byte 3    ...   byte 32
 */
typedef struct bc7215FormatPkt_t
{
    /*
     * Protocol signature byte with embedded control flags
     * 
     * This union allows access to the signature as either:
     * - Individual bit fields for flag manipulation
     * - Complete byte for direct read/write operations
     */
    union
    {
        /*
         * Bit field structure for individual flag access
         */
        struct
        {
            uint8_t sig : 6;    /* Protocol signature (bits 0-5): Identifies IR protocol type (0-63) */
            uint8_t c56k : 1;   /* 56kHz carrier flag (bit 6): 1=Enable 56kHz carrier, 0=Use standard 38kHz */
            uint8_t noCA : 1;   /* No Carrier flag (bit 7): 1=Disable carrier (baseband), 0=Enable carrier */
        } bits;
        
        uint8_t inByte;         /* Complete signature byte for direct access */
    } signature;
    
    /*
     * Format parameter array (32 bytes)
     * 
     * Contains timing parameters that define the IR protocol characteristics:
     * - Pulse/space durations
     * - Header timing
     * - Bit encoding timing (0 and 1 representations)
     * - Repeat sequence timing
     * - Inter-frame gaps
     * 
     */
    uint8_t format[32];         /* Protocol timing parameters (32 bytes) */
} bc7215FormatPkt_t;

#endif
