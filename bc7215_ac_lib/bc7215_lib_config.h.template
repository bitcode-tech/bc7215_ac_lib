/*
 * BC7215 Library Configuration Header
 *
 * Description:
 *   This configuration header file provides the necessary constants and macros
 *   for the BC7215 library. It defines key parameters such as maximum payload data
 *   length and other configurable settings that directly affect the operation of
 *   the BC7215 library. This file allows for customization of the library to suit
 *   different requirements and constraints.
 *
 * Author: Bitcode
 * Date: 2024-01-12
 *
 *
 * Notes:
 *   This configuration file is essential for the BC7215 library. The definitions
 *   here, especially BC7215_MAX_DATA_BYTE_LENGTH, determine the memory requirements
 *   and limitations of the library in handling IR data. Adjustments to these values
 *   should be made with an understanding of the memory constraints and IR data
 *   requirements of the specific application.
 *
 */

#ifndef BC7215_LIB_CONFIG_H
#define BC7215_LIB_CONFIG_H

/* If UART sending uses hardware flow control (CTS) 0 = No */
#define TX_HW_FLOW_CONTROL 0

/* If bc7215_process_uart_data() is called in UART interrupt 1 = Yes */
#define USE_UART_INTERRUPT 1

/* If bc7215_process_uart_data() is not called in interrupt but in a different thread 0 = No */
#define PROCESS_IN_DIFFERENT_THREAD 0

/* If IR receiving(decoding) is enabled, 1 = Yes
 * change this value to '0' to save system resources (less code and less RAM used) 
 */
#define ENABLE_RECEIVING 1

#if ENABLE_RECEIVING == 1

/* If only data packet is needed and BC7215 will always work in simple mode, 1 = Yes
 * change this value to '0' to save system resources (less code and less RAM used)
 */
#define ENABLE_FORMAT 1

#endif

/* If IR transmitting is enabled, 1 = Yes
 * change this value to '0' save system resources (less code)
 */
#define ENABLE_TRANSMITTING 1

/* Maximum processable payload data length in byte, this value must <= 512,
 * most IR remote controllers send less than 42 bytes. The larger this value,
 * the larger memory is required to run this library.
 */
#define BC7215_MAX_RX_DATA_SIZE 56

/* the polynominal used for CRC calculation, default is 0x07 for CRC-8-CCITT */
#define BC7215_CRC8_POLY 0x07

#endif /* BC7215_LIB_CONFIG_H */
