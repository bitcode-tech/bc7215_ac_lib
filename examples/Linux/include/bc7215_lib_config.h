/*
 * BC7215 Linux port configuration.
 *
 * BUSY must be connected to the serial port CTS input and MOD must be
 * connected to the serial port DTR output.
 */
#ifndef BC7215_LIB_CONFIG_H
#define BC7215_LIB_CONFIG_H

#define TX_HW_FLOW_CONTROL          1
#define USE_UART_INTERRUPT          0
#define PROCESS_IN_DIFFERENT_THREAD 1

#define ENABLE_RECEIVING            1
#if ENABLE_RECEIVING == 1
#define ENABLE_FORMAT               1
#endif
#define ENABLE_TRANSMITTING         1

#define BC7215_MAX_RX_DATA_SIZE     56
#define BC7215_CRC8_POLY            0x07

#endif /* BC7215_LIB_CONFIG_H */
