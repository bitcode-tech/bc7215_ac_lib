/*
 * BC7215 Library Header
 *
 * Description:
 *   This header file defines the interfaces and data structures for the BC7215
 *   library. It includes functionalities for configuring the library,
 *   managing data transmission and reception, and handling different operational
 *   modes. The library is designed to work with BC7215, the universal IR
 *   decoder/encoder chip.
 *
 * Author: Bitcode
 * Initial Date: 2024-01-12
 *
 * Version: 4.3
 *
 * Notes:
 *   This library is dependent on 'bc7215_lib_config.h' for configuration
 *   settings and assumes the availability of standard uint8_t data types
 *   defined in 'bc7215_types.h'.
 *
 *   The library provides interfaces for IR data decoding(receiving) and encoding(transmitting),
 *   and status inquiries related to the BC7215 chip.
 *
 */

#ifndef BC7215_LIB_H
#define BC7215_LIB_H

#include <stddef.h>                   /* definition of NULL */
#include "bc7215_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /* ************************************************************* */
    /* *** Function pointer types for various callback functions *** */
    /* ************************************************************* */
    typedef void (*voidFunction_t)(void);
    typedef void (*uint8ParamFunction_t)(uint8_t);
    typedef uint8_t (*readIOFunction_t)(void);

    /* **************************************************************** */
    /* *** Configuration callback functions for the BC7215 library. *** */
    /* **************************************************************** */
    /* These are hardware related function pointers need to be provided by the user.
     * These pointers must be configured before any other library functions can be called.
     */

#if ENABLE_RECEIVING == 1
#if USE_UART_INTERRUPT == 1

    /* When no hardware flow control is used, assuming interrupt is used for UART
     * user should provide the "int_en_function()" which enables the UART interrupt.
     */
    void bc7215_config_uart_int_en_funct(voidFunction_t int_en_function);

    /* user should provide the "int_dis_function()" which disables the UART interrupt. */
    void bc7215_config_uart_int_dis_funct(voidFunction_t int_dis_function);

#elif PROCESS_IN_DIFFERENT_THREAD == 1

    /* If bc7215_process_uart_data() if called in a different thread, we need a mutex lock
     * mechanism to prevent conflict accessing to the library's internal buffer
     */
    void bc7215_config_mutex_lock_funct(voidFunction_t mutex_lock_function);
    void bc7215_config_mutex_unlock_funct(voidFunction_t mutex_unlock_function);

#endif
#endif

    /* user should provide the "send_byte_function()" to send 1 byte,
     * and this function should only return after the data has been completely sent.
     */
    void bc7215_config_uart_send_byte(uint8ParamFunction_t send_byte_function);

    /* user should provide the "set_mod_function()" to set the "mod" pin on BC7215 to high. */
    void bc7215_config_mod_set_high_funct(voidFunction_t set_mod_function);

    /* user should provide the "clear_mod_function()" to clear the "mod" pin on BC7215 to low. */
    void bc7215_config_mod_set_low_funct(voidFunction_t clear_mod_function);

    /* user should provide the "read_mod_function()" to read the status of the "MOD" pin on BC7215. */
    void bc7215_config_read_mod_funct(readIOFunction_t read_mod_function);

#if TX_HW_FLOW_CONTROL == 0
    /* user should provide the "read_busy_function()" to read the status of the "BUSY" pin on BC7215. */
    void bc7215_config_read_busy_funct(readIOFunction_t read_busy_function);
#endif

    /* ***************************** */
    /* *** data process function *** */
    /* ***************************** */
    /* If interrupt is used for UART data process, this function should be called from the user UART
     * RX interrupt service routine when a byte is received at UART from BC7215.
     * On a Windows or Linux computer, this is usually called in serial event callback.
     */
    void bc7215_process_uart_data(uint8_t data);

    /* ************************************** */
    /* *** Working mode control functions *** */
    /* ************************************** */
    /* set bc7215 to receive(decoder) mode,
     * After this function is called, user should wait at least 18ms before any other function calls being made.
     */
    void bc7215_set_rx(void);

    /* set bc7215 receive mode */
    void bc7215_set_rx_mode(uint8_t mode);

    /* set bc7215 to transmit(encoder) mode,
     * After this function is called, user should wait at least 1ms before any other function calls being made .
     */
    void bc7215_set_tx(void);

    /* set bc7215 to shutdown mode (only works in tx mode) */
    void bc7215_set_shutdown(void);
    
    /* check if BC7215 chip is in receiving or sending process, it is equal to !cmdCompleted() in TX mode. */
    uint8_t bc7215_is_busy(void);

    /* ******************************** */
    /* *** status inquiry functions *** */
    /* ******************************** */
#if ENABLE_RECEIVING == 1

    /* check if there is a data packet received, return non-zero if available */
    uint8_t bc7215_data_ready(void);

#    if ENABLE_FORMAT == 1

    /* check if there is an IR format packet received, return non-zero if available */
    uint8_t bc7215_format_ready(void);

#    endif

#endif

    /* check if bc7215_send_data() or bc7215_set_shutdown_mode() has been completed, return non-zero if completed. */
    uint8_t bc7215_cmd_completed(void);

    /* **************************************** */
    /* *** Receiving mode related functions *** */
    /* **************************************** */
#if ENABLE_RECEIVING == 1

    /* clear received data packet */
    void bc7215_clr_data(void);

    /* clear received format packet */
    void bc7215_clr_format(void);

    /* get the bit length of payload of the received data packet */
    uint16_t bc7215_get_len(void);

    /* get the number of bytes of the received data packet */
    uint16_t bc7215_dpkt_size(void);

    /* copy received IR payload data to "target", return the status/signature byte,
     * or "0xff" if data is no longer available (overwritten by new incoming data)
     */
    uint8_t bc7215_get_data(bc7215DataVarPkt_t* target);

    /* Get payload data from received data packet into any address directly. It will
     * not include the 'bitLen' part of the data packet. 'size' is number of bytes to be
     * copied, if the actual size is smaller than the 'size', only actual size is copied.
     * The return value is the actual number of bytes copied.
     */
#    if BC7215_MAX_RX_DATA_SIZE >= 256
    uint16_t bc7215_get_raw(void* addr, uint16_t size);
#    else
    uint8_t bc7215_get_raw(void* addr, uint8_t size);
#    endif

#    if ENABLE_FORMAT == 1

    /* copy IR format to 'target', return the signature byte,
     * or "0xff" if data is no longer available (overwritten by new incoming data)
     */
    uint8_t bc7215_get_format(bc7215FormatPkt_t* target);

#    endif

#endif

#if ENABLE_TRANSMITTING == 1

    /* ******************************************* */
    /* *** Transmitting mode related functions *** */
    /* ******************************************* */
    /* download IR format to bc7215 */
    void bc7215_load_format(const bc7215FormatPkt_t* source);

    /* Send data via IR
     * if data is longer than 16 bytes, this function will only return when there are 16 or less
     * bytes left in the queue to be sent.
     */
    void bc7215_IR_tx(const bc7215DataVarPkt_t* source);

    /* Send raw data via IR
     * This function send any data, 'size' is in bytes, range from 1 to 511.
     * Please keep in mind IR sending rate is slow so big sized data takes
     * long time to send and this function only returns when there are 16 or less
     * bytes left in the queue to be sent.
     */
    void bc7215_send_raw(const void* rawData, uint16_t size);

#endif

    /* set sending IR format to use 56k carrier */
    void bc7215_set_C56K(bc7215FormatPkt_t* target);

    /* clear sending IR format to use 37.5k carrier */
    void bc7215_clr_C56K(bc7215FormatPkt_t* target);

    /* set sending IR format to use no carrier */
    void bc7215_set_NOCA(bc7215FormatPkt_t* target);

    /* clear NOCA bit in format to enable carrier */
    void bc7215_clr_NOCA(bc7215FormatPkt_t* target);

    /* calculate crc-8, a tool for communication */
#if BC7215_MAX_RX_DATA_SIZE < 256
    uint8_t bc7215_crc8(const uint8_t* data, uint8_t len);
#else
    uint8_t bc7215_crc8(const uint8_t* data, uint16_t len);
#endif

    /* calculate the data packet size */
    uint16_t bc7215_cal_size(const bc7215DataVarPkt_t* dataPkt);

    /* copy data packet from source to any address. if they are overlapped, the source
     * packet would be destroyed after calling this function.
     */
    void bc7215_copy_dpkt(void* target, const bc7215DataVarPkt_t* source);

    /* compare if 2 data packets are equal. return 1 if true, 0 if false. */
    uint8_t bc7215_compare_dpkt(uint8_t sig, const bc7215DataVarPkt_t* pkt1, const bc7215DataVarPkt_t* pkt2);

#ifdef __cplusplus
}
#endif

#endif
