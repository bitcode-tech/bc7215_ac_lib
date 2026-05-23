
/*
 * BC7215 Library Implementation
 *
 * Description:
 *   This source file contains the implementation of the BC7215 library. It provides
 *   the functionality to interact with the BC7215 chip, including setting up the
 *   hardware interface, managing data transmission and reception, and controlling
 *   different operational modes. This implementation relies on the definitions and
 *   configurations set in 'bc7215_lib.h' and 'bc7215_lib_config.h'.
 *
 * Author: Bitcode
 * Initial Date: 2024-01-12
 *
 * Current Version: 4.3
 *
 * Notes:
 *   The functions defined in this file are designed to provide a seamless interface
 *   for working with the BC7215 IR decoder/encoder chip. Users should familiarize
 *   themselves with the API provided and configure the library according to their
 *   application's requirements using 'bc7215_lib_config.h'.
 *
 *   The implementation assumes a compatible hardware setup and correct configuration
 *   settings. It is crucial for users to ensure that their hardware setup matches
 *   the expectations of this library to avoid any operational issues.
 *
 */

#include "bc7215_lib.h"

/* Define assert macro for C89 compatibility if <assert.h> is unavailable */
#ifdef NDEBUG
#    include <assert.h>
#else
/* Custom assert macro: enters infinite loop on failure (mimics original behavior) */
#    define assert(expr)                                                                                               \
        do                                                                                                             \
        {                                                                                                              \
            if (!(expr))                                                                                               \
            { /* Assertion failed: expr */                                                                             \
                while (1)                                                                                              \
                    ;                                                                                                  \
            }                                                                                                          \
        } while (0)
#endif

/* ********************************************************** */
/* *** Callback function pointers for the BC7215 library. *** */
/* ********************************************************** */
/* These pointers point to hardware related functions.
 * These pointers must be configured before any other library functions can be called.
 */
#if ENABLE_RECEIVING == 1
static voidFunction_t enUartIntFunction = NULL;  /* pointer to UART interrupt disable function */
static voidFunction_t disUartIntFunction = NULL; /* pointer to UART interrupt enable function */
#endif
#if PROCESS_IN_DIFFERENT_THREAD == 1
static voidFunction_t mutexLockFunction = NULL;
static voidFunction_t mutexUnlockFunction = NULL;
#endif
static uint8ParamFunction_t uartSendByteFunction = NULL; /* pointer to UART send 1 byte function */
static voidFunction_t       modSetHighFunction = NULL;   /* pointer to mod pin set high function */
static voidFunction_t       modSetLowFunction = NULL;    /* pointer to mod pin set low function */
static readIOFunction_t     readModFunction = NULL;      /* pointer to read mod pin function */
#if TX_HW_FLOW_CONTROL == 0
static readIOFunction_t readBusyFunction = NULL; /* pointer to read busy pin function */
#endif

/* **************************************************************** */
/* *** Internal variables and functions for the BC7215 library. *** */
/* **************************************************************** */
volatile struct
{
    uint8_t formatPktReady : 1;
    uint8_t dataPktReady : 1;
    uint8_t pktStarted : 1;
    uint8_t overLap : 1;
    uint8_t cmdComplete : 1;
} bc7215Status;

#if BC7215_BUFFER_SIZE < 256
typedef struct
{
    uint16_t bitLen;
    uint8_t  start;
    uint8_t  end;
    uint8_t  count;
} pktInfo_t;
#else
typedef struct
{
    uint16_t bitLen;
    uint16_t start;
    uint16_t end;
    uint16_t count;
} pktInfo_t;
#endif

#if ENABLE_RECEIVING == 1
#    if ENABLE_FORMAT == 1

/* buffer size is data packet size + format packet size */
#        define BC7215_BUFFER_SIZE ((BC7215_MAX_RX_DATA_SIZE + 3) + (32 + 1))

#    else

#        define BC7215_BUFFER_SIZE (BC7215_MAX_RX_DATA_SIZE + 3)

#    endif

static uint8_t   circularBuffer[BC7215_BUFFER_SIZE];
static pktInfo_t curPktInfo;
static pktInfo_t prePktInfo;

#    if BC7215_BUFFER_SIZE < 256
static uint8_t startPos;
static int8_t  lastWritingPos;
static uint8_t byteCount;
static uint8_t bufBackRead(uint8_t pos, uint8_t n);
static uint8_t bufRead(uint8_t pos, uint8_t n);
#    else
static uint16_t startPos;
static uint16_t lastWritingPos;
static uint16_t byteCount;
static uint8_t  bufBackRead(uint16_t pos, uint16_t n);
static uint8_t  bufRead(uint16_t pos, uint16_t n);
#    endif
/* bufBackRead(): read the (n)th byte backwards from the pos, n range from 0 to BC7215_BUFFER_SIZE-1 */
/* bufRead():  read the (n)th byte after the pos, n range from 0 to BC7215_BUFFER_SIZE-1 */

/* **************************************** */
/* *** Callback configuration functions *** */
/* **************************************** */

/* UART interrupt enable function configuration
 * The function must have a signature of " void function_name(void) "
 */
void bc7215_config_uart_int_en_funct(voidFunction_t en_int_function)
{
    assert(en_int_function != NULL); /* Ensure UART interrupt enable function is provided */
    enUartIntFunction = en_int_function;
}

/* UART interrupt disable function configuration
 * The function must have a signature of " void function_name(void) "
 */
void bc7215_config_uart_int_dis_funct(voidFunction_t dis_int_function)
{
    assert(dis_int_function != NULL); /* Ensure UART interrupt disable function is provided */
    disUartIntFunction = dis_int_function;
}
#endif

#if PROCESS_IN_DIFFERENT_THREAD == 1
void bc7215_config_mutex_lock_funct(voidFunction_t mutex_lock_function)
{
    assert(mutex_lock_function != NULL); /* Ensure mutex lock function is provided */
    mutexLockFunction = mutex_lock_function;
}

void bc7215_config_mutex_unlock_funct(voidFunction_t mutex_unlock_function)
{
    assert(mutex_unlock_function != NULL); /* Ensure mutex unlock function is provided */
    mutexUnlockFunction = mutex_unlock_function;
}
#endif

#if ENABLE_TRANSMITTING == 1
static void byte_stuffing_send(uint8_t data);
#endif

/* UART send 1 byte function configuration
 * The function must have a signature of " void function_name(uint8_t) ",
 * and this function should only return after the data has been completely sent.
 */
void bc7215_config_uart_send_byte(uint8ParamFunction_t send_byte_function)
{
    assert(send_byte_function != NULL); /* Ensure UART send byte function is provided */
    uartSendByteFunction = send_byte_function;
}

/* MOD pin set high function configuration
 * The function must have a signature of " void function_name(void) ",
 */
void bc7215_config_mod_set_high_funct(voidFunction_t set_mod_function)
{
    assert(set_mod_function != NULL); /* Ensure MOD pin set high function is provided */
    modSetHighFunction = set_mod_function;
}

/* MOD pin set low function configuration
 * The function must have a signature of " void function_name(void) ",
 */
void bc7215_config_mod_set_low_funct(voidFunction_t clear_mod_function)
{
    assert(clear_mod_function != NULL); /* Ensure MOD pin set low function is provided */
    modSetLowFunction = clear_mod_function;
}

/* Read MOD pin function configuration
 * The function must have a signature of " uint8_t function_name(void) ",
 * return 0 if MOD pin is low, non-zero if MOD pin is high
 */
void bc7215_config_read_mod_funct(readIOFunction_t read_mod_function)
{
    assert(read_mod_function != NULL); /* Ensure MOD pin read function is provided */
    readModFunction = read_mod_function;
}

#if TX_HW_FLOW_CONTROL == 0
/* Read BUSY pin function configuration
 * The function must have a signature of " uint8_t function_name(void) ",
 * return 0 if BUSY pin is low, non-zero if BUSY pin is high
 */
void bc7215_config_read_busy_funct(readIOFunction_t read_busy_function)
{
    assert(read_busy_function != NULL); /* Ensure BUSY pin read function is provided */
    readBusyFunction = read_busy_function;
}
#endif

/* ***************************** */
/* *** data process function *** */
/* ***************************** */
/* this function should be called from the user UART RX interrupt service routine when a byte
 * is received from BC7215.
 */
void bc7215_process_uart_data(uint8_t data)
{
#if ENABLE_RECEIVING == 1
    static uint8_t previousData = 0;
    uint8_t        temp;
#endif

    if (readModFunction() == 0) /* MOD=0 means bc7215 is in transmit mode */
    {
        if (data == 0x7a)
        {
            bc7215Status.cmdComplete = 1;
        }
    }

#if ENABLE_RECEIVING == 1
    else /* bc7215 is in receive mode */
    {
#    if PROCESS_IN_DIFFERENT_THREAD == 1
        mutexLockFunction(); /* wait and lock resource */
#    endif
        if (data == 0x7a)
        {
            if (!bc7215Status.overLap)
            {
                if (previousData == 0x7a) /* if 0x7a 0x7a is the mark of format packet */
                {
                    curPktInfo
                        = prePktInfo;        // just received packet was a format packet, restore saved data packet info
                    bc7215Status.dataPktReady = 0;
#    if ENABLE_FORMAT == 1
                    if (byteCount == 33) /* check the packet size */
                    {
                        bc7215Status.formatPktReady = 1;
                    }
#    endif
                    // if packet length is as expected
                    if ((byteCount + curPktInfo.count <= BC7215_BUFFER_SIZE)
                        && ((curPktInfo.bitLen + 7) / 8 + 3 == curPktInfo.count))
                    {
                        bc7215Status.dataPktReady = 1;
                    }
                }
                else /* if this is the first 0x7a received */
                {
                    prePktInfo = curPktInfo;        // new packet received, backup previous packet information
                    curPktInfo.start = startPos;
                    curPktInfo.end = lastWritingPos;
                    curPktInfo.count = byteCount;
                    curPktInfo.bitLen
                        = ((uint16_t)bufBackRead(lastWritingPos, 0) << 8) | bufBackRead(lastWritingPos, 1);
                    /* get the bit count of the data packet */

                    if ((curPktInfo.bitLen + 7) / 8 + 3
                        == byteCount) /* if the byte count of received packet is correct */
                    {
                        bc7215Status.dataPktReady = 1;
                    }
                }
            }
            previousData = 0x7a;
            bc7215Status.pktStarted = 0;
        }
        else /* incoming data is not 0x7a */
        {
            if (!bc7215Status.pktStarted) /* if it's the start of a new packet */
            {
                bc7215Status.pktStarted = 1; /* clear new packet indicator */
                bc7215Status.overLap = 0;
                byteCount = 0;
                bc7215Status.dataPktReady = 0; /* new data is coming, clear dataPktReady and formatPktReady flags */
                bc7215Status.formatPktReady = 0;
                startPos = lastWritingPos + 1; /* save start buffer position of the packet */
                if (startPos >= BC7215_BUFFER_SIZE)
                {
                    startPos = 0;
                }
            }
            if (data == 0x7b) /* byte-destuffing, ignore 0x7b */
            {
                previousData = 0x7b;
#    if PROCESS_IN_DIFFERENT_THREAD == 1
		        mutexUnlockFunction(); /* release resource */
#    endif
                return;
            }
            else
            {
                temp = data;
                if (previousData == 0x7b)
                {
                    data &= 0x7f;
                }
                previousData = temp;
            } /* end of byte-destuffing */
            /* save data to buffer */
            lastWritingPos++;
            if (lastWritingPos >= BC7215_BUFFER_SIZE)
            {
                lastWritingPos = 0;
            }
            circularBuffer[lastWritingPos] = data;
            byteCount++;
            if (byteCount > BC7215_BUFFER_SIZE)
            {
                bc7215Status.overLap = 1;
            }
        }
#    if PROCESS_IN_DIFFERENT_THREAD == 1
        mutexUnlockFunction(); /* release resource */
#    endif
    }
#endif
}

/* ************************************** */
/* *** Working mode control functions *** */
/* ************************************** */

/* set bc7215 to receive(decoder) mode,
 * After this function is called, user should wait at least 18ms before any other function calls being made.
 */
void bc7215_set_rx(void)
{
    modSetHighFunction();
    bc7215Status.cmdComplete = 1;
}

/* set bc7215 to transmit(encoder) mode,
 * After this function is called, user should wait at least 1ms before any other function calls being made .
 */
void bc7215_set_tx(void)
{
    modSetLowFunction();
    bc7215Status.dataPktReady = 0;
    bc7215Status.formatPktReady = 0;
    bc7215Status.pktStarted = 0;
    bc7215Status.cmdComplete = 1;
}

/* set bc7215 to shutdown mode (only works in tx mode) */
void bc7215_set_shutdown(void)
{
    if (readModFunction() == 0) /* if bc7215 is in transmit mode */
    {
        uartSendByteFunction(0xf7);
        uartSendByteFunction(0x00);
    }
    bc7215Status.cmdComplete = 0;
}

#if ENABLE_RECEIVING == 1
/* set bc7215 receive mode */
void bc7215_set_rx_mode(uint8_t mode)
{
#    if TX_HW_FLOW_CONTROL == 0
    while (readBusyFunction() != 0)
        ; /* wait if bc7215 busy is high */
#    endif
    uartSendByteFunction(mode);
}

/* ******************************** */
/* *** status inquiry functions *** */
/* ******************************** */

uint8_t bc7215_is_busy(void) { return (readModFunction() == 0) ? !bc7215Status.cmdComplete : bc7215Status.pktStarted; }

/* check if there is a data packet received, return non-zero if available */
uint8_t bc7215_data_ready(void)
{
    if (readModFunction() != 0) /* if bc7215 is in receive mode */
    {
        return bc7215Status.dataPktReady;
    }
    else
    {
        return 0;
    }
}

#    if ENABLE_FORMAT == 1
/* check if there is an IR format packet received, return non-zero if available */
uint8_t bc7215_format_ready(void)
{
    if (readModFunction() != 0) /* if bc7215 is in receive mode */
    {
        return bc7215Status.formatPktReady;
    }
    else
    {
        return 0;
    }
}
#    endif
#endif

/* check if bc7215_send_data() or bc7215_set_shutdown_mode() has been completed, return non-zero if completed. */
uint8_t bc7215_cmd_completed(void)
{
    if (readModFunction() == 0) /* if bc7215 is in transmit mode */
    {
        return bc7215Status.cmdComplete;
    }
    else
    {
        return 1;
    }
}

#if ENABLE_RECEIVING == 1
/* **************************************** */
/* *** Receiving mode related functions *** */
/* **************************************** */

/* clear received data packet */
void bc7215_clr_data(void) { bc7215Status.dataPktReady = 0; }

/* clear received format packet */
void bc7215_clr_format(void) { bc7215Status.formatPktReady = 0; }

/* get the bit length of the received data packet */
uint16_t bc7215_get_len(void)
{
    if (bc7215Status.dataPktReady)
    {
        return curPktInfo.bitLen;
    }
    else
    {
        return 0;
    }
}

/* get the number of bytes of the received data packet */
uint16_t bc7215_dpkt_size(void)
{
    if (bc7215Status.dataPktReady)
    {
        return (curPktInfo.bitLen + 7) / 8 + 2;
    }
    else
    {
        return 0;
    }
}

/* Get IR payload data when it's available
 * copy received IR payload data to "target", return the status/signature byte,
 * or "0xff" if data is no longer available (overwritten by new incoming data)
 */
uint8_t bc7215_get_data(bc7215DataVarPkt_t* target)
{
    uint8_t  status;
    uint16_t i;
    uint16_t rtnBitLen;

#    if USE_UART_INTERRUPT == 1
    disUartIntFunction(); /* disable UART interrupt */
#    endif
#    if PROCESS_IN_DIFFERENT_THREAD == 1
    mutexLockFunction(); /* wait and lock resource */
#    endif
    status = 0xff;        // if data packet is not ready, return 0xff(Err bit = 1)
    rtnBitLen = 8;        // if data not ready or bitLen is out of range, only return a minimum value
    if (bc7215Status.dataPktReady)
    {
        status = bufBackRead(curPktInfo.end, 2);
        if (curPktInfo.bitLen < BC7215_MAX_RX_DATA_SIZE * 8)
        {
            rtnBitLen = curPktInfo.bitLen;
        }
        for (i = 0; i < (rtnBitLen + 7) / 8; i++)
        {
            target->data[i] = bufRead(curPktInfo.start, i);        // datStartPos, i);
        }
        bc7215Status.dataPktReady = 0;
    }
    target->bitLen = rtnBitLen;
#    if USE_UART_INTERRUPT == 1
    enUartIntFunction(); /* enable UART interrupt */
#    endif
#    if PROCESS_IN_DIFFERENT_THREAD == 1
    mutexUnlockFunction(); /* release resource */
#    endif
    return status;
}

/* Get payload data from received data packet into any address directly. It will
 * not include the 'bitLen' part of the data packet. 'size' is number of bytes to be
 * copied, if the actual size is smaller than the 'size', only actual size is copied.
 * The return value is the actual number of bytes copied.
 */
#    if BC7215_MAX_RX_DATA_SIZE >= 256
uint16_t bc7215_get_raw(void* addr, uint16_t size)
{
    uint16_t i;
#    else
uint8_t         bc7215_get_raw(void* addr, uint8_t size)
{
    uint8_t  i;
#    endif

#    if USE_UART_INTERRUPT == 1
    disUartIntFunction(); /* disable UART interrupt */
#    endif
#    if PROCESS_IN_DIFFERENT_THREAD == 1
    mutexLockFunction(); /* wait and lock resource */
#    endif
    if (bc7215Status.dataPktReady)
    {
        bc7215Status.dataPktReady = 0;
        if (size > (curPktInfo.bitLen + 7) / 8)
        {
            size = (curPktInfo.bitLen + 7) / 8;
        }

        if (size != 0)
        {
            for (i = 0; i < size; i++)
            {
                *((uint8_t*)addr + i) = bufRead(curPktInfo.start, i);        // datStartPos, i);
            }
        }
    }
    else
    {
        size = 0;
    }
#    if USE_UART_INTERRUPT == 1
    enUartIntFunction(); /* enable UART interrupt */
#    endif
#    if PROCESS_IN_DIFFERENT_THREAD == 1
    mutexUnlockFunction(); /* release resource */
#    endif
    return size;
}

#    if ENABLE_FORMAT == 1
/* Get IR format data when it's available
 * copy IR format to 'target', return the signature byte,
 * or "0xff" if data is no longer available (overwritten by new incoming data)
 */
uint8_t bc7215_get_format(bc7215FormatPkt_t* target)
{
    uint8_t i;
    uint8_t signature;
#        if USE_UART_INTERRUPT == 1
    disUartIntFunction(); /* disable UART interrupt */
#        endif
#        if PROCESS_IN_DIFFERENT_THREAD == 1
    mutexLockFunction(); /* wait and lock resource */
#        endif
    signature = 0xff;
    if (bc7215Status.formatPktReady)
    {
        signature = bufRead(startPos, 0);
        target->signature.inByte = signature;
        for (i = 0; i < 32; i++)
        {
            target->format[i] = bufRead(startPos, i + 1);
        }
        bc7215Status.formatPktReady = 0;
    }
#        if USE_UART_INTERRUPT == 1
    enUartIntFunction(); /* enable UART interrupt */
#        endif
#        if PROCESS_IN_DIFFERENT_THREAD == 1
    mutexUnlockFunction(); /* release resource */
#        endif
    return signature;
}
#    endif
#endif

#if ENABLE_TRANSMITTING == 1
/* ******************************************* */
/* *** Transmitting mode related functions *** */
/* ******************************************* */

/* download IR format to bc7215 */
void bc7215_load_format(const bc7215FormatPkt_t* source)
{
    uint8_t i;
    if (readModFunction() == 0) /* if bc7215 is in transmit mode */
    {
#    if TX_HW_FLOW_CONTROL == 0
        while (readBusyFunction() != 0)
            ; /* wait if bc7215 busy is high */
#    endif
        uartSendByteFunction(0xf6);
#    if TX_HW_FLOW_CONTROL == 0
        while (readBusyFunction() != 0)
            ;
#    endif
        uartSendByteFunction(0x01);
        byte_stuffing_send(source->signature.inByte);
        for (i = 0; i < 32; i++)
        {
            byte_stuffing_send(source->format[i]);
        }
    }
}

/* Send data via IR
 * if data is longer than 16 bytes, this function will only return after all the excessive bytes are sent.
 */
void bc7215_IR_tx(const bc7215DataVarPkt_t* source)
{
#    if (BC7215_MAX_RX_DATA_SIZE + 2) < 256
    uint8_t i;
    uint8_t bytes;
#    else
    uint16_t i;
    uint16_t bytes;
#    endif
    if ((readModFunction() == 0) && (source->bitLen >= 8)) /* check if bc7215 is in transmitting mode */
    {
        bc7215Status.cmdComplete = 0;
#    if TX_HW_FLOW_CONTROL == 0
        while (readBusyFunction() != 0)
            ; /* wait if bc7215 busy is high */
#    endif
        uartSendByteFunction(0xf5);
#    if TX_HW_FLOW_CONTROL == 0
        while (readBusyFunction() != 0)
            ;
#    endif
        uartSendByteFunction(0x02);
        byte_stuffing_send(source->bitLen & 0xff);
        byte_stuffing_send(source->bitLen >> 8);
        bytes = (source->bitLen + 7) / 8; /* set bytes = total number of data bytes +2 */
        for (i = 0; i < bytes; i++)       /* send from 2nd byte of the data packet */
        {
            byte_stuffing_send(source->data[i]);
        }
    }
}

/* Send raw data via IR
 * This function send any data, 'size' is in bytes, range from 1 to 511.
 * Please keep in mind IR sending rate is slow so big sized data takes
 * long time to send and this function only returns when there are 16 or less
 * bytes left in the queue to be sent.
 */
void bc7215_send_raw(const void* rawData, uint16_t size)
{
    uint16_t i;
    if ((readModFunction() == 0) && (size < 0x200))
    {
        bc7215Status.cmdComplete = 0;
#    if TX_HW_FLOW_CONTROL == 0
        while (readBusyFunction() != 0)
            ; /* wait if bc7215 busy is high */
#    endif
        uartSendByteFunction(0xf5);
#    if TX_HW_FLOW_CONTROL == 0
        while (readBusyFunction() != 0)
            ;
#    endif
        uartSendByteFunction(0x02);
        byte_stuffing_send((size * 8) & 0xff); /* send bit length of the data */
        byte_stuffing_send((size * 8) >> 8);
        for (i = 0; i < size; i++)
        {
            byte_stuffing_send(*((uint8_t*)rawData + i));
        }
    }
}

static void byte_stuffing_send(uint8_t data)
{
#    if TX_HW_FLOW_CONTROL == 0
    while (readBusyFunction() != 0)
        ;
#    endif
    if ((data == 0x7a) || (data == 0x7b))
    {
        uartSendByteFunction(0x7b);
#    if TX_HW_FLOW_CONTROL == 0
        while (readBusyFunction() != 0)
            ;
#    endif
        uartSendByteFunction(data | 0x80);
    }
    else
    {
        uartSendByteFunction(data);
    }
}
#endif

/* set sending IR format to use 56k carrier */
void bc7215_set_C56K(bc7215FormatPkt_t* target) { target->signature.bits.c56k = 1; }

/* clear sending IR format to use 37.5k carrier */
void bc7215_clr_C56K(bc7215FormatPkt_t* target) { target->signature.bits.c56k = 0; }

/* set sending IR format to use no carrier */
void bc7215_set_NOCA(bc7215FormatPkt_t* target) { target->signature.bits.noCA = 1; }

/* clear NOCA bit in format to enable carrier */
void bc7215_clr_NOCA(bc7215FormatPkt_t* target) { target->signature.bits.noCA = 0; }

#if ENABLE_RECEIVING == 1
/* ***************************** */
/* *** Buffer read functions *** */
/* ***************************** */

/* read the (n)th byte backwards from the pos, n range from 0 to BC7215_BUFFER_SIZE-1 */
#    if BC7215_BUFFER_SIZE < 256
static uint8_t bufBackRead(uint8_t pos, uint8_t n)
#    else
static uint8_t bufBackRead(uint16_t pos, uint16_t n)
#    endif
{
    if (pos >= n)
    {
        return circularBuffer[pos - n];
    }
    else
    {
        return circularBuffer[BC7215_BUFFER_SIZE + pos - n];
    }
}

/* read the (n)th byte start from the pos, n range from 0 to BC7215_BUFFER_SIZE-1 */
#    if BC7215_BUFFER_SIZE < 256
static uint8_t bufRead(uint8_t pos, uint8_t n)
#    else
static uint8_t bufRead(uint16_t pos, uint16_t n)
#    endif
{
    if (pos + n >= BC7215_BUFFER_SIZE)
    {
        return circularBuffer[pos + n - BC7215_BUFFER_SIZE];
    }
    else
    {
        return circularBuffer[pos + n];
    }
}
#endif

/* calculate CRC-8 */
#if BC7215_MAX_RX_DATA_SIZE < 256
uint8_t bc7215_crc8(const uint8_t* data, uint8_t len)
{
    uint8_t i;
    uint8_t j;
    uint8_t crc = 0;
    for (i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ BC7215_CRC8_POLY;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}
#else
uint8_t bc7215_crc8(const uint8_t* data, uint16_t len)
{
    uint16_t i;
    uint8_t  j;
    uint8_t  crc = 0;
    for (i = 0; i < len; ++i)
    {
        crc ^= data[i];
        for (j = 0; j < 8; ++j)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ BC7215_CRC8_POLY;
            }
            else
            {
                crc <<= 1;
            }
        }
    }
    return crc;
}
#endif

/* calculate the size (bytes) of a data packet */
uint16_t bc7215_cal_size(const bc7215DataVarPkt_t* dataPkt)
{
    if (dataPkt->bitLen < 0x1000)
    {
        return (dataPkt->bitLen + 7) / 8 + 2;
    }
    else
    {
        return 0;
    }
}

/* copy data packet from source to any address. if they are overlapped, the source
 * packet would be destroyed after calling this function.
 */
void bc7215_copy_dpkt(void* target, const bc7215DataVarPkt_t* source)
{
    uint16_t i;
    uint16_t totalLen;
    totalLen = bc7215_cal_size(source);
    if ((void*)source > target) /* if source is located behind target in memory, copy from front to end */
    {
        for (i = 0; i < totalLen; i++)
        {
            *((uint8_t*)target + i) = *((uint8_t*)source + i);
        }
    }
    else if ((void*)source < target) /* if source is located before target in memory, copy from end to front */
    {
        for (i = totalLen; i > 0; i--)
        {
            *((uint8_t*)target + i - 1) = *((uint8_t*)source + i - 1);
        }
    }
}

/* compare if 2 data packets are equal. return 1 if true, 0 if false. */
uint8_t bc7215_compare_dpkt(uint8_t sig, const bc7215DataVarPkt_t* pkt1, const bc7215DataVarPkt_t* pkt2)
{
    uint8_t  i;
    uint8_t  bits;
    uint8_t  dat1;
    uint8_t  dat2;
    uint16_t i16;
    uint16_t len;

    if (pkt1->bitLen != pkt2->bitLen)
    {
        return 0;
    }
    len = pkt1->bitLen / 8;
    bits = pkt1->bitLen & 0x07;
    for (i16 = 0; i16 < len; ++i16)
    {
        if (pkt1->data[i16] != pkt2->data[i16])
        {
            return 0;
        }
    }
    if (bits != 0) /* if the end of data is not a complete byte */
    {
        dat1 = pkt1->data[len];
        dat2 = pkt2->data[len];
        if (dat1 == dat2)
        {
            return 1;
        }
        if ((sig & 0x30) == 0x30) /* TP0:TP1 = 11, if data is MSB first (PWM) */
        {
            for (i = 0; i < bits; ++i)
            {
                if ((dat1 & 0x01) != (dat2 & 0x01))
                {
                    return 0;
                }
                dat1 >>= 1;
                dat2 >>= 1;
            }
        }
        else /* if data is LSB first (PPM) */
        {
            for (i = 0; i < bits; ++i)
            {
                if ((dat1 & 0x80) != (dat2 & 0x80))
                {
                    return 0;
                }
                dat1 <<= 1;
                dat2 <<= 1;
            }
        }
    }
    return 1;
}
