#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// The original BC7215 C driver is written in C.  Keep it inside extern "C"
// so this C++ wrapper can link against the C symbols without name mangling.
extern "C"
{
#include "bc7215_lib.h"
}

namespace bc7215
{

/**
 * @brief ESP-IDF C++ wrapper for the BC7215 / BC7215A low-level C driver.
 *
 * This class is responsible for:
 *   - configuring the ESP32 UART used to communicate with BC7215;
 *   - configuring the MOD GPIO used by the BC7215 library to switch modes;
 *   - connecting the C library callback functions to ESP-IDF UART/GPIO APIs;
 *   - running a small RX polling task that feeds received UART bytes into the C library;
 *   - providing a C++-friendly interface around the original bc7215_* functions.
 *
 * Important limitation:
 * The underlying bc7215_lib uses global/static callback state, so this wrapper
 * intentionally supports only one active BC7215 object at a time.
 */
class BC7215
{
public:
    /**
     * @brief Construct a BC7215 UART/GPIO wrapper.
     *
     * The constructor only stores pin and UART parameters.  Hardware is not
     * configured until begin() is called, so construction is safe even before
     * the ESP-IDF scheduler and drivers are fully ready.
     *
     * @param uart_num UART port used for BC7215 communication.
     * @param tx_pin ESP32 TX pin connected to BC7215 RX.
     * @param rx_pin ESP32 RX pin connected to BC7215 TX.
     * @param cts_busy_pin ESP32 CTS input connected to BC7215 BUSY output.
     * @param mod_pin ESP32 GPIO connected to BC7215 MOD pin.
     */
    BC7215(uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t cts_busy_pin, gpio_num_t mod_pin);

    /**
     * @brief Stop the driver and release ESP-IDF resources.
     *
     * The destructor calls end(), so an explicitly created object will clean up
     * its UART driver, RX task and mutex when destroyed.
     */
    ~BC7215();

    // The wrapper owns hardware resources and registers global callbacks, so it
    // must not be copied or assigned.
    BC7215(const BC7215&) = delete;
    BC7215& operator=(const BC7215&) = delete;

    /**
     * @brief Initialize GPIO, UART, callbacks and the RX polling task.
     *
     * UART is configured as 8 data bits, no parity, 2 stop bits, with CTS flow
     * control enabled.  BC7215 BUSY is connected to ESP32 CTS so the ESP UART
     * hardware automatically pauses transmission while BC7215 is busy.
     *
     * @param rx_poll_ms Polling interval used by the RX task. 0 is treated as 10 ms.
     * @param rx_buffer_size ESP-IDF UART RX buffer size in bytes.
     * @param tx_buffer_size ESP-IDF UART TX buffer size in bytes.
     * @param rx_task_stack_size FreeRTOS stack size for the RX polling task.
     * @param rx_task_priority FreeRTOS priority for the RX polling task.
     * @return ESP_OK on success, otherwise an ESP-IDF error code.
     */
    esp_err_t begin(uint32_t rx_poll_ms = 10, int rx_buffer_size = 256, int tx_buffer_size = 256,
        uint32_t rx_task_stack_size = 4096, UBaseType_t rx_task_priority = 10);

    /**
     * @brief Stop the RX task, delete the UART driver and reset owned resources.
     */
    void end();

    /** @brief Return true after begin() has completed successfully. */
    bool started() const { return started_; }

    // Mode control helpers.  These directly call the corresponding bc7215_lib
    // functions and are kept small on purpose.
    void set_rx();                         ///< Put BC7215 into RX/learning mode.
    void set_tx();                         ///< Put BC7215 into TX/encoding mode.
    void shutdown();                       ///< Put BC7215 into shutdown mode.
    void set_rx_mode(uint8_t mode);        ///< Select BC7215 receive/decoding mode.

    bool is_busy() const;              ///< Read BC7215 busy status from the C library.
    bool cmd_completed() const;        ///< Check whether the current BC7215 command has completed.

#if ENABLE_RECEIVING == 1
    // Receive-side helpers.  These are compiled only when receiving is enabled
    // in bc7215_lib_config.h.
    bool     data_ready() const;                          ///< True when decoded IR data is ready.
    void     clr_data();                                  ///< Clear the decoded data-ready flag/buffer.
    uint16_t get_len() const;                             ///< Length of the most recently received data.
    uint16_t data_packet_size() const;                    ///< Size of the decoded BC7215 data packet.
    uint8_t  get_data(bc7215DataVarPkt_t* target);        ///< Copy decoded data into a variable-size packet buffer.
    uint8_t  get_data(bc7215DataMaxPkt_t& target);        ///< Copy decoded data into a max-size packet buffer.
    uint16_t get_raw(void* addr, uint16_t size);          ///< Copy raw received bytes into caller buffer.

#    if ENABLE_FORMAT == 1
    // Format-packet helpers.  Format data describes how the IR waveform should
    // be interpreted/transmitted and is often saved together with data packets.
    bool    format_ready() const;
    void    clr_format();
    uint8_t get_format(bc7215FormatPkt_t* target);
    uint8_t get_format(bc7215FormatPkt_t& target);
#    endif
#endif

#if ENABLE_TRANSMITTING == 1
    // Transmit-side helpers.  The BC7215 library uses a format packet plus a
    // data packet to reproduce the learned IR signal.
    void load_format(const bc7215FormatPkt_t* source);
    void load_format(const bc7215FormatPkt_t& source);
    void ir_tx(const bc7215DataVarPkt_t* source);
    void ir_tx(const bc7215DataMaxPkt_t& source);
    void send_raw(const void* raw_data, uint16_t size);
#endif

    uint16_t calc_size(const bc7215DataVarPkt_t* data_pkt) const;        ///< Calculate data packet storage size.
    uint8_t  crc8(const uint8_t* data, uint16_t len) const;              ///< Calculate BC7215 CRC8.

private:
    static constexpr int    kBusyActiveLevel = 1;         ///< BC7215 BUSY is active high.
    static constexpr size_t kRxReadChunkSize = 32;        ///< Max bytes read from UART in one RX-task iteration.

    // Pointer used by static C callbacks to reach the current C++ object.
    // Because the C driver stores only function pointers, this wrapper supports
    // one active instance at a time.
    static BC7215* active_;

    // Hardware mapping configured by the constructor.
    uart_port_t uart_num_;
    gpio_num_t  tx_pin_;
    gpio_num_t  rx_pin_;
    gpio_num_t  cts_busy_pin_;
    gpio_num_t  mod_pin_;

    // Runtime resources created by begin() and released by end().
    SemaphoreHandle_t mutex_ = nullptr;
    TaskHandle_t      rx_task_ = nullptr;
    volatile bool     rx_task_running_ = false;
    bool              started_ = false;
    bool              uart_driver_installed_ = false;
    uint32_t          rx_poll_ms_ = 10;

    // Convert GPIO_NUM_NC to ESP-IDF's UART_PIN_NO_CHANGE, otherwise return the
    // numeric GPIO value expected by uart_set_pin().
    static int gpio_to_uart_pin_(gpio_num_t pin);

    // FreeRTOS task entry must be static; it forwards to rx_task_loop_().
    static void rx_task_trampoline_(void* arg);
    void        rx_task_loop_();

    // Static callbacks registered into bc7215_lib.  They bridge the C driver's
    // platform hooks to ESP-IDF UART, GPIO and FreeRTOS mutex APIs.
    static void    uart_send_byte_cb_(uint8_t data);
    static void    mod_set_high_cb_();
    static void    mod_set_low_cb_();
    static uint8_t read_mod_cb_();
    static void    mutex_lock_cb_();
    static void    mutex_unlock_cb_();
};

}        // namespace bc7215
