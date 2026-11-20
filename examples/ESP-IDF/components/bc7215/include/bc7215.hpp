#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

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
 *   - providing a synchronous RX polling method (poll_rx) instead of a background task;
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
     */
    BC7215(uart_port_t uart_num, gpio_num_t esp32_tx_pin, gpio_num_t esp32_rx_pin, gpio_num_t esp32_cts_pin, gpio_num_t mod_pin);

    /**
     * @brief Stop the driver and release ESP-IDF resources.
     */
    ~BC7215();

    // The wrapper owns hardware resources and registers global callbacks, so it
    // must not be copied or assigned.
    BC7215(const BC7215&) = delete;
    BC7215& operator=(const BC7215&) = delete;

    /**
     * @brief Initialize GPIO, UART, and callbacks.
     *
     * @param rx_buffer_size ESP-IDF UART RX buffer size in bytes.
     * @param tx_buffer_size ESP-IDF UART TX buffer size in bytes.
     * @return ESP_OK on success, otherwise an ESP-IDF error code.
     */
    esp_err_t begin(int rx_buffer_size = 256, int tx_buffer_size = 256);

    /**
     * @brief Delete the UART driver and reset owned resources.
     */
    void end();

    /** @brief Return true after begin() has completed successfully. */
    bool started() const { return started_; }

    void set_rx();
    void set_tx();
    void shutdown();
    void set_rx_mode(uint8_t mode);

    bool is_busy();
    bool cmd_completed();

#if ENABLE_RECEIVING == 1
    bool     data_ready();
    void     clr_data();
    uint16_t get_len() const;
    uint16_t data_packet_size() const;
    uint8_t  get_data(bc7215DataVarPkt_t* target);
    uint8_t  get_data(bc7215DataMaxPkt_t& target);
    uint16_t get_raw(void* addr, uint16_t size);

#    if ENABLE_FORMAT == 1
    bool    format_ready();
    void    clr_format();
    uint8_t get_format(bc7215FormatPkt_t* target);
    uint8_t get_format(bc7215FormatPkt_t& target);
#    endif
#endif

#if ENABLE_TRANSMITTING == 1
    void load_format(const bc7215FormatPkt_t* source);
    void load_format(const bc7215FormatPkt_t& source);
    void ir_tx(const bc7215DataVarPkt_t* source);
    void ir_tx(const bc7215DataMaxPkt_t& source);
    void send_raw(const void* raw_data, uint16_t size);
#endif

    uint16_t calc_size(const bc7215DataVarPkt_t* data_pkt) const;
    uint8_t  crc8(const uint8_t* data, uint16_t len) const;

private:
    static constexpr int kBusyActiveLevel = 1;

    static BC7215* active_;

    // Hardware mapping configured by the constructor.
    uart_port_t uart_num_;
    gpio_num_t  esp32_tx_pin_;
    gpio_num_t  esp32_rx_pin_;
    gpio_num_t  esp32_cts_pin_;
    gpio_num_t  mod_pin_;

    bool        started_ = false;
    bool        uart_driver_installed_ = false;

    /**
     * @brief Synchronously poll the UART buffer and feed bytes into the BC7215 C decoder.
     * Returns immediately if an IR data packet or format packet completes.
     */
    void poll_rx();

    static int gpio_to_uart_pin_(gpio_num_t pin);

    static void    uart_send_byte_cb_(uint8_t data);
    static void    mod_set_high_cb_();
    static void    mod_set_low_cb_();
    static uint8_t read_mod_cb_();
};

}        // namespace bc7215
