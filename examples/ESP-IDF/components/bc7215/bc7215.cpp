#include "bc7215.hpp"

#include <algorithm>

#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"

// This wrapper relies on BC7215 BUSY being connected to the ESP32 UART CTS pin.
// Keep this enabled so the ESP32 UART hardware automatically waits while the
// BC7215 chip is busy and not ready to accept another byte.
#if TX_HW_FLOW_CONTROL != 1
#    error "BC7215 requires TX_HW_FLOW_CONTROL to be 1 in bc7215_lib_config.h"
#endif

// In this ESP-IDF wrapper, UART RX is handled by our own FreeRTOS polling task,
// not by the interrupt mode inside the original C library.
#if USE_UART_INTERRUPT != 0
#    error "BC7215 RX task mode requires USE_UART_INTERRUPT to be 0 in bc7215_lib_config.h"
#endif

// The original C library must be configured to process received UART bytes in a
// separate context.  The rx_task_loop_() below provides that context.
#if PROCESS_IN_DIFFERENT_THREAD != 1
#    error "BC7215 RX task mode requires PROCESS_IN_DIFFERENT_THREAD to be 1 in bc7215_lib_config.h"
#endif

namespace
{
constexpr const char* TAG = "BC7215";

// Kept here as a named timeout value for future expansion.  At present byte
// sending is delegated to uart_write_bytes(), and CTS controls the actual timing.
constexpr TickType_t kSendByteTxDoneTimeout = portMAX_DELAY;
}        // namespace

namespace bc7215
{

// The underlying C library stores only callback function pointers.  Static
// callbacks therefore use active_ to locate the C++ object that owns the UART,
// MOD GPIO and mutex.  This is also why only one active instance is supported.
BC7215* BC7215::active_ = nullptr;

BC7215::BC7215(uart_port_t uart_num, gpio_num_t esp32_tx_pin, gpio_num_t esp32_rx_pin, gpio_num_t esp32_cts_pin, gpio_num_t mod_pin)
    : uart_num_(uart_num)
    , esp32_tx_pin_(esp32_tx_pin)
    , esp32_rx_pin_(esp32_rx_pin)
    , esp32_cts_pin_(esp32_cts_pin)
    , mod_pin_(mod_pin)
{
    // Only store configuration here.  Do not touch ESP-IDF drivers before begin().
}

BC7215::~BC7215()
{
    // Make destruction safe even if the caller forgot to call end().
    end();
}

esp_err_t BC7215::begin(uint32_t rx_poll_ms, int rx_buffer_size, int tx_buffer_size, uint32_t rx_task_stack_size,
    UBaseType_t rx_task_priority)
{
    // Allow repeated begin() calls after successful initialization.
    if (started_)
    {
        return ESP_OK;
    }

    // bc7215_lib callbacks are global/static, so a second active wrapper would
    // make callbacks ambiguous.  Refuse it explicitly instead of failing later.
    if (active_ != nullptr && active_ != this)
    {
        ESP_LOGE(TAG, "bc7215_lib uses static state; only one active BC7215 is supported");
        return ESP_ERR_INVALID_STATE;
    }

    // MOD must be output-capable.  BUSY/CTS only needs to be a valid GPIO input.
    if (!GPIO_IS_VALID_OUTPUT_GPIO(mod_pin_) || !GPIO_IS_VALID_GPIO(esp32_cts_pin_))
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t     err = ESP_OK;
    gpio_config_t mod_conf = {};
    uart_config_t uart_conf = {};

    // Register this instance before installing callbacks, because callback
    // execution depends on active_.
    active_ = this;
    rx_poll_ms_ = (rx_poll_ms == 0) ? 10 : rx_poll_ms;

    // The original C library can call lock/unlock callbacks while processing
    // commands and RX data.  Use a FreeRTOS mutex to serialize those sections.
    mutex_ = xSemaphoreCreateMutex();
    if (mutex_ == nullptr)
    {
        active_ = nullptr;
        return ESP_ERR_NO_MEM;
    }

    // MOD is driven by the BC7215 C library through callbacks.  Configure it as
    // input/output so the library can both drive and read back the level.
    mod_conf.pin_bit_mask = 1ULL << static_cast<uint32_t>(mod_pin_);
    mod_conf.mode = GPIO_MODE_INPUT_OUTPUT;
    mod_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    mod_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    mod_conf.intr_type = GPIO_INTR_DISABLE;

    err = gpio_config(&mod_conf);
    if (err != ESP_OK)
    {
        goto fail;
    }

    // Default to TX/encoder mode so MOD is not left floating during startup.
    gpio_set_level(mod_pin_, 0);

    // BC7215 serial format: 19200 bps by default, 8 data bits, no parity, 2 stop
    // bits.  CTS is connected to BC7215 BUSY for hardware flow control.
    uart_conf.baud_rate = 19200;
    uart_conf.data_bits = UART_DATA_8_BITS;
    uart_conf.parity = UART_PARITY_DISABLE;
    uart_conf.stop_bits = UART_STOP_BITS_2;
    uart_conf.flow_ctrl = UART_HW_FLOWCTRL_CTS;
    uart_conf.rx_flow_ctrl_thresh = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    uart_conf.source_clk = UART_SCLK_DEFAULT;
#endif

    // Install the UART driver without an ESP-IDF event queue.  RX is handled by
    // rx_task_loop_(), which polls uart_get_buffered_data_len().
    void load_format(const bc7215FormatPkt_t* source);
    err = uart_driver_install(uart_num_, rx_buffer_size, tx_buffer_size, 0, nullptr, 0);
    if (err != ESP_OK)
    {
        goto fail;
    }
    uart_driver_installed_ = true;

    err = uart_param_config(uart_num_, &uart_conf);
    if (err != ESP_OK)
    {
        goto fail;
    }

    // Map TX/RX/CTS pins to the selected UART.  RTS is not used.
    err = uart_set_pin(uart_num_, gpio_to_uart_pin_(esp32_tx_pin_), gpio_to_uart_pin_(esp32_rx_pin_), UART_PIN_NO_CHANGE,
        gpio_to_uart_pin_(esp32_cts_pin_));
    if (err != ESP_OK)
    {
        goto fail;
    }

    // BUSY is active-high, so pulldown keeps the CTS input idle during early
    // bring-up or if the external line is temporarily floating.
    gpio_set_pull_mode(esp32_cts_pin_, GPIO_PULLDOWN_ONLY);
    uart_flush_input(uart_num_);

    // Connect platform-dependent operations expected by the original C library.
    // After this, bc7215_lib can send bytes, switch MOD, read MOD and lock its
    // critical sections without knowing anything about ESP-IDF.
    bc7215_config_uart_send_byte(&BC7215::uart_send_byte_cb_);
    bc7215_config_mod_set_high_funct(&BC7215::mod_set_high_cb_);
    bc7215_config_mod_set_low_funct(&BC7215::mod_set_low_cb_);
    bc7215_config_read_mod_funct(&BC7215::read_mod_cb_);
    bc7215_config_mutex_lock_funct(&BC7215::mutex_lock_cb_);
    bc7215_config_mutex_unlock_funct(&BC7215::mutex_unlock_cb_);

    // Start the polling task that moves bytes from ESP-IDF's UART RX buffer into
    // bc7215_process_uart_data().
    rx_task_running_ = true;
    if (xTaskCreate(&BC7215::rx_task_trampoline_, "bc7215_rx", rx_task_stack_size, this, rx_task_priority, &rx_task_)
        != pdPASS)
    {
        rx_task_running_ = false;
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    started_ = true;
    return ESP_OK;

fail:
    // Roll back every resource that may have been created before the failure.
    // This keeps begin() failure paths safe and allows the caller to retry.
    ESP_LOGE(TAG, "begin failed: %s", esp_err_to_name(err));
    rx_task_running_ = false;
    if (rx_task_ != nullptr)
    {
        vTaskDelete(rx_task_);
        rx_task_ = nullptr;
    }
    if (uart_driver_installed_)
    {
        uart_driver_delete(uart_num_);
        uart_driver_installed_ = false;
    }
    if (mutex_ != nullptr)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
    if (active_ == this)
    {
        active_ = nullptr;
    }
    return err;
}

void BC7215::end()
{
    // Nothing to release if this object was never started and is not registered
    // as the active callback target.
    if (!started_ && active_ != this)
    {
        return;
    }

    // Ask the RX task to exit.  Give it a short chance to delete itself before
    // force deleting it, so normal shutdown does not leave a dangling task handle.
    rx_task_running_ = false;
    if (rx_task_ != nullptr)
    {
        for (int i = 0; i < 20 && rx_task_ != nullptr; ++i)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (rx_task_ != nullptr)
        {
            vTaskDelete(rx_task_);
            rx_task_ = nullptr;
        }
    }

    // Wait briefly for queued TX data to leave the UART before deleting the driver.
    void load_format(const bc7215FormatPkt_t* source);
    if (uart_driver_installed_)
    {
        uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(100));
        uart_driver_delete(uart_num_);
        uart_driver_installed_ = false;
    }

    if (mutex_ != nullptr)
    {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }

    // Return MOD pin to its reset state so later code can reconfigure it cleanly.
    if (GPIO_IS_VALID_GPIO(mod_pin_))
    {
        gpio_reset_pin(mod_pin_);
    }

    if (active_ == this)
    {
        active_ = nullptr;
    }
    started_ = false;
}

void BC7215::set_rx()
{
    // Switch BC7215 to receive / learning mode.
    bc7215_set_rx();
}

void BC7215::set_tx()
{
    // Switch BC7215 to transmit / encoder mode.
    bc7215_set_tx();
}

void BC7215::shutdown() { bc7215_set_shutdown(); }

void BC7215::set_rx_mode(uint8_t mode)
{
    // The mode value is defined by the original BC7215 C library/protocol.
    bc7215_set_rx_mode(mode);
}

bool BC7215::is_busy() const { return bc7215_is_busy() != 0; }

bool BC7215::cmd_completed() const { return bc7215_cmd_completed() != 0; }

#if ENABLE_RECEIVING == 1
bool BC7215::data_ready() const { return bc7215_data_ready() != 0; }

void BC7215::clr_data() { bc7215_clr_data(); }

uint16_t BC7215::get_len() const { return bc7215_get_len(); }

uint16_t BC7215::data_packet_size() const { return bc7215_dpkt_size(); }

uint8_t BC7215::get_data(bc7215DataVarPkt_t* target)
{
    // Caller must provide a buffer large enough for the decoded variable-length packet.
    return bc7215_get_data(target);
}

uint8_t BC7215::get_data(bc7215DataMaxPkt_t& target)
{
    // bc7215DataMaxPkt_t has the same packet header layout as bc7215DataVarPkt_t,
    // but reserves maximum storage for payload bytes.
    return bc7215_get_data(reinterpret_cast<bc7215DataVarPkt_t*>(&target));
}

uint16_t BC7215::get_raw(void* addr, uint16_t size)
{
#    if BC7215_MAX_RX_DATA_SIZE >= 256
    return bc7215_get_raw(addr, size);
#    else
    // Some bc7215_lib configurations use an 8-bit length parameter for raw data.
    // Clamp the request so the cast below cannot wrap around.
    if (size > 0xFF)
    {
        size = 0xFF;
    }
    return bc7215_get_raw(addr, static_cast<uint8_t>(size));
#    endif
}

#    if ENABLE_FORMAT == 1
bool BC7215::format_ready() const { return bc7215_format_ready() != 0; }

void BC7215::clr_format() { bc7215_clr_format(); }

uint8_t BC7215::get_format(bc7215FormatPkt_t* target) { return bc7215_get_format(target); }

uint8_t BC7215::get_format(bc7215FormatPkt_t& target) { return bc7215_get_format(&target); }
#    endif
#endif

#if ENABLE_TRANSMITTING == 1
void BC7215::load_format(const bc7215FormatPkt_t* source)
{
    // Load timing/protocol format before transmitting a matching data packet.
    bc7215_load_format(source);
}

void BC7215::load_format(const bc7215FormatPkt_t& source) { bc7215_load_format(&source); }

void BC7215::ir_tx(const bc7215DataVarPkt_t* source)
{
    // Transmit a variable-length BC7215 data packet using the currently loaded format.
    bc7215_IR_tx(source);
}

void BC7215::ir_tx(const bc7215DataMaxPkt_t& source)
{
    // Reinterpret the max-size storage as the variable-size packet layout expected
    // by the C API.  The actual bitLen field determines the real payload length.
    bc7215_IR_tx(reinterpret_cast<const bc7215DataVarPkt_t*>(&source));
}

void BC7215::send_raw(const void* raw_data, uint16_t size)
{
    // Send raw bytes directly through the BC7215 library's raw-send path.
    bc7215_send_raw(raw_data, size);
}
#endif

uint16_t BC7215::calc_size(const bc7215DataVarPkt_t* data_pkt) const { return bc7215_cal_size(data_pkt); }

uint8_t BC7215::crc8(const uint8_t* data, uint16_t len) const
{
#if BC7215_MAX_RX_DATA_SIZE < 256
    // Match the C API's smaller length type when the library is configured with
    // a maximum RX size below 256 bytes.
    if (len > 0xFF)
    {
        len = 0xFF;
    }
    return bc7215_crc8(data, static_cast<uint8_t>(len));
#else
    return bc7215_crc8(data, len);
#endif
}

int BC7215::gpio_to_uart_pin_(gpio_num_t pin)
{
    // ESP-IDF uses UART_PIN_NO_CHANGE instead of GPIO_NUM_NC in uart_set_pin().
    return (pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : static_cast<int>(pin);
}

void BC7215::rx_task_trampoline_(void* arg)
{
    // FreeRTOS C task entry point -> C++ member function.
    static_cast<BC7215*>(arg)->rx_task_loop_();
}

void BC7215::rx_task_loop_()
{
    uint8_t rx_buf[kRxReadChunkSize];

    while (rx_task_running_)
    {
        size_t buffered_len = 0;

        // Instead of blocking on uart_read_bytes(), first check how many bytes
        // ESP-IDF already has.  This keeps the task simple and avoids an event queue.
        if (uart_get_buffered_data_len(uart_num_, &buffered_len) == ESP_OK && buffered_len > 0)
        {
            while (rx_task_running_ && buffered_len > 0)
            {
                const size_t to_read = std::min(buffered_len, sizeof(rx_buf));
                const int    read_len = uart_read_bytes(uart_num_, rx_buf, to_read, 0);
                if (read_len <= 0)
                {
                    break;
                }

                // Feed every byte into the original BC7215 parser/state machine.
                // This is the equivalent of an interrupt byte-receive callback.
                for (int i = 0; i < read_len; ++i)
                {
                    bc7215_process_uart_data(rx_buf[i]);
                }

                // More data may have arrived while we were processing this chunk.
                buffered_len = 0;
                if (uart_get_buffered_data_len(uart_num_, &buffered_len) != ESP_OK)
                {
                    break;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(rx_poll_ms_));
    }

    // Mark the task handle invalid before self-deleting.  end() watches this
    // handle to know whether the task has exited by itself.
    rx_task_ = nullptr;
    vTaskDelete(nullptr);
}

void BC7215::uart_send_byte_cb_(uint8_t data)
{
    BC7215* self = active_;
    if (self == nullptr || !self->started_)
    {
        return;
    }

    // Called by bc7215_lib whenever it wants to send one byte.  The ESP32 UART
    // driver and CTS hardware flow control handle the actual output timing.
    const uint8_t byte = data;
    uart_write_bytes(self->uart_num_, &byte, 1);
}

void BC7215::mod_set_high_cb_()
{
    if (active_ != nullptr)
    {
        gpio_set_level(active_->mod_pin_, 1);
    }
}

void BC7215::mod_set_low_cb_()
{
    if (active_ != nullptr)
    {
        gpio_set_level(active_->mod_pin_, 0);
    }
}

uint8_t BC7215::read_mod_cb_()
{
    if (active_ == nullptr)
    {
        return 0;
    }
    return gpio_get_level(active_->mod_pin_) ? 1 : 0;
}

void BC7215::mutex_lock_cb_()
{
    if (active_ != nullptr && active_->mutex_ != nullptr)
    {
        xSemaphoreTake(active_->mutex_, portMAX_DELAY);
    }
}

void BC7215::mutex_unlock_cb_()
{
    if (active_ != nullptr && active_->mutex_ != nullptr)
    {
        xSemaphoreGive(active_->mutex_);
    }
}

}        // namespace bc7215
