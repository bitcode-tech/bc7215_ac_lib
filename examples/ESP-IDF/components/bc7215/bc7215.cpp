#include "bc7215.hpp"

#include <algorithm>

#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_timer.h"

#if TX_HW_FLOW_CONTROL != 1
#    error "BC7215 requires TX_HW_FLOW_CONTROL to be 1 in bc7215_lib_config.h"
#endif

#if USE_UART_INTERRUPT != 0
#    error "BC7215 RX task mode requires USE_UART_INTERRUPT to be 0 in bc7215_lib_config.h"
#endif

// Now strictly using single-threaded manual polling
#if PROCESS_IN_DIFFERENT_THREAD != 0
#    error "BC7215 manual polling requires PROCESS_IN_DIFFERENT_THREAD to be 0 in bc7215_lib_config.h"
#endif

namespace
{
constexpr const char* TAG = "BC7215";
}        // namespace

namespace bc7215
{

BC7215* BC7215::active_ = nullptr;

BC7215::BC7215(uart_port_t uart_num, gpio_num_t esp32_tx_pin, gpio_num_t esp32_rx_pin, gpio_num_t esp32_cts_pin, gpio_num_t mod_pin)
    : uart_num_(uart_num)
    , esp32_tx_pin_(esp32_tx_pin)
    , esp32_rx_pin_(esp32_rx_pin)
    , esp32_cts_pin_(esp32_cts_pin)
    , mod_pin_(mod_pin)
{
}

BC7215::~BC7215()
{
    end();
}

esp_err_t BC7215::begin(int rx_buffer_size, int tx_buffer_size)
{
    if (started_)
    {
        return ESP_OK;
    }

    if (active_ != nullptr && active_ != this)
    {
        ESP_LOGE(TAG, "bc7215_lib uses static state; only one active BC7215 is supported");
        return ESP_ERR_INVALID_STATE;
    }

    if (!GPIO_IS_VALID_OUTPUT_GPIO(mod_pin_) || !GPIO_IS_VALID_GPIO(esp32_cts_pin_))
    {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t     err = ESP_OK;
    gpio_config_t mod_conf = {};
    uart_config_t uart_conf = {};

    active_ = this;

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

    gpio_set_level(mod_pin_, 0);

    uart_conf.baud_rate = 19200;
    uart_conf.data_bits = UART_DATA_8_BITS;
    uart_conf.parity = UART_PARITY_DISABLE;
    uart_conf.stop_bits = UART_STOP_BITS_2;
    uart_conf.flow_ctrl = UART_HW_FLOWCTRL_CTS;
    uart_conf.rx_flow_ctrl_thresh = 0;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    uart_conf.source_clk = UART_SCLK_DEFAULT;
#endif

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

    err = uart_set_pin(uart_num_, gpio_to_uart_pin_(esp32_tx_pin_), gpio_to_uart_pin_(esp32_rx_pin_), UART_PIN_NO_CHANGE,
        gpio_to_uart_pin_(esp32_cts_pin_));
    if (err != ESP_OK)
    {
        goto fail;
    }

    gpio_set_pull_mode(esp32_cts_pin_, GPIO_PULLDOWN_ONLY);
    uart_flush_input(uart_num_);

    bc7215_config_uart_send_byte(&BC7215::uart_send_byte_cb_);
    bc7215_config_mod_set_high_funct(&BC7215::mod_set_high_cb_);
    bc7215_config_mod_set_low_funct(&BC7215::mod_set_low_cb_);
    bc7215_config_read_mod_funct(&BC7215::read_mod_cb_);

    started_ = true;
    return ESP_OK;

fail:
    ESP_LOGE(TAG, "begin failed: %s", esp_err_to_name(err));
    if (uart_driver_installed_)
    {
        uart_driver_delete(uart_num_);
        uart_driver_installed_ = false;
    }
    if (active_ == this)
    {
        active_ = nullptr;
    }
    return err;
}

void BC7215::end()
{
    if (!started_ && active_ != this)
    {
        return;
    }

    if (uart_driver_installed_)
    {
        uart_wait_tx_done(uart_num_, pdMS_TO_TICKS(100));
        uart_driver_delete(uart_num_);
        uart_driver_installed_ = false;
    }

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

void BC7215::poll_rx()
{
    if (!started_ || !uart_driver_installed_)
    {
        return;
    }

    uint8_t rx_byte = 0;
    while (true)
    {
        // Zero timeout makes uart_read_bytes non-blocking
        const int read_len = uart_read_bytes(uart_num_, &rx_byte, 1, 0);
        
        if (read_len == 1)
        {
            bc7215_process_uart_data(rx_byte);

            bool ready = false;
#if ENABLE_RECEIVING == 1
            if (bc7215_data_ready())
            {
                ready = true;
            }
# if ENABLE_FORMAT == 1
            if (bc7215_format_ready())
            {
                ready = true;
            }
# endif
#endif
            // Break out early if a full packet was parsed. The rest stay in UART ringbuffer.
            if (ready)
            {
                return;
            }
        }
        else
        {
            // Empty buffer (read_len <= 0)
            break;
        }
    }
}

void BC7215::set_rx() { bc7215_set_rx(); }
void BC7215::set_tx() { bc7215_set_tx(); }
void BC7215::shutdown() { bc7215_set_shutdown(); }
void BC7215::set_rx_mode(uint8_t mode) { bc7215_set_rx_mode(mode); }

bool BC7215::is_busy()
{
	poll_rx();
	return bc7215_is_busy() != 0; 
}

bool BC7215::cmd_completed() 
{
	poll_rx();
	return bc7215_cmd_completed() != 0; 
}

#if ENABLE_RECEIVING == 1
bool BC7215::data_ready() 
{
	poll_rx();
	return bc7215_data_ready() != 0; 
}

void BC7215::clr_data() { bc7215_clr_data(); }
uint16_t BC7215::get_len() const { return bc7215_get_len(); }
uint16_t BC7215::data_packet_size() const { return bc7215_dpkt_size(); }
uint8_t BC7215::get_data(bc7215DataVarPkt_t* target) { return bc7215_get_data(target); }
uint8_t BC7215::get_data(bc7215DataMaxPkt_t& target) { return bc7215_get_data(reinterpret_cast<bc7215DataVarPkt_t*>(&target)); }

uint16_t BC7215::get_raw(void* addr, uint16_t size)
{
#    if BC7215_MAX_RX_DATA_SIZE >= 256
    return bc7215_get_raw(addr, size);
#    else
    if (size > 0xFF)
    {
        size = 0xFF;
    }
    return bc7215_get_raw(addr, static_cast<uint8_t>(size));
#    endif
}

#    if ENABLE_FORMAT == 1
bool BC7215::format_ready() 
{
	poll_rx();
	return bc7215_format_ready() != 0; 
}

void BC7215::clr_format() { bc7215_clr_format(); }
uint8_t BC7215::get_format(bc7215FormatPkt_t* target) { return bc7215_get_format(target); }
uint8_t BC7215::get_format(bc7215FormatPkt_t& target) { return bc7215_get_format(&target); }
#    endif
#endif

#if ENABLE_TRANSMITTING == 1
void BC7215::load_format(const bc7215FormatPkt_t* source) { bc7215_load_format(source); }
void BC7215::load_format(const bc7215FormatPkt_t& source) { bc7215_load_format(&source); }
void BC7215::ir_tx(const bc7215DataVarPkt_t* source) { bc7215_IR_tx(source); }
void BC7215::ir_tx(const bc7215DataMaxPkt_t& source) { bc7215_IR_tx(reinterpret_cast<const bc7215DataVarPkt_t*>(&source)); }
void BC7215::send_raw(const void* raw_data, uint16_t size) { bc7215_send_raw(raw_data, size); }
#endif

uint16_t BC7215::calc_size(const bc7215DataVarPkt_t* data_pkt) const { return bc7215_cal_size(data_pkt); }

uint8_t BC7215::crc8(const uint8_t* data, uint16_t len) const
{
#if BC7215_MAX_RX_DATA_SIZE < 256
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
    return (pin == GPIO_NUM_NC) ? UART_PIN_NO_CHANGE : static_cast<int>(pin);
}

void BC7215::uart_send_byte_cb_(uint8_t data)
{
    BC7215* self = active_;
    if (self == nullptr || !self->started_)
    {
        return;
    }
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

}        // namespace bc7215
