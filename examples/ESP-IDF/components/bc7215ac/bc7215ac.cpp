#include "bc7215ac.hpp"

// Implementation of the high-level AC controller wrapper.
// The low-level BC7215 class only knows how to capture/transmit IR packets;
// this file adds AC protocol matching, command generation, predefined data,
// and parsing of AC remote-control frames.

#include <algorithm>
#include <cstring>

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// This wrapper needs receive, format, and transmit support from the underlying
// C library. Fail at compile time if the library configuration is incompatible.
#if ENABLE_RECEIVING != 1
#    error "BC7215AC requires ENABLE_RECEIVING to be 1 in bc7215_lib_config.h"
#endif

#if ENABLE_FORMAT != 1
#    error "BC7215AC requires ENABLE_FORMAT to be 1 in bc7215_lib_config.h"
#endif

#if ENABLE_TRANSMITTING != 1
#    error "BC7215AC requires ENABLE_TRANSMITTING to be 1 in bc7215_lib_config.h"
#endif

namespace bc7215
{

// Store the hardware pin/UART configuration by constructing the embedded
// low-level driver. No hardware is touched until begin() is called.
BC7215AC::BC7215AC(
    uart_port_t uart_num, gpio_num_t esp32_tx_pin, gpio_num_t esp32_rx_pin, gpio_num_t esp32_cts_pin, gpio_num_t mod_pin)
    : bc7215_(uart_num, esp32_tx_pin, esp32_rx_pin, esp32_cts_pin, mod_pin)
{
}

// Initialize the low-level BC7215 driver and reset AC-wrapper runtime state.
// After begin(), the chip is left in TX mode so it is ready to send commands.
esp_err_t BC7215AC::begin()
{
    // Skip initialization if the wrapper has already been marked as started.
    if (started_)
    {
        return ESP_OK;
    }

    // Initialize UART, GPIO, callbacks, mutex, and the RX polling task.
    esp_err_t err = bc7215_.begin();
    if (err != ESP_OK)
    {
        return err;
    }

    // Pairing/protocol state is intentionally reset on startup.
    init_ok = false;
    use_fahrenheit_ = false;
    format_received_ = false;
    timer_start_ms_ = 0;

    if (!bc7215_.started())
    {
        return ESP_ERR_INVALID_STATE;
    }

    // Default to transmit mode after hardware setup. Capture functions will
    // explicitly switch to RX mode when needed.
    bc7215_.set_tx();
    started_ = true;
    return ESP_OK;
}

// Switch to Fahrenheit mode. Changing units invalidates the existing match
// because the AC helper library uses different temperature offsets/tables.
void BC7215AC::set_fahrenheit()
{
    if (!use_fahrenheit_)
    {
        init_ok = false;
        current_temp = 78;
    }
    use_fahrenheit_ = true;
}

// Switch back to Celsius mode and reset the default command temperature.
void BC7215AC::set_celsius()
{
    if (use_fahrenheit_)
    {
        init_ok = false;
        current_temp = 25;
    }
    use_fahrenheit_ = false;
}

// Public query uses positive wording because the internal flag stores only the
// non-default Fahrenheit state.
bool BC7215AC::is_celsius() const { return !use_fahrenheit_; }

// Start an IR capture sequence.
// rx_mode is passed to the BC7215 library: in this project, mode 1 is used for
// AC pairing, while mode 0 is used for normal IR decoding/parsing.
void BC7215AC::start_capture(uint8_t rx_mode)
{
    format_received_ = false;

    // Switch the chip to receive mode and clear any old decoded data before
    // accepting new IR frames.
    bc7215_.set_rx();
    delay_ms_(kModeSwitchDelayMs);
    bc7215_.set_rx_mode(rx_mode);
    bc7215_.clr_data();
    bc7215_.clr_format();
	sample_count = 0;
    timer_start_ms_ = now_ms_();
}

// Stop receiving and return the BC7215 to transmit mode.
void BC7215AC::stop_capture()
{
    format_received_ = false;
    bc7215_.set_tx();
    delay_ms_(kModeSwitchDelayMs);
}

// Polling capture state machine.
// Returns true only after at least one frame has been received and the receiver
// has stayed idle for kCaptureIdleMs, which indicates the current IR burst ended.
bool BC7215AC::signal_captured()
{
    // While BUSY is active, the chip is still receiving/processing a signal.
    // Keep extending the idle timer so we do not finish the capture too early.
    if (bc7215_.is_busy())
    {
        timer_start_ms_ = now_ms_();
    }
    else
    {
        // A complete data packet is available in the low-level receive buffer.
        if (bc7215_.data_ready())
        {
            if (sample_count < kMaxSamples)
            {
                sample_status[sample_count] = bc7215_.get_data(sample_data[sample_count]);
                // Some captures include a separately decoded format packet.
                // Store it and link format+data as one combined message for the AC library.
                if (bc7215_.format_ready())
                {
                    bc7215_.get_format(sample_format[sample_count]);
                    format_received_ = true;
                }
                ++sample_count;
            }
            timer_start_ms_ = now_ms_();
        }
    }

    // Do not report completion until at least one sample is stored.
    if (sample_count > 0)
    {

        if ((now_ms_() - timer_start_ms_) > kCaptureIdleMs)
        {
            return true;
        }
    }

    return false;
}

// Initialize the AC protocol matcher from the samples collected by capture.
// One-sample and multi-sample paths call different C-library entry points.
bool BC7215AC::init()
{
    init_ok = false;

    // A single frame can be initialized directly.
    if (sample_count == 1)
    {
        // For one frame, use the raw sample data directly unless the chip
        // reported a decode error.
        if (sample_status[0] & kErrStatusBit)
        {
            return false;
        }
        if (use_fahrenheit_)
        {
            init_ok = bc7215_ac_init_f(
                sample_status[0], reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
        }
        else
        {
            init_ok
                = bc7215_ac_init(sample_status[0], reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
        }
    }
    else if (sample_count > 1)
    {
        // Multi-frame matching may require bit inversion correction first.
        if (!reverse_marked_samples_())
        {
            return false;
        }

        if (use_fahrenheit_)
        {
            init_ok = bc7215_ac_init2_f(sample_count, received_message_, 0);
        }
        else
        {
            init_ok = bc7215_ac_init2(sample_count, received_message_, 0);
        }
    }

    return init_ok;
}

// Initialize from previously saved packets, e.g. data restored from NVS flash.
// The packets are converted into the same single-sample layout used after capture.
bool BC7215AC::init(const bc7215DataMaxPkt_t& data, const bc7215FormatPkt_t& format)
{
    sample_data[0] = data;
    sample_format[0] = format;
    sample_status[0] = format.signature.inByte;
    sample_count = 1;

    if (use_fahrenheit_)
    {
        init_ok
            = bc7215_ac_init_f(sample_status[0], reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
    }
    else
    {
        init_ok = bc7215_ac_init(sample_status[0], reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
    }

    return init_ok;
}

// Try the next protocol candidate after a successful pairing. This is useful
// when the first decoded match sends IR but does not control the AC correctly.
bool BC7215AC::match_next()
{
    if (init_ok)
    {
        init_ok = bc7215_ac_find_next();
    }
    return init_ok;
}

// Number of predefined AC protocol entries compiled into the C library.
uint8_t BC7215AC::predefined_count() const { return bc7215_ac_predefined_cnt(); }

// Human-readable predefined protocol name for menu display.
const char* BC7215AC::predefined_name(uint8_t index) const
{
    if (index < predefined_count())
    {
        return bc7215_ac_predefined_name(index);
    }
    return nullptr;
}

// Initialize the AC protocol library from a built-in predefined protocol.
bool BC7215AC::init_predefined(uint8_t index)
{
    init_ok = false;

    if (index >= predefined_count())
    {
        return false;
    }

    if (!copy_predefined_data_(index))
    {
        return false;
    }

    if (use_fahrenheit_)
    {
        init_ok
            = bc7215_ac_init_f(sample_status[0], reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
    }
    else
    {
        init_ok = bc7215_ac_init(sample_status[0], reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
    }

    return init_ok;
}

// Generate and immediately transmit a full AC state command.
// The C library expects temperature as an offset from 16C or 60F, so the public
// absolute temperature is converted before calling bc7215_ac_set*.
const bc7215DataVarPkt_t* BC7215AC::set_to(int temp, int mode, int fan, int key)
{
    if (!init_ok)
    {
        return nullptr;
    }

    const bc7215DataVarPkt_t* data_pkt = nullptr;

    // Remember the requested state so future on() fallback can reproduce it.
    current_temp = static_cast<int8_t>(temp);
    current_mode = static_cast<int8_t>(mode);
    current_fan = static_cast<int8_t>(fan);
    current_key = static_cast<int8_t>(key);
	if (use_fahrenheit_)
	{
	    data_pkt = bc7215_ac_set_f(current_temp - 60, current_mode, current_fan, current_key);
	}
	else
	{
	    data_pkt = bc7215_ac_set(current_temp - 16, current_mode, current_fan, current_key);
	}
    send_ac_cmd_(data_pkt);
    return data_pkt;
}

// Send a power-on command. Some protocol tables do not provide a dedicated
// ON packet; in that case, resend the last full state command as a fallback.
const bc7215DataVarPkt_t* BC7215AC::on()
{
    if (!init_ok)
    {
        return nullptr;
    }

    const bc7215DataVarPkt_t* data_pkt = bc7215_ac_on();
    if (data_pkt != nullptr)
	{
    	send_ac_cmd_(data_pkt);
    }
    return set_to(current_temp, current_mode, current_fan, current_key);
}

// Send a power-off command generated by the AC protocol library.
const bc7215DataVarPkt_t* BC7215AC::off()
{
    if (!init_ok)
    {
        return nullptr;
    }

    const bc7215DataVarPkt_t* data_pkt = bc7215_ac_off();
    send_ac_cmd_(data_pkt);
    return data_pkt;
}

// Parse the most recently captured IR frame(s) into AC state values.
// Before parsing, replace the AC library base frame with the new capture so the
// library compares against the current protocol format.
bool BC7215AC::parse(int& temp, int& mode, int& fan, int& power)
{
    if (!init_ok)
    {
        return false;
    }

    if (sample_count == 1)
    {
        if (!(sample_status[0] & kErrStatusBit))
        {
            bc7215_ac_replace_base(sample_status[0], reinterpret_cast<const bc7215DataVarPkt_t*>(&sample_data[0]));
        }
        else
        {
            return false;
        }
    }
    else if (sample_count > 1)
    {
        // For multi-frame captures, prepare the linked combined-message array.
        if (!reverse_marked_samples_())
        {
            return false;
        }
        bc7215_ac_replace_base(sample_count, reinterpret_cast<const bc7215DataVarPkt_t*>(received_message_));
    }

    int8_t t = 0;
    int8_t m = 0;
    int8_t f = 0;
    int8_t p = 0;
    bool   result = false;

    // The C parser returns temperature as an offset; convert it back to the
    // user-facing scale before returning.
    if (use_fahrenheit_)
    {
        result = bc7215_ac_parse_f(&t, &m, &f, &p);
        t += 60;
    }
    else
    {
        result = bc7215_ac_parse(&t, &m, &f, &p);
        t += 16;
    }
    // Keep the last known state synchronized with parsed remote-control input.
    if (result)
    {
        current_temp = t;
        current_mode = m;
        current_fan = f;
    }

    temp = static_cast<int>(t);
    mode = static_cast<int>(m);
    fan = static_cast<int>(f);
    power = static_cast<int>(p);
    return result;
}

// Forward BC7215 BUSY status for callers that need to wait for TX/RX activity.
bool BC7215AC::is_busy() const { return bc7215_.is_busy(); }

// Current base data packet selected by the AC protocol library.
const bc7215DataVarPkt_t* BC7215AC::data_packet() const { return bc7215_ac_get_base_data(); }

// Current base format packet. Together with data_packet(), this is enough to
// save the pairing result and restore it later without re-capturing the remote.
const bc7215FormatPkt_t* BC7215AC::format_packet() const { return bc7215_ac_get_base_fmt(); }

// Replace the current base data packet.
// Usually used for special function controls other than Temp, Mode and Fan
bool BC7215AC::replace_base(bc7215DataMaxPkt_t& dataPkt)
{
	uint8_t status;
	status = bc7215_ac_get_base_fmt()->signature.bits.sig;
	return bc7215_ac_replace_base(status, reinterpret_cast<const bc7215DataVarPkt_t*>(&dataPkt));
}

// AC helper-library version string for diagnostics/menu display.
const char* BC7215AC::lib_version() const { return bc7215_ac_get_ver(); }

// Millisecond timestamp based on ESP-IDF high-resolution timer.
uint64_t BC7215AC::now_ms_() { return static_cast<uint64_t>(esp_timer_get_time() / 1000ULL); }

// FreeRTOS-friendly delay; does not busy-wait the CPU.
void BC7215AC::delay_ms_(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// Calculate the byte count occupied by a variable-length data packet:
// 2 bytes of bitLen plus the payload rounded up to full bytes.
uint16_t BC7215AC::data_packet_bytes_(const bc7215DataVarPkt_t* pkt)
{
    if (pkt == nullptr || pkt->bitLen >= 0x1000)
    {
        return 0;
    }
    return static_cast<uint16_t>((pkt->bitLen + 7) / 8 + 2);
}

// Some decodes are marked as bit-reversed/inverted by the chip/library.
// Correct those payload bytes in-place before giving samples to the AC matcher.
bool BC7215AC::reverse_marked_samples_()
{
    for (uint8_t j = 0; j < sample_count && j < kMaxSamples; ++j)
    {
        if (sample_status[j] & kErrStatusBit)
        {
            return false;
        }
        if ((sample_status[j] & kReverseStatusBit) == 0)
        {
            continue;
        }

        const uint16_t bytes = static_cast<uint16_t>((sample_data[j].bitLen + 7) / 8);
        const uint16_t safe_bytes = std::min<uint16_t>(bytes, BC7215_MAX_RX_DATA_SIZE);
        for (uint16_t i = 0; i < safe_bytes; ++i)
        {
            sample_data[j].data[i] = static_cast<uint8_t>(~sample_data[j].data[i]);
        }
        sample_status[j] = static_cast<uint8_t>(sample_status[j] & ~kReverseStatusBit);
    }
    return true;
}

// Copy one predefined protocol record into the normal sample buffers so the
// same initialization path can be reused.
bool BC7215AC::copy_predefined_data_(uint8_t index)
{
    const bc7215DataVarPkt_t* src_data
        = use_fahrenheit_ ? bc7215_ac_predefined_data_f(index) : bc7215_ac_predefined_data(index);
    const bc7215FormatPkt_t* src_format = bc7215_ac_predefined_fmt(index);

    if (src_data == nullptr || src_format == nullptr)
    {
        return false;
    }

    const uint16_t copy_bytes = (src_data->bitLen + 7) / 8;
    if (copy_bytes == 0 || copy_bytes > sizeof(sample_data[0]))
    {
        return false;
    }

    sample_data[0].bitLen = src_data->bitLen;
    std::memcpy(&sample_data[0].data, src_data->data, copy_bytes);
    sample_format[0] = *src_format;
    sample_status[0] = sample_format[0].signature.bits.sig;
    sample_count = 1;
    return true;
}

// Transmit an AC command through BC7215.
// The AC library may return either:
//   - a normal data packet, which should be sent with the current base format; or
//   - a combined message, whose own format must be loaded before its data packet.
void BC7215AC::send_ac_cmd_(const bc7215DataVarPkt_t* data_pkt)
{
    if (data_pkt == nullptr)
    {
        return;
    }

    bc7215_.set_tx();
	wait_tx_complete();

    // In this library, bitLen == 0 is used as a marker for bc7215CombinedMsg_t,
    // not as an empty IR packet.
    if (data_pkt->bitLen == 0)
    {
        const bc7215CombinedMsg_t* combined_msg = reinterpret_cast<const bc7215CombinedMsg_t*>(data_pkt);
        if (combined_msg->body.msg.fmt != nullptr && combined_msg->body.msg.datPkt != nullptr)
        {
            bc7215_.load_format(combined_msg->body.msg.fmt);
            bc7215_.ir_tx(combined_msg->body.msg.datPkt);
        }
    }
    else
    {
        // Normal generated command: reuse the base format found during pairing.
        const bc7215FormatPkt_t* base_format = bc7215_ac_get_base_fmt();
        if (base_format != nullptr)
        {
            bc7215_.load_format(base_format);
            bc7215_.ir_tx(data_pkt);
        }
    }
}

// Wait for on-going transmitting to finish, then pause for kCmdIntervalDelayMs
// If bc7215 keeps busy for longer than kTxCheckIntervalMs*kTransmitTimoutLoops
// return false;
bool BC7215AC::wait_tx_complete()
{
	int i;

    for (i=0; i<kTransmitTimoutLoops; i++)
	{
		if (!bc7215_.is_busy())
		{
    		delay_ms_(kCmdIntervalDelayMs);
			break;
		}
		delay_ms_(kTxCheckIntervalMs);
	}
	if (i >= kTransmitTimoutLoops)
	{
		return false;
	}
	return true;
}


}        // namespace bc7215
