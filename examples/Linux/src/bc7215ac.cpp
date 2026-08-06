/*
 * BC7215AC Linux C++ wrapper
 *
 * This class provides a high-level air-conditioner control interface on top
 * of the lower-level BC7215 driver. It wraps the original C AC library and
 * hides the multi-step operations for pairing, control, predefined protocols,
 * and IR command parsing.
 */
#include "bc7215ac.hpp"

#include <algorithm>
#include <cstring>
#include <ctime>
#include <unistd.h>

// This wrapper relies on all three BC7215 library features:
// receiving is needed for pairing/parsing, format support is needed for
// learned AC protocols, and transmitting is needed for generated AC commands.
#if ENABLE_RECEIVING != 1
# error "BC7215AC requires ENABLE_RECEIVING to be 1"
#endif
#if ENABLE_FORMAT != 1
# error "BC7215AC requires ENABLE_FORMAT to be 1"
#endif
#if ENABLE_TRANSMITTING != 1
# error "BC7215AC requires ENABLE_TRANSMITTING to be 1"
#endif

namespace bc7215 {


// Construct the high-level AC wrapper and pass serial/MOD settings to the low-level driver.
BC7215AC::BC7215AC(const char* serial_device, bool mod_reverse)
    : bc7215_(serial_device, mod_reverse)
{
}


// Initialize the BC7215 driver and leave the chip in TX mode by default.
bool BC7215AC::begin()
{
    if (started_) {
        return true;
    }

    if (!bc7215_.begin()) {
        return false;
    }

    init_ok = false;
    use_fahrenheit_ = false;
    format_received_ = false;
    timer_start_ms_ = 0;

    if (!bc7215_.started()) {
        return false;
    }

    // Switch mode at begin to wake up BC7215 in case it's in sleep mode
	// Default to transmit mode after hardware setup. Capture functions will
    // explicitly switch to RX mode when needed.
	bc7215_.set_rx();
    delay_ms_(kModeSwitchDelayMs);
    bc7215_.set_tx();
    started_ = true;
    return true;
}


// Stop the low-level driver and release the serial port.
void BC7215AC::end()
{
    bc7215_.end();
    started_ = false;
}


// Switch to Fahrenheit operation. Existing pairing data becomes invalid.
void BC7215AC::set_fahrenheit()
{
    if (!use_fahrenheit_) {
        init_ok = false;
        current_temp = 78;
    }
    use_fahrenheit_ = true;
}


// Switch to Celsius operation. Existing Fahrenheit pairing data becomes invalid.
void BC7215AC::set_celsius()
{
    if (use_fahrenheit_) {
        init_ok = false;
        current_temp = 25;
    }
    use_fahrenheit_ = false;
}


// Query the currently selected temperature unit.
bool BC7215AC::is_celsius() const
{
    return !use_fahrenheit_;
}


// Enter BC7215 RX mode and begin collecting one or more IR data segments.
void BC7215AC::start_capture(uint8_t rx_mode)
{
    // Each capture begins from a clean software state. The BC7215 C driver
    // buffers are cleared after the chip has been switched to RX mode.
    format_received_ = false;

    bc7215_.set_rx();
    delay_ms_(kModeSwitchDelayMs);
    bc7215_.set_rx_mode(rx_mode);
    bc7215_.clr_data();
    bc7215_.clr_format();
	sample_count = 0;
    timer_start_ms_ = now_ms_();
}


// Stop capture and return the BC7215A to TX/encoder mode.
void BC7215AC::stop_capture()
{
    format_received_ = false;
    bc7215_.set_tx();
    delay_ms_(kModeSwitchDelayMs);
}


// Poll for received IR data and decide when the capture has become idle.
bool BC7215AC::signal_captured()
{
    if (bc7215_.is_busy()) {
        // While the chip is busy receiving or processing IR data, keep extending
        // the idle timer. Capture ends only after data has been received and the
        // bus stays idle for kCaptureIdleMs.
        timer_start_ms_ = now_ms_();
    } else {
        if (bc7215_.data_ready()) {
            if (sample_count < kMaxSamples) {
                sample_status[sample_count] = bc7215_.get_data(sample_data[sample_count]);
                if (bc7215_.format_ready()) {
                    bc7215_.get_format(sample_format[sample_count]);
                    format_received_ = true;
                }
                ++sample_count;
            }
            timer_start_ms_ = now_ms_();
        }
    }

    // Once at least one segment has been received and no more bytes arrive for
    // the idle interval, treat the whole IR command as complete.
    if (sample_count > 0 && (now_ms_() - timer_start_ms_) > kCaptureIdleMs) {
        return true;
    }
    return false;
}


// Initialize/pair the AC library using the samples captured from a remote.
bool BC7215AC::init()
{
	uint8_t pkt0_rev;
    init_ok = false;

    if (sample_count == 1) {
        // Single-segment commands can be passed to the C library directly.
        if (sample_status[0] & kErrStatusBit) {
            return false;
        }
        if (use_fahrenheit_) {
            init_ok = bc7215_ac_init_f(sample_status[0],
                                       reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
        } else {
            init_ok = bc7215_ac_init(sample_status[0],
                                     reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
        }
    } else if ((sample_count > 1) && (sample_count <= 4)) {
        // Multi-segment AC commands are represented by the combined-message
        // array stored in received_message_.
		pkt0_rev = sample_status[0]&kReverseStatusBit;
        if (!reverse_marked_samples_()) {
            return false;
        }
        if (use_fahrenheit_) {
            init_ok = bc7215_ac_init2_f(sample_count | pkt0_rev, received_message_, 0);
        } else {
            init_ok = bc7215_ac_init2(sample_count | pkt0_rev, received_message_, 0);
        }
    }

    return init_ok;
}


// Initialize/pair the AC library from previously saved data and format packets.
bool BC7215AC::init(uint8_t status, const bc7215DataMaxPkt_t& data, const bc7215FormatPkt_t& format)
{
    sample_data[0] = data;
    sample_format[0] = format;
    sample_status[0] = status;
    sample_count = 1;

    if (use_fahrenheit_) {
        init_ok = bc7215_ac_init_f(sample_status[0],
                                   reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
    } else {
        init_ok = bc7215_ac_init(sample_status[0],
                                 reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
    }
    return init_ok;
}


// Try the next protocol candidate after a successful but incorrect match.
bool BC7215AC::match_next()
{
    if (init_ok) {
        init_ok = bc7215_ac_find_next();
    }
    return init_ok;
}


// Return how many predefined protocols are available in the C library.
uint8_t BC7215AC::predefined_count() const
{
    return bc7215_ac_predefined_cnt();
}


// Return the display name of one predefined protocol entry.
const char* BC7215AC::predefined_name(uint8_t index) const
{
    if (index < predefined_count()) {
        return bc7215_ac_predefined_name(index);
    }
    return nullptr;
}


// Initialize/pair the AC library using one predefined protocol entry.
bool BC7215AC::init_predefined(uint8_t index)
{
    init_ok = false;
    if (index >= predefined_count()) {
        return false;
    }
    if (!copy_predefined_data_(index)) {
        return false;
    }
    if (use_fahrenheit_) {
        init_ok = bc7215_ac_init_f(sample_status[0],
                                   reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
    } else {
        init_ok = bc7215_ac_init(sample_status[0],
                                 reinterpret_cast<const bc7215DataVarPkt_t*>(&received_message_[0]));
    }
    return init_ok;
}


// Generate and transmit an AC command with new temp/mode/fan/key settings.
const bc7215DataVarPkt_t* BC7215AC::set_to(int temp, int mode, int fan, int key)
{
    if (!init_ok) {
        return nullptr;
    }

    current_temp = static_cast<int8_t>(temp);
    current_mode = static_cast<int8_t>(mode);
    current_fan = static_cast<int8_t>(fan);
    current_key = static_cast<int8_t>(key);

    const bc7215DataVarPkt_t* data_pkt = nullptr;
    if (use_fahrenheit_) {
        data_pkt = bc7215_ac_set_f(current_temp - 60, current_mode, current_fan, current_key);
    } else {
        data_pkt = bc7215_ac_set(current_temp - 16, current_mode, current_fan, current_key);
    }
    send_ac_cmd_(data_pkt);
    return data_pkt;
}


// Generate and transmit the AC power-on command, or fall back to a state command.
const bc7215DataVarPkt_t* BC7215AC::on()
{
    if (!init_ok) {
        return nullptr;
    }
    const bc7215DataVarPkt_t* data_pkt = bc7215_ac_on();
    if (data_pkt == nullptr) {
        return set_to(current_temp, current_mode, current_fan, current_key);
    }
    send_ac_cmd_(data_pkt);
    return data_pkt;
}


// Generate and transmit the AC power-off command.
const bc7215DataVarPkt_t* BC7215AC::off()
{
    if (!init_ok) {
        return nullptr;
    }
    const bc7215DataVarPkt_t* data_pkt = bc7215_ac_off();
    send_ac_cmd_(data_pkt);
    return data_pkt;
}


// Parse the last captured IR command according to the currently paired protocol.
bool BC7215AC::parse(int& temp, int& mode, int& fan, int& power)
{
	uint8_t pkt0_rev;
	uint8_t temp_status;
	bc7215DataMaxPkt_t temp_pkt;
	const bc7215DataVarPkt_t* base_pkt;
	bool rep_result;

    if (!init_ok) {
        return false;
    }

	// Backup base status & data
	temp_status = bc7215_ac_get_base_status();
	base_pkt = bc7215_ac_get_base_data();
	temp_pkt.bitLen = base_pkt->bitLen;
	for (int i = 0; i < (temp_pkt.bitLen+7)/8; i++)
	{
		temp_pkt.data[i] = base_pkt->data[i];
	}

    if (sample_count == 1) {
        if (!(sample_status[0] & kErrStatusBit)) {
            rep_result = bc7215_ac_replace_base(sample_status[0], reinterpret_cast<const bc7215DataVarPkt_t*>(&sample_data[0]));
        } else {
            return false;
        }
    } else if ((sample_count > 1) && (sample_count <= 4)) {
		pkt0_rev = sample_status[0]&kReverseStatusBit;
        if (!reverse_marked_samples_()) {
            return false;
        }
        rep_result = bc7215_ac_replace_base(sample_count | pkt0_rev, reinterpret_cast<const bc7215DataVarPkt_t*>(received_message_));
    }
	else {
		return false;
	}

    int8_t t = 0;
    int8_t m = 0;
    int8_t f = 0;
    int8_t p = 0;
    bool result = false;

    if (use_fahrenheit_) {
        result = bc7215_ac_parse_f(&t, &m, &f, &p);
        t += 60;
    } else {
        result = bc7215_ac_parse(&t, &m, &f, &p);
        t += 16;
    }

    if (result) {
        // Keep the cached AC state synchronized with successfully decoded
        // remote-control commands.
        current_temp = t;
        current_mode = m;
        current_fan = f;
    }

	// If replace base was not successful, restore prevoious base
	if (!rep_result)
	{
		bc7215_ac_replace_base(temp_status, reinterpret_cast<const bc7215DataVarPkt_t*>(&temp_pkt));
	}

    temp = static_cast<int>(t);
    mode = static_cast<int>(m);
    fan = static_cast<int>(f);
    power = static_cast<int>(p);
    return result;
}


// Report whether the BC7215A is still busy, usually during IR transmission.
bool BC7215AC::is_busy() const
{
    return bc7215_.is_busy();
}


// Return the base status byte currently used by the AC library.
uint8_t BC7215AC::status_byte() const
{
    return bc7215_ac_get_base_status();
}

// Return the base data packet currently used by the AC library.
const bc7215DataVarPkt_t* BC7215AC::data_packet() const
{
    return bc7215_ac_get_base_data();
}


// Return the base format packet currently used by the AC library.
const bc7215FormatPkt_t* BC7215AC::format_packet() const
{
    return bc7215_ac_get_base_fmt();
}


// Replace the current base data packet.
// Usually used for special function controls other than Temp, Mode and Fan
bool BC7215AC::replace_base(bc7215DataMaxPkt_t& dataPkt)
{
	uint8_t status;
	status = bc7215_ac_get_base_status();
	return bc7215_ac_replace_base(status, reinterpret_cast<const bc7215DataVarPkt_t*>(&dataPkt));
}

// Return the version string of the underlying AC C library.
const char* BC7215AC::lib_version() const
{
    return bc7215_ac_get_ver();
}


// Monotonic timestamp helper used for capture idle timing.
uint64_t BC7215AC::now_ms_()
{
    timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000000ULL;
}


// Millisecond delay helper for mode-switch settling.
void BC7215AC::delay_ms_(uint32_t ms)
{
    usleep(static_cast<useconds_t>(ms) * 1000U);
}


// Safely calculate the byte size of a variable-length data packet.
uint16_t BC7215AC::data_packet_bytes_(const bc7215DataVarPkt_t* pkt)
{
    if (pkt == nullptr || pkt->bitLen >= 0x1000) {
        return 0;
    }
    return static_cast<uint16_t>((pkt->bitLen + 7) / 8 + sizeof(uint16_t));
}



// Check error and apply bit inversion to samples have different BC7215 reverse-data status bit than the 1st packet.
bool BC7215AC::reverse_marked_samples_()
{
 	if (sample_status[0] & kErrStatusBit)				// check error bit in first packet
	{
		return false;
	}
	uint8_t Pkt0Rev = sample_status[0]&kReverseStatusBit;	// get REV bit in first packet
    for (uint8_t j = 1; j < sample_count && j < kMaxSamples; ++j) {
        if (sample_status[j] & kErrStatusBit) {
            return false;
        }
        if ((sample_status[j] & kReverseStatusBit) == Pkt0Rev) {
            continue;
        }

        const uint16_t bytes = static_cast<uint16_t>((sample_data[j].bitLen + 7) / 8);
        const uint16_t safe_bytes = std::min<uint16_t>(bytes, BC7215_MAX_RX_DATA_SIZE);
        for (uint16_t i = 0; i < safe_bytes; ++i) {
            sample_data[j].data[i] = static_cast<uint8_t>(~sample_data[j].data[i]);
        }
        sample_status[j] = static_cast<uint8_t>(sample_status[j] & ~kReverseStatusBit) | Pkt0Rev;
    }
    return true;
}


// Copy predefined protocol data into the wrapper's sample buffers.
bool BC7215AC::copy_predefined_data_(uint8_t index)
{
    const bc7215DataVarPkt_t* src_data = use_fahrenheit_ ?
        bc7215_ac_predefined_data_f(index) : bc7215_ac_predefined_data(index);
    const bc7215FormatPkt_t* src_format = bc7215_ac_predefined_fmt(index);

    if (src_data == nullptr || src_format == nullptr) {
        return false;
    }

    const uint16_t copy_bytes = static_cast<uint16_t>((src_data->bitLen + 7) / 8);
    if (copy_bytes == 0 || copy_bytes > sizeof(sample_data[0].data)) {
        return false;
    }

    sample_data[0].bitLen = src_data->bitLen;
    std::memcpy(sample_data[0].data, src_data->data, copy_bytes);
    sample_format[0] = *src_format;
    sample_status[0] = sample_format[0].signature.bits.sig;
    sample_count = 1;
    return true;
}


// Load the correct format and transmit a generated AC command through BC7215A.
void BC7215AC::send_ac_cmd_(const bc7215DataVarPkt_t* data_pkt)
{
    if (data_pkt == nullptr) {
        return;
    }

    bc7215_.set_tx();

    if (data_pkt->bitLen == 0) {
        // A zero bit length is used by the C library as a marker for a combined
        // multi-segment message. In that case, the format pointer and data pointer
        // must be loaded from the combined structure.
        const bc7215CombinedMsg_t* combined_msg = reinterpret_cast<const bc7215CombinedMsg_t*>(data_pkt);
        if (combined_msg->body.msg.fmt != nullptr && combined_msg->body.msg.datPkt != nullptr) {
            bc7215_.load_format(combined_msg->body.msg.fmt);
            bc7215_.ir_tx(combined_msg->body.msg.datPkt);
        }
    } else {
        const bc7215FormatPkt_t* base_format = bc7215_ac_get_base_fmt();
        if (base_format != nullptr) {
            bc7215_.load_format(base_format);
            bc7215_.ir_tx(data_pkt);
        }
    }
}

} // namespace bc7215
