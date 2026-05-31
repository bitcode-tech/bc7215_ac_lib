#pragma once

// C++ wrapper for the BC7215 air-conditioner protocol helper library.
// This layer combines the low-level BC7215 UART driver with the AC-specific
// pairing, predefined-protocol, command-generation, and IR parsing APIs.

#include <cstdint>

#include "bc7215.hpp"
#include "esp_err.h"

// The AC protocol library is a C library. Keep it inside extern "C" so that
// this C++ wrapper can link to the original C symbols without name mangling.
extern "C"
{
#include "bc7215_ac_lib.h"
}

namespace bc7215
{

// High-level air-conditioner controller built on top of BC7215.
//
// Responsibilities:
//   1. Capture IR samples from an existing AC remote.
//   2. Initialize the AC protocol library from captured or predefined data.
//   3. Generate AC control commands, then send them through the BC7215 chip.
//   4. Parse later captured IR frames back into temp/mode/fan/power states.
class BC7215AC
{
private:
    // Maximum number of IR frames retained during pairing/parsing.
    static constexpr uint8_t kMaxSamples = 4;
    // Once at least one frame is received, this idle time marks the end of the
    // current capture sequence.
    static constexpr uint32_t kCaptureIdleMs = 200;
    // Small guard delay after switching BC7215 between RX and TX modes.
    static constexpr uint32_t kModeSwitchDelayMs = 50;
    // Status bits returned by the low-level library. Reverse means the received
    // data bits need to be inverted before passing them to the AC protocol library.
    static constexpr uint8_t kReverseStatusBit = 0x40;
    static constexpr uint8_t kErrStatusBit = 0x80;

public:
    // Construct the wrapper and its embedded low-level BC7215 driver.
    // Hardware initialization is intentionally delayed until begin().
    explicit BC7215AC(
        uart_port_t uart_num, gpio_num_t tx_pin, gpio_num_t rx_pin, gpio_num_t cts_busy_pin, gpio_num_t mod_pin);

    // Initialize the underlying UART/GPIO driver and put BC7215 into TX mode.
    esp_err_t begin();

    // This object owns hardware resources and runtime buffers, so copying would
    // duplicate driver state incorrectly. Use one instance per BC7215 device.
    BC7215AC(const BC7215AC&) = delete;
    BC7215AC& operator=(const BC7215AC&) = delete;

    // Wrapper-level start flag accessor. In this source, begin() checks this flag
    // before reinitializing; ensure started_ is set after successful initialization
    // if you want repeated begin() calls to be skipped.
    bool started() const { return started_; }

    // Public state kept close to the Arduino library for easier porting/debugging.
    // True only after the AC protocol library has been successfully paired or
    // initialized from saved/predefined protocol data. Command functions return
    // nullptr while this flag is false.
    bool init_ok = false;
    // Captured sample buffers. Some AC protocols require more than one IR frame
    // for reliable identification, so up to kMaxSamples frames are stored.
    uint8_t            sample_count = 0;
    uint8_t            sample_status[kMaxSamples] = {};
    bc7215DataMaxPkt_t sample_data[kMaxSamples] = {};
    bc7215FormatPkt_t  sample_format[kMaxSamples] = {};

    // Temperature unit selection. Changing unit invalidates the current
    // AC-library match because Celsius and Fahrenheit tables/offsets differ.
    void set_fahrenheit();
    void set_celsius();
    bool is_celsius() const;

    // Capture workflow. start_capture() switches BC7215 to RX mode;
    // signal_captured() should then be polled until it reports that one capture
    // burst has ended; stop_capture() returns the chip to TX mode.
    void start_capture(uint8_t rx_mode);
    void stop_capture();
    bool signal_captured();

    // Initialize/pair AC protocol from captured samples or explicit packets.
    // init() uses the samples currently stored in sample_data/sample_format.
    bool init();
    bool init(const bc7215DataMaxPkt_t& data, const bc7215FormatPkt_t& format);
    bool match_next();

    // Predefined built-in protocols. These are fallback protocol records bundled
    // in the AC library for formats the BC7215 chip cannot directly decode.
    uint8_t     predefined_count() const;
    const char* predefined_name(uint8_t index) const;
    bool        init_predefined(uint8_t index);

    // Generate and transmit AC commands. The returned pointer is useful for
    // debugging/printing, while send_ac_cmd_() performs the actual transmission.
    const bc7215DataVarPkt_t* set_to(int temp, int mode = -1, int fan = -1, int key = 2);
    const bc7215DataVarPkt_t* on();
    const bc7215DataVarPkt_t* off();

    // Parse the last captured IR signal using the currently initialized AC
    // protocol base. The output values are converted to user-facing units.
    bool parse(int& temp, int& mode, int& fan, int& power);

    // Status/accessors. data_packet()/format_packet() expose the current base
    // protocol data so it can be saved to NVS and restored later.
    bool                      is_busy() const;
    const bc7215DataVarPkt_t* data_packet() const;
    const bc7215FormatPkt_t*  format_packet() const;
    const char*               lib_version() const;

    // Optional access to the lower-level driver for advanced/debug operations.
    BC7215&       driver() { return bc7215_; }
    const BC7215& driver() const { return bc7215_; }

private:
    // Embedded low-level hardware driver. This wrapper owns it directly so
    // initialization order is well-defined.
    BC7215 bc7215_;
    // Combined-message descriptors link each captured format packet to its
    // corresponding data packet without copying both into another large buffer.
    const bc7215CombinedMsg_t received_message_[kMaxSamples] = {
		{.bitLen = 0, .body = {.msg = {.fmt = &sample_format[0], .datPkt = reinterpret_cast<bc7215DataVarPkt_t*>(&sample_data[0])}}},
		{.bitLen = 0, .body = {.msg = {.fmt = &sample_format[1], .datPkt = reinterpret_cast<bc7215DataVarPkt_t*>(&sample_data[1])}}},
		{.bitLen = 0, .body = {.msg = {.fmt = &sample_format[2], .datPkt = reinterpret_cast<bc7215DataVarPkt_t*>(&sample_data[2])}}},
		{.bitLen = 0, .body = {.msg = {.fmt = &sample_format[3], .datPkt = reinterpret_cast<bc7215DataVarPkt_t*>(&sample_data[3])}}}
	};

    uint64_t            timer_start_ms_ = 0;
    // Last known command state. Used when a protocol has no dedicated ON command:
    // on() can resend the previous temperature/mode/fan/key setting instead.
    int8_t current_temp = 25;
    int8_t current_mode = 1;
    int8_t current_fan = 1;
    int8_t current_key = 3;
    bool   started_ = false;
    bool   format_received_ = false;
    bool   use_fahrenheit_ = false;

    // Time/delay helpers wrapped here to keep ESP-IDF calls out of public code.
    static uint64_t now_ms_();
    static void     delay_ms_(uint32_t ms);
    static uint16_t data_packet_bytes_(const bc7215DataVarPkt_t* pkt);

    // Internal sample/message buffer management.
    bool reverse_marked_samples_();
    bool copy_predefined_data_(uint8_t index);

    // Load the correct format into BC7215 and transmit either a normal data
    // packet or a combined format+data packet.
    void send_ac_cmd_(const bc7215DataVarPkt_t* data_pkt);
};

}        // namespace bc7215
