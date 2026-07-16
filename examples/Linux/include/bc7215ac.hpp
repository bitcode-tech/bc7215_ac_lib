#pragma once

#include <cstdint>
#include "bc7215.hpp"

extern "C" {
#include "bc7215_ac_lib.h"
}

namespace bc7215 {

class BC7215AC {
private:
    static constexpr uint8_t kMaxSamples = 4;
    static constexpr uint32_t kCaptureIdleMs = 200;
    static constexpr uint32_t kModeSwitchDelayMs = 50;
    static constexpr uint8_t kReverseStatusBit = 0x40;
    static constexpr uint8_t kErrStatusBit = 0x80;
public:
    explicit BC7215AC(const char* serial_device, bool mod_reverse = false);

    bool begin();
    void end();

    BC7215AC(const BC7215AC&) = delete;
    BC7215AC& operator=(const BC7215AC&) = delete;

    bool started() const { return started_; }
    const char* last_error() const { return bc7215_.last_error(); }

    bool init_ok = false;
    uint8_t sample_count = 0;
    uint8_t sample_status[kMaxSamples] = {};
    bc7215DataMaxPkt_t sample_data[kMaxSamples] = {};
    bc7215FormatPkt_t sample_format[kMaxSamples] = {};

    void set_fahrenheit();
    void set_celsius();
    bool is_celsius() const;

    void start_capture(uint8_t rx_mode);
    void stop_capture();
    bool signal_captured();

    bool init();
    bool init(uint8_t status, const bc7215DataMaxPkt_t& data, const bc7215FormatPkt_t& format);
    bool match_next();

    uint8_t predefined_count() const;
    const char* predefined_name(uint8_t index) const;
    bool init_predefined(uint8_t index);

    const bc7215DataVarPkt_t* set_to(int temp, int mode = -1, int fan = -1, int key = 2);
    const bc7215DataVarPkt_t* on();
    const bc7215DataVarPkt_t* off();

    bool parse(int& temp, int& mode, int& fan, int& power);

    bool is_busy() const;
	uint8_t status_byte() const;
    const bc7215DataVarPkt_t* data_packet() const;
    const bc7215FormatPkt_t* format_packet() const;
    const char* lib_version() const;

    BC7215& driver() { return bc7215_; }
    const BC7215& driver() const { return bc7215_; }

private:
    BC7215 bc7215_;
    const bc7215CombinedMsg_t received_message_[kMaxSamples] = {
		{.bitLen = 0, .body = {.msg = {.fmt = &sample_format[0], .datPkt = reinterpret_cast<bc7215DataVarPkt_t*>(&sample_data[0])}}},
		{.bitLen = 0, .body = {.msg = {.fmt = &sample_format[1], .datPkt = reinterpret_cast<bc7215DataVarPkt_t*>(&sample_data[1])}}},
		{.bitLen = 0, .body = {.msg = {.fmt = &sample_format[2], .datPkt = reinterpret_cast<bc7215DataVarPkt_t*>(&sample_data[2])}}},
		{.bitLen = 0, .body = {.msg = {.fmt = &sample_format[3], .datPkt = reinterpret_cast<bc7215DataVarPkt_t*>(&sample_data[3])}}}
	};

    uint64_t timer_start_ms_ = 0;

    int8_t current_temp = 25;
    int8_t current_mode = 1;
    int8_t current_fan = 1;
    int8_t current_key = 3;

    bool started_ = false;
    bool format_received_ = false;
    bool use_fahrenheit_ = false;

    static uint64_t now_ms_();
    static void delay_ms_(uint32_t ms);
    static uint16_t data_packet_bytes_(const bc7215DataVarPkt_t* pkt);

    bool reverse_marked_samples_();
    bool copy_predefined_data_(uint8_t index);
    void send_ac_cmd_(const bc7215DataVarPkt_t* data_pkt);
};

} // namespace bc7215
