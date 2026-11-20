#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

extern "C" {
#include "bc7215_lib.h"
}

namespace bc7215 {

/**
 * Linux/POSIX C++ wrapper for the BC7215 / BC7215A low-level C driver.
 *
 * Hardware mapping:
 *   Linux serial TX  -> BC7215 RX
 *   Linux serial RX  <- BC7215 TX
 *   Linux serial CTS <- BC7215 BUSY
 *   Linux serial DTR -> BC7215 MOD
 *   GND             -> GND
 *
 * The original C library stores global callback pointers, so this wrapper
 * intentionally supports only one active object at a time.
 */
class BC7215 {
public:
    explicit BC7215(const char* device_path, bool mod_reverse = false);
    ~BC7215();

    BC7215(const BC7215&) = delete;
    BC7215& operator=(const BC7215&) = delete;

    bool begin();
    void end();
    

    bool started() const { return started_; }
    const char* last_error() const { return last_error_.c_str(); }

    void set_rx();
    void set_tx();
    void shutdown();
    void set_rx_mode(uint8_t mode);

    bool is_busy();
    bool cmd_completed();

#if ENABLE_RECEIVING == 1
    bool data_ready();
    void clr_data();
    uint16_t get_len() const;
    uint16_t data_packet_size() const;
    uint8_t get_data(bc7215DataVarPkt_t* target);
    uint8_t get_data(bc7215DataMaxPkt_t& target);
    uint16_t get_raw(void* addr, uint16_t size);
# if ENABLE_FORMAT == 1
    bool format_ready();
    void clr_format();
    uint8_t get_format(bc7215FormatPkt_t* target);
    uint8_t get_format(bc7215FormatPkt_t& target);
# endif
#endif

#if ENABLE_TRANSMITTING == 1
    void load_format(const bc7215FormatPkt_t* source);
    void load_format(const bc7215FormatPkt_t& source);
    void ir_tx(const bc7215DataVarPkt_t* source);
    void ir_tx(const bc7215DataMaxPkt_t& source);
    void send_raw(const void* raw_data, uint16_t size);
#endif

    uint16_t calc_size(const bc7215DataVarPkt_t* data_pkt) const;
    uint8_t crc8(const uint8_t* data, uint16_t len) const;

private:
    static BC7215* active_;

    std::string device_path_;
    bool mod_reverse_ = false;
    int serial_fd_ = -1;

    bool started_ = false;
    std::string last_error_;

    void set_error_(const char* prefix);
    void set_error_(const std::string& msg);

    bool open_serial_();
    bool configure_serial_();
    bool set_dtr_(bool logical_high);
    bool read_dtr_(uint8_t& logical_high) const;
    bool write_byte_(uint8_t data);
    // 主动轮询串口，若有数据则逐字节送入解析器；一旦解析出完整数据包即返回，未读字节保留在底层缓冲
    void poll_rx();

    static void uart_send_byte_cb_(uint8_t data);
    static void mod_set_high_cb_();
    static void mod_set_low_cb_();
    static uint8_t read_mod_cb_();
};

} // namespace bc7215
