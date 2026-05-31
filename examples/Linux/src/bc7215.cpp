/*
 * BC7215 Linux C++ wrapper
 *
 * This file replaces the ESP-IDF UART/GPIO/FreeRTOS layer with a POSIX/Linux
 * implementation. The original BC7215 C driver still provides the protocol
 * logic; this wrapper supplies serial I/O, DTR-based MOD control, CTS hardware
 * flow control, a polling RX thread, and pthread mutex callbacks.
 */
#include "bc7215.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

// Some libc/kernel header combinations expose the hardware-flow-control flag
// as CNEW_RTSCTS instead of CRTSCTS. Map it so the rest of the code can use one
// name.
#ifndef CRTSCTS
# ifdef CNEW_RTSCTS
#  define CRTSCTS CNEW_RTSCTS
# endif
#endif

// The Linux port expects BC7215 BUSY to be wired to the serial CTS input and
// uses the tty driver's hardware flow control when transmitting.
#if TX_HW_FLOW_CONTROL != 1
# error "Linux BC7215 port requires TX_HW_FLOW_CONTROL to be 1"
#endif
#if USE_UART_INTERRUPT != 0
# error "Linux BC7215 port requires USE_UART_INTERRUPT to be 0"
#endif
#if PROCESS_IN_DIFFERENT_THREAD != 1
# error "Linux BC7215 port requires PROCESS_IN_DIFFERENT_THREAD to be 1"
#endif
#ifndef CRTSCTS
# error "This target does not expose CRTSCTS hardware flow-control support"
#endif

namespace {


// Configure termios to raw mode without relying only on cfmakeraw availability.
static void make_raw_termios(termios& tio)
{
#ifdef _BSD_SOURCE
    cfmakeraw(&tio);
#else
    tio.c_iflag &= static_cast<tcflag_t>(~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON));
    tio.c_oflag &= static_cast<tcflag_t>(~OPOST);
    tio.c_lflag &= static_cast<tcflag_t>(~(ECHO | ECHONL | ICANON | ISIG | IEXTEN));
    tio.c_cflag &= static_cast<tcflag_t>(~(CSIZE | PARENB));
    tio.c_cflag |= CS8;
#endif
}

} // namespace

namespace bc7215 {


// The original C library stores callbacks globally, so only one active wrapper instance is supported.
BC7215* BC7215::active_ = nullptr;


// Store the serial device path and the selected MOD/DTR polarity mapping.
BC7215::BC7215(const char* device_path, bool mod_reverse)
    : device_path_(device_path != nullptr ? device_path : "/dev/ttyUSB0"),
      mod_reverse_(mod_reverse)
{
}


// Ensure the serial port and RX thread are released if the object is destroyed.
BC7215::~BC7215()
{
    end();
}


// Save the last system-call error message for diagnostics.
void BC7215::set_error_(const char* prefix)
{
    last_error_ = prefix;
    last_error_ += ": ";
    last_error_ += std::strerror(errno);
}


// Save a non-errno error message for diagnostics.
void BC7215::set_error_(const std::string& msg)
{
    last_error_ = msg;
}


// Open/configure the serial port, register C-library callbacks, and start the RX thread.
bool BC7215::begin(uint32_t rx_poll_ms)
{
    if (started_) {
        return true;
    }

    if (active_ != nullptr && active_ != this) {
        // Callback functions in the C library are global, not per object.
        // Supporting multiple simultaneous instances would require an extra
        // dispatcher layer in the C library itself.
        set_error_("bc7215_lib uses global callbacks; only one active BC7215 instance is supported");
        return false;
    }

    rx_poll_ms_ = (rx_poll_ms == 0) ? 10 : rx_poll_ms;

    if (pthread_mutex_init(&mutex_, nullptr) != 0) {
        set_error_("pthread_mutex_init failed");
        return false;
    }
    mutex_initialized_ = true;

    if (!open_serial_()) {
        end();
        return false;
    }

    if (!configure_serial_()) {
        end();
        return false;
    }

    active_ = this;

    // Default to BC7215 TX/encoder mode so MOD is not left floating.
    set_dtr_(false);

    bc7215_config_uart_send_byte(&BC7215::uart_send_byte_cb_);
    bc7215_config_mod_set_high_funct(&BC7215::mod_set_high_cb_);
    bc7215_config_mod_set_low_funct(&BC7215::mod_set_low_cb_);
    bc7215_config_read_mod_funct(&BC7215::read_mod_cb_);
    bc7215_config_mutex_lock_funct(&BC7215::mutex_lock_cb_);
    bc7215_config_mutex_unlock_funct(&BC7215::mutex_unlock_cb_);

    // The ESP-IDF version processes RX data in a FreeRTOS task. The Linux port
    // uses an equivalent POSIX thread that polls the tty and feeds the C decoder.
    rx_thread_running_ = true;
    if (pthread_create(&rx_thread_, nullptr, &BC7215::rx_thread_trampoline_, this) != 0) {
        set_error_("pthread_create failed");
        rx_thread_running_ = false;
        end();
        return false;
    }
    rx_thread_created_ = true;

    started_ = true;
    return true;
}


// Stop the RX thread, close the serial port, and unregister this active instance.
void BC7215::end()
{
    rx_thread_running_ = false;

    if (rx_thread_created_) {
        pthread_join(rx_thread_, nullptr);
        rx_thread_created_ = false;
    }

    if (serial_fd_ >= 0) {
        set_dtr_(false);
        tcflush(serial_fd_, TCIOFLUSH);
        close(serial_fd_);
        serial_fd_ = -1;
    }

    if (mutex_initialized_) {
        pthread_mutex_destroy(&mutex_);
        mutex_initialized_ = false;
    }

    if (active_ == this) {
        active_ = nullptr;
    }

    started_ = false;
}


// Open the Linux tty device in nonblocking mode.
bool BC7215::open_serial_()
{
    serial_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
        set_error_(("open " + device_path_).c_str());
        return false;
    }
    return true;
}


// Configure the tty as 19200 bps, 8N2, raw mode, with CTS hardware flow control.
bool BC7215::configure_serial_()
{
    termios tio {};
    if (tcgetattr(serial_fd_, &tio) != 0) {
        set_error_("tcgetattr failed");
        return false;
    }

    make_raw_termios(tio);

    cfsetispeed(&tio, B19200);
    cfsetospeed(&tio, B19200);

    tio.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tio.c_cflag |= CS8;
    tio.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tio.c_cflag |= CSTOPB;      // 2 stop bits: 8N2
    tio.c_cflag |= CLOCAL;
    tio.c_cflag |= CREAD;
    tio.c_cflag &= static_cast<tcflag_t>(~HUPCL);
    tio.c_cflag |= CRTSCTS;     // BC7215 BUSY -> Linux CTS; TX pauses automatically while busy.

    tio.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));

    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tio) != 0) {
        set_error_("tcsetattr failed");
        return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    return true;
}


// Drive the BC7215 MOD signal through the serial port DTR modem-control line.
bool BC7215::set_dtr_(bool logical_high)
{
    if (serial_fd_ < 0) {
        return false;
    }

    // Default polarity: many USB-UART adapters expose DTR as an active-low TTL signal.
    // Therefore logical MOD high means DTR is deasserted, and logical MOD low
    // means DTR is asserted. --mod-reverse flips this mapping.
    const bool physical_high = mod_reverse_ ? logical_high : !logical_high;
    int bit = TIOCM_DTR;
    if (ioctl(serial_fd_, physical_high ? TIOCMBIS : TIOCMBIC, &bit) != 0) {
        set_error_("DTR ioctl failed");
        return false;
    }
    return true;
}


// Read back the logical MOD level from the DTR modem-control state.
bool BC7215::read_dtr_(uint8_t& logical_high) const
{
    logical_high = 0;
    if (serial_fd_ < 0) {
        return false;
    }

    int status = 0;
    if (ioctl(serial_fd_, TIOCMGET, &status) != 0) {
        return false;
    }

    bool physical_high = (status & TIOCM_DTR) != 0;
    bool logical = mod_reverse_ ? physical_high : !physical_high;
    logical_high = logical ? 1 : 0;
    return true;
}


// Write one byte to the serial port; CTS flow control is handled by the tty driver.
bool BC7215::write_byte_(uint8_t data)
{
    while (started_ && serial_fd_ >= 0) {
        const ssize_t n = write(serial_fd_, &data, 1);
        if (n == 1) {
            return true;
        }
        if (n < 0 && errno == EINTR) {
            continue;
        }
        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
            pollfd pfd {};
            pfd.fd = serial_fd_;
            pfd.events = POLLOUT;
            poll(&pfd, 1, 100);
            continue;
        }
        set_error_("serial write failed");
        return false;
    }
    return false;
}


// pthread entry point that forwards execution to the C++ member function.
void* BC7215::rx_thread_trampoline_(void* arg)
{
    static_cast<BC7215*>(arg)->rx_thread_loop_();
    return nullptr;
}


// Poll and read the serial port, then feed each byte to the BC7215 C decoder.
void BC7215::rx_thread_loop_()
{
    uint8_t rx_buf[kRxReadChunkSize];

    while (rx_thread_running_) {
        pollfd pfd {};
        pfd.fd = serial_fd_;
        pfd.events = POLLIN;

        const int ret = poll(&pfd, 1, static_cast<int>(rx_poll_ms_));
        if (ret <= 0) {
            continue;
        }

        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            continue;
        }

        if ((pfd.revents & POLLIN) == 0) {
            continue;
        }

        // Drain all currently available bytes. read() is nonblocking, so the
        // inner loop exits when the tty buffer becomes empty.
        while (rx_thread_running_) {
            const ssize_t n = read(serial_fd_, rx_buf, sizeof(rx_buf));
            if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    bc7215_process_uart_data(rx_buf[i]);
                }
                continue;
            }
            if (n < 0 && errno == EINTR) {
                continue;
            }
            if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
                break;
            }
            break;
        }
    }
}


// Put the BC7215A into receiver mode.
void BC7215::set_rx()
{
    bc7215_set_rx();
}


// Put the BC7215A into transmitter/encoder mode.
void BC7215::set_tx()
{
    bc7215_set_tx();
}


// Put the BC7215A into shutdown mode.
void BC7215::shutdown()
{
    bc7215_set_shutdown();
}


// Select the BC7215A receive mode, such as simple or complex mode.
void BC7215::set_rx_mode(uint8_t mode)
{
    bc7215_set_rx_mode(mode);
}


// Query the BC7215A BUSY state through the C driver.
bool BC7215::is_busy() const
{
    return bc7215_is_busy() != 0;
}


// Query whether the last BC7215 command has completed.
bool BC7215::cmd_completed() const
{
    return bc7215_cmd_completed() != 0;
}

#if ENABLE_RECEIVING == 1

// Query whether a received IR data packet is available.
bool BC7215::data_ready() const
{
    return bc7215_data_ready() != 0;
}


// Clear the received data-ready state inside the C driver.
void BC7215::clr_data()
{
    bc7215_clr_data();
}


// Return the current received data length reported by the C driver.
uint16_t BC7215::get_len() const
{
    return bc7215_get_len();
}


// Return the received packet size reported by the C driver.
uint16_t BC7215::data_packet_size() const
{
    return bc7215_dpkt_size();
}


// Copy the received data packet into a caller-provided variable packet buffer.
uint8_t BC7215::get_data(bc7215DataVarPkt_t* target)
{
    return bc7215_get_data(target);
}


// Convenience overload for copying data into a maximum-size packet buffer.
uint8_t BC7215::get_data(bc7215DataMaxPkt_t& target)
{
    return bc7215_get_data(reinterpret_cast<bc7215DataVarPkt_t*>(&target));
}


// Copy raw received bytes, adapting to the C driver's configured size type.
uint16_t BC7215::get_raw(void* addr, uint16_t size)
{
# if BC7215_MAX_RX_DATA_SIZE >= 256
    return bc7215_get_raw(addr, size);
# else
    if (size > 0xFF) {
        size = 0xFF;
    }
    return bc7215_get_raw(addr, static_cast<uint8_t>(size));
# endif
}

# if ENABLE_FORMAT == 1

// Query whether a received format packet is available.
bool BC7215::format_ready() const
{
    return bc7215_format_ready() != 0;
}


// Clear the format-ready state inside the C driver.
void BC7215::clr_format()
{
    bc7215_clr_format();
}


// Copy the received format packet into a caller-provided buffer.
uint8_t BC7215::get_format(bc7215FormatPkt_t* target)
{
    return bc7215_get_format(target);
}


// Convenience overload for copying the received format packet by reference.
uint8_t BC7215::get_format(bc7215FormatPkt_t& target)
{
    return bc7215_get_format(&target);
}
# endif
#endif

#if ENABLE_TRANSMITTING == 1

// Load a format packet into the BC7215 transmitter.
void BC7215::load_format(const bc7215FormatPkt_t* source)
{
    bc7215_load_format(source);
}


// Convenience overload for loading a format packet by reference.
void BC7215::load_format(const bc7215FormatPkt_t& source)
{
    bc7215_load_format(&source);
}


// Transmit one variable-length IR data packet.
void BC7215::ir_tx(const bc7215DataVarPkt_t* source)
{
    bc7215_IR_tx(source);
}


// Convenience overload for transmitting a maximum-size IR data packet.
void BC7215::ir_tx(const bc7215DataMaxPkt_t& source)
{
    bc7215_IR_tx(reinterpret_cast<const bc7215DataVarPkt_t*>(&source));
}


// Send raw BC7215 command bytes through the C driver.
void BC7215::send_raw(const void* raw_data, uint16_t size)
{
    bc7215_send_raw(raw_data, size);
}
#endif


// Ask the C driver to calculate the storage size of a data packet.
uint16_t BC7215::calc_size(const bc7215DataVarPkt_t* data_pkt) const
{
    return bc7215_cal_size(data_pkt);
}


// Calculate the BC7215 CRC8 value, adapting length type to configuration.
uint8_t BC7215::crc8(const uint8_t* data, uint16_t len) const
{
#if BC7215_MAX_RX_DATA_SIZE < 256
    if (len > 0xFF) {
        len = 0xFF;
    }
    return bc7215_crc8(data, static_cast<uint8_t>(len));
#else
    return bc7215_crc8(data, len);
#endif
}


// C-library callback: send one UART byte to the Linux serial port.
void BC7215::uart_send_byte_cb_(uint8_t data)
{
    if (active_ == nullptr || !active_->started_) {
        return;
    }
    active_->write_byte_(data);
}


// C-library callback: set logical MOD high through DTR.
void BC7215::mod_set_high_cb_()
{
    if (active_ != nullptr) {
        active_->set_dtr_(true);
    }
}


// C-library callback: set logical MOD low through DTR.
void BC7215::mod_set_low_cb_()
{
    if (active_ != nullptr) {
        active_->set_dtr_(false);
    }
}


// C-library callback: read logical MOD level from DTR.
uint8_t BC7215::read_mod_cb_()
{
    uint8_t value = 0;
    if (active_ != nullptr) {
        active_->read_dtr_(value);
    }
    return value;
}


// C-library callback: lock the pthread mutex around shared driver state.
void BC7215::mutex_lock_cb_()
{
    if (active_ != nullptr && active_->mutex_initialized_) {
        pthread_mutex_lock(&active_->mutex_);
    }
}


// C-library callback: unlock the pthread mutex around shared driver state.
void BC7215::mutex_unlock_cb_()
{
    if (active_ != nullptr && active_->mutex_initialized_) {
        pthread_mutex_unlock(&active_->mutex_);
    }
}

} // namespace bc7215
