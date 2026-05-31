#include "console_line.hpp"

#include <cerrno>
#include <cstdio>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

ConsoleLineReader::~ConsoleLineReader()
{
    end();
}

bool ConsoleLineReader::begin()
{
    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (old_flags_ < 0) {
        return false;
    }

    if (fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK) != 0) {
        return false;
    }

    active_ = true;
    return true;
}

void ConsoleLineReader::end()
{
    if (active_ && old_flags_ >= 0) {
        fcntl(STDIN_FILENO, F_SETFL, old_flags_);
    }
    active_ = false;
}

void ConsoleLineReader::clear_buffer()
{
    buffer_.clear();
    last_was_cr_ = false;

    char tmp[64];
    while (read(STDIN_FILENO, tmp, sizeof(tmp)) > 0) {
    }
}

bool ConsoleLineReader::read_line(std::string& out)
{
    out.clear();

    pollfd pfd {};
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;

    const int pr = poll(&pfd, 1, 0);
    if (pr <= 0 || (pfd.revents & POLLIN) == 0) {
        return false;
    }

    char ch = 0;
    while (true) {
        const ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n == 1) {
            if (ch == '\r') {
                last_was_cr_ = true;
                out = buffer_;
                buffer_.clear();
                return true;
            }
            if (ch == '\n') {
                if (last_was_cr_) {
                    last_was_cr_ = false;
                    continue;
                }
                out = buffer_;
                buffer_.clear();
                return true;
            }
            last_was_cr_ = false;

            // Usually terminal canonical mode handles Backspace before bytes reach us.
            // This fallback also works if stdin is not in canonical mode.
            if (ch == '\b' || ch == 0x7f) {
                if (!buffer_.empty()) {
                    buffer_.pop_back();
                }
                continue;
            }

            buffer_.push_back(ch);
            continue;
        }

        if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
            return false;
        }
        return false;
    }
}
