#pragma once

#include <string>

class ConsoleLineReader {
public:
    ConsoleLineReader() = default;
    ~ConsoleLineReader();

    bool begin();
    void end();

    bool read_line(std::string& out);
    void clear_buffer();

private:
    int old_flags_ = -1;
    bool active_ = false;
    std::string buffer_;
    bool last_was_cr_ = false;
};
