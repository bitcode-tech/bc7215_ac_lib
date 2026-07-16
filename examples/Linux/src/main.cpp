/*
 * BC7215A Linux demo main program
 *
 * This file implements the terminal user interface for the Linux port.
 * It keeps the same menu flow as the ESP-IDF demo, but uses stdin/stdout
 * instead of an ESP-IDF monitor console. Menu commands are line based:
 * the command takes effect only after the user presses Enter.
 */
#include <cctype>
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <strings.h>
#include <unistd.h>

#include "bc7215ac.hpp"
#include "console_line.hpp"

// Main loop delay. The menu jobs are written as small state machines, so the
// program can keep polling the BC7215A without blocking the whole process.
static constexpr uint32_t kLoopIntervalMs = 10;

// Maximum time to wait for an IR transmission to finish before reporting a timeout.
static constexpr uint32_t kTxTimeoutMs = 3000;

// Text tables used when printing decoded or user-selected AC parameters.
// The last item in each table is used as a safe fallback for invalid values.
static const char* MODES[] = { "Auto", "Cool", "Heat", "Dry", "Fan", "Keep", "n/a" };
static const char* FANSPEED[] = { "Auto", "Low", "Med", "High", "Keep", "n/a" };
static const char* PRESSED_KEY[] = { "Temp +", "Temp -", "Mode", "Fan Speed", "Keep" };
static const char* PWR_STATUS[] = { "OFF", "ON", "TOG", "n/a" };

// Top-level menu state. Each value corresponds to one major function in the
// demo, such as pairing, AC control, save/restore, or IR parsing.
enum class L1State {
    ChooseUnit,
    MainMenu,
    Capture,
    AcControl,
    Backup,
    Restore,
    FindNext,
    LoadPredef,
    IrParsing,
};

// Secondary state used inside each menu job. This avoids long blocking
// functions and makes it easy to keep returning to the main polling loop.
enum class L2State {
    Step1,
    Step2,
    Step3,
    Step4,
    Step5,
};

// The AC controller is created after command-line options are parsed, because
// the serial device path and DTR polarity are supplied by the user.
static std::unique_ptr<bc7215::BC7215AC> ac;

// Line-based stdin reader. The terminal itself handles backspace editing; this
// object only fetches complete lines after Enter is pressed.
static ConsoleLineReader console;

// File used to persist pairing information on Linux instead of ESP-IDF NVS.
static std::string store_path = "bc7215_ac_saved.bin";

static L1State main_state = L1State::ChooseUnit;
static L2State l2_state = L2State::Step1;
static L2State go_back_state = L2State::Step1;

// Runtime variables shared by menu jobs. They are intentionally kept at file
// scope to mirror the simple state-machine style of the original ESP-IDF demo.
static uint64_t start_time_ms = 0;
static const bc7215DataVarPkt_t* data_pkt = nullptr;
static const bc7215FormatPkt_t* format_pkt = nullptr;
static uint8_t ir_status = 0;
static bc7215DataMaxPkt_t ir_data = {};
static bc7215FormatPkt_t ir_format = {};


// Return a monotonic millisecond timestamp for timeouts and capture idle detection.
static uint64_t now_ms()
{
    timespec ts {};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000ULL +
           static_cast<uint64_t>(ts.tv_nsec) / 1000000ULL;
}


// Small sleep helper used by the cooperative menu loop.
static void delay_ms(uint32_t ms)
{
    usleep(static_cast<useconds_t>(ms) * 1000U);
}


// Remove leading and trailing whitespace from a console input line.
static std::string trim(const std::string& s)
{
    size_t first = 0;
    while (first < s.size() && std::isspace(static_cast<unsigned char>(s[first]))) {
        ++first;
    }
    size_t last = s.size();
    while (last > first && std::isspace(static_cast<unsigned char>(s[last - 1]))) {
        --last;
    }
    return s.substr(first, last - first);
}


// Drop any already-typed console input before showing a new prompt.
static void clear_console_input()
{
    console.clear_buffer();
}


// Read one completed input line and use its first non-space character as a menu command.
static bool read_menu_char(char& out)
{
    std::string line;
    if (!console.read_line(line)) {
        return false;
    }

    line = trim(line);
    if (line.empty()) {
        out = '\n';
        return true;
    }

    out = line[0];
    return true;
}


// Read one completed input line into a C buffer for sscanf-style parsing.
static bool read_console_line(char* out, size_t out_size)
{
    std::string line;
    if (!console.read_line(line)) {
        return false;
    }

    if (out != nullptr && out_size > 0) {
        std::snprintf(out, out_size, "%s", trim(line).c_str());
    }
    return true;
}


// Calculate how many bytes must be saved for a variable-length BC7215 data packet.
static size_t data_packet_storage_size(const bc7215DataVarPkt_t* pkt)
{
    if (pkt == nullptr) {
        return 0;
    }
    return static_cast<size_t>((pkt->bitLen + 7) / 8 + sizeof(uint16_t));
}


// Resolve normal and combined packets to the actual data packet used for printing.
static const bc7215DataVarPkt_t* printable_data_packet(const bc7215DataVarPkt_t* pkt)
{
    if (pkt == nullptr) {
        return nullptr;
    }
    if (pkt->bitLen == 0) {
        const auto* combined = reinterpret_cast<const bc7215CombinedMsg_t*>(pkt);
        return combined->body.msg.datPkt;
    }
    return pkt;
}


// Print a raw byte buffer in hexadecimal form.
static void print_data(const void* data, size_t len)
{
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) {
        std::printf("%02X ", bytes[i]);
    }
    std::printf("\n");
}


// Print only the IR payload bytes of a BC7215 data packet.
static void print_packet_payload(const bc7215DataVarPkt_t* pkt)
{
    pkt = printable_data_packet(pkt);
    if (pkt == nullptr) {
        std::printf("(null)\n");
        return;
    }
    print_data(pkt->data, (pkt->bitLen + 7) / 8);
}


// On-disk header for saved pairing data. The layout is packed so files are
// stable across x86 and embedded Linux builds.
#pragma pack(push, 1)
struct SavedHeader {
    char magic[8];
    uint8_t unit;       // 1=Celsius, 0=Fahrenheit
	uint8_t status;
    uint16_t format_len;
    uint16_t data_len;
};
#pragma pack(pop)


// Save the initialized AC protocol information to a small binary file.
static bool save_ac_config()
{
    if (!ac->init_ok) {
        return false;
    }

	uint8_t status = ac->status_byte();
    format_pkt = ac->format_packet();
    data_pkt = ac->data_packet();
    if (format_pkt == nullptr || data_pkt == nullptr) {
        return false;
    }

    const size_t data_len = data_packet_storage_size(data_pkt);
    if (data_len == 0 || data_len > sizeof(bc7215DataMaxPkt_t)) {
        std::printf("Invalid data packet size: %u\n", static_cast<unsigned>(data_len));
        return false;
    }

    SavedHeader header {};
    std::memcpy(header.magic, "BCACLIN1", 8);
    header.unit = ac->is_celsius() ? 1 : 0;
	header.status = status;
    header.format_len = static_cast<uint16_t>(sizeof(bc7215FormatPkt_t));
    header.data_len = static_cast<uint16_t>(data_len);

    std::ofstream ofs(store_path.c_str(), std::ios::binary | std::ios::trunc);
    if (!ofs) {
        std::printf("Open save file failed: %s\n", store_path.c_str());
        return false;
    }

    ofs.write(reinterpret_cast<const char*>(&header), sizeof(header));
    ofs.write(reinterpret_cast<const char*>(format_pkt), sizeof(bc7215FormatPkt_t));
    ofs.write(reinterpret_cast<const char*>(data_pkt), static_cast<std::streamsize>(data_len));
    return static_cast<bool>(ofs);
}


// Load previously saved AC protocol information from the binary file.
static bool load_ac_config(uint8_t& status, bc7215FormatPkt_t& format, bc7215DataMaxPkt_t& data, bool& is_celsius)
{
    std::memset(&format, 0, sizeof(format));
    std::memset(&data, 0, sizeof(data));
    is_celsius = true;

    std::ifstream ifs(store_path.c_str(), std::ios::binary);
    if (!ifs) {
        std::printf("No saved AC configuration found: %s\n", store_path.c_str());
        return false;
    }

    SavedHeader header {};
    ifs.read(reinterpret_cast<char*>(&header), sizeof(header));
    if (!ifs || std::memcmp(header.magic, "BCACLIN1", 8) != 0) {
        std::printf("Saved file has invalid magic: %s\n", store_path.c_str());
        return false;
    }

    if (header.format_len != sizeof(bc7215FormatPkt_t) ||
        header.data_len == 0 ||
        header.data_len > sizeof(bc7215DataMaxPkt_t)) {
        std::printf("Saved file has invalid sizes. format=%u data=%u\n",
                    static_cast<unsigned>(header.format_len),
                    static_cast<unsigned>(header.data_len));
        return false;
    }

    ifs.read(reinterpret_cast<char*>(&format), sizeof(bc7215FormatPkt_t));
    ifs.read(reinterpret_cast<char*>(&data), header.data_len);
    if (!ifs) {
        std::printf("Saved file read failed: %s\n", store_path.c_str());
        return false;
    }
	status = header.status;
    is_celsius = (header.unit != 0);
    return true;
}


// Display temperature-unit selection menu.
static void show_unit_menu()
{
    std::printf("Please choose your AC's temperature unit, then press Enter:\n");
    std::printf(" 1. Celsius\n");
    std::printf(" 2. Fahrenheit\n\n> ");
}


// Display the top-level demo menu.
static void show_main_menu()
{
    std::printf("\n\n*****************************************************\n");
    std::printf("* Welcome to BC7215A Universal AC Controller Demo   *\n");
    std::printf("*****************************************************\n");
    std::printf("AC Library Version: %s\n", ac->lib_version());
    std::printf("Current AC control library status: %s\n",
                ac->init_ok ? "***INITIALIZED***" : "Not initialized (must be paired before use)");
    std::printf("Temperature Unit: %s\n", ac->is_celsius() ? "Celsius" : "Fahrenheit");
    std::printf("Please select, then press Enter:\n");
    std::printf(" 1. Pairing with AC\n");
    std::printf(" 2. Control air conditioner\n");
    std::printf(" 3. Save pairing data\n");
    std::printf(" 4. Read saved data and pair with it\n");
    std::printf(" 5. Try next match\n");
    std::printf(" 6. Load predefined protocol\n");
    std::printf(" 7. Parse IR signal\n");
    std::printf(" 8. Set AC temp unit\n\n> ");
}


// Display the AC control submenu.
static void show_ctrl_menu()
{
    std::printf("\nAC Control, please select, then press Enter:\n");
    std::printf(" 1. Set AC parameters\n");
    std::printf(" 2. Power on\n");
    std::printf(" 3. Power off\n");
    std::printf(" 4. Return to upper menu\n\n> ");
}


// Display valid AC parameter ranges and input format.
static void show_param_menu()
{
    if (ac->is_celsius()) {
        std::printf(" *** AC parameter adjustment ***\n");
        std::printf("Format: temperature,mode,fan,key  e.g. 24,1,2,0\n");
        std::printf("Fewer parameters are allowed, e.g. 18,2 means 18C Heating, fan/key unchanged.\n");
        std::printf("Temperature(C)  Mode   Fan     Key\n");
        std::printf(" Range: 16-30  0-Auto  0-Auto  0-Temp+\n");
    } else {
        std::printf(" *** AC parameter adjustment ***\n");
        std::printf("Format: temperature,mode,fan,key  e.g. 75,1,2,0\n");
        std::printf("Fewer parameters are allowed, e.g. 75,2 means 75F Heating, fan/key unchanged.\n");
        std::printf("Temperature(F)  Mode   Fan     Key\n");
        std::printf(" Range: 60~88  0-Auto  0-Auto  0-Temp+\n");
    }
    std::printf("               1-Cool  1-Low   1-Temp-\n");
    std::printf("               2-Heat  2-Med   2-Mode\n");
    std::printf("               3-Dry   3-High  3-Fan\n");
    std::printf("               4-Fan\n");
    std::printf("Values outside the range mean keep current state. Enter 'exit' to return.\n\n> ");
}


// Display predefined AC protocol entries supplied by the AC library.
static void show_predef_menu()
{
    std::printf("\nSome protocols are not supported for direct decoding by BC7215A.\n");
    std::printf("When direct sampling fails, try predefined data.\n");
    std::printf("Please select, then press Enter:\n");
    for (uint8_t i = 0; i < ac->predefined_count(); ++i) {
        std::printf(" %u. %s\n", static_cast<unsigned>(i), ac->predefined_name(i));
    }
    std::printf("\n> ");
}


// State machine for selecting Celsius or Fahrenheit operation.
static void choose_unit_job()
{
    char ch = 0;
    switch (l2_state) {
    case L2State::Step1:
        show_unit_menu();
        clear_console_input();
        l2_state = L2State::Step2;
        break;
    case L2State::Step2:
        if (read_menu_char(ch)) {
            if (ch == '1') {
                ac->set_celsius();
                std::printf("AC has been set to Celsius.\n");
                l2_state = L2State::Step3;
            } else if (ch == '2') {
                ac->set_fahrenheit();
                std::printf("AC has been set to Fahrenheit.\n");
                l2_state = L2State::Step3;
            } else {
                l2_state = L2State::Step1;
            }
        }
        break;
    case L2State::Step3:
        main_state = L1State::MainMenu;
        l2_state = L2State::Step1;
        break;
    default:
        break;
    }
}


// State machine for the main menu.
static void main_menu_job()
{
    char ch = 0;
    switch (l2_state) {
    case L2State::Step1:
        show_main_menu();
        clear_console_input();
        l2_state = L2State::Step2;
        break;
    case L2State::Step2:
        if (read_menu_char(ch)) {
            switch (ch) {
            case '1': main_state = L1State::Capture; l2_state = L2State::Step1; break;
            case '2':
                if (ac->init_ok) { main_state = L1State::AcControl; l2_state = L2State::Step1; }
                else { std::printf("\nAC control library not initialized yet, please pair first.\n> "); }
                break;
            case '3':
                if (ac->init_ok) { main_state = L1State::Backup; l2_state = L2State::Step1; }
                else { std::printf("\nData can only be saved after successful pairing.\n> "); }
                break;
            case '4': main_state = L1State::Restore; l2_state = L2State::Step1; break;
            case '5':
                if (ac->init_ok) { main_state = L1State::FindNext; l2_state = L2State::Step1; }
                else { std::printf("\nThis function is only available after pairing.\n> "); }
                break;
            case '6': main_state = L1State::LoadPredef; l2_state = L2State::Step1; break;
            case '7': main_state = L1State::IrParsing; l2_state = L2State::Step1; break;
            case '8': main_state = L1State::ChooseUnit; l2_state = L2State::Step1; break;
            default: main_state = L1State::MainMenu; l2_state = L2State::Step1; break;
            }
        }
        break;
    default:
        break;
    }
}


// Pairing workflow: capture one remote-control command and initialize the AC library.
static void capture_job()
{
    char ch = 0;
    switch (l2_state) {
    case L2State::Step1:
        std::printf("\nNow performing IR AC pairing.\n");
        if (ac->is_celsius()) {
            std::printf("Please set AC remote to 25C, Cool, Fan, then press Enter.\n> ");
        } else {
            std::printf("Please set AC remote to 78F, Cool, Fan, then press Enter.\n> ");
        }
        clear_console_input();
        l2_state = L2State::Step2;
        break;
    case L2State::Step2:
        if (read_menu_char(ch)) {
            std::printf("Now aim at IR receiver and press Fan button on remote.\n");
            std::printf("Will automatically proceed after receiving signal...\n");
            ac->start_capture(1);
            l2_state = L2State::Step3;
        }
        break;
    case L2State::Step3:
        if (ac->signal_captured()) {
            ac->stop_capture();
            if (ac->init()) {
                std::printf("Received data: ");
                data_pkt = ac->data_packet();
                print_packet_payload(data_pkt);
                std::printf("AC control library initialization using received data SUCCESS.\n");
            } else {
                std::printf("AC control library initialization FAILED.\n");
            }
            l2_state = L2State::Step4;
        }
        break;
    case L2State::Step4:
        std::printf("Press Enter to return to the main menu.\n> ");
        clear_console_input();
        l2_state = L2State::Step5;
        break;
    case L2State::Step5:
        if (read_menu_char(ch)) {
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;
    default:
        break;
    }
}


// AC control workflow: set parameters, power on, or power off.
static void ac_control_job()
{
    char ch = 0;
    char input[160] = {};

    switch (l2_state) {
    case L2State::Step1:
        show_ctrl_menu();
        clear_console_input();
        l2_state = L2State::Step2;
        break;
    case L2State::Step2:
        if (read_menu_char(ch)) {
            switch (ch) {
            case '1': show_param_menu(); clear_console_input(); l2_state = L2State::Step3; break;
            case '2':
                std::printf("Sending AC power on command\n");
                start_time_ms = now_ms();
                data_pkt = ac->on();
                std::printf("Sending data: "); print_packet_payload(data_pkt);
                go_back_state = L2State::Step2;
                l2_state = L2State::Step4;
                break;
            case '3':
                std::printf("Sending AC power off command\n");
                start_time_ms = now_ms();
                data_pkt = ac->off();
                std::printf("Sending data: "); print_packet_payload(data_pkt);
                go_back_state = L2State::Step2;
                l2_state = L2State::Step4;
                break;
            case '4':
                main_state = L1State::MainMenu;
                l2_state = L2State::Step1;
                break;
            default:
                std::printf("> ");
                break;
            }
        }
        break;
    case L2State::Step3:
        if (read_console_line(input, sizeof(input))) {
            if (input[0] == '\0') {
                std::printf("> ");
                break;
            }
            if (strcasecmp(input, "exit") == 0) {
                l2_state = L2State::Step1;
                break;
            }

            int t = -1, m = -1, f = -1, k = -1;
            std::sscanf(input, "%d,%d,%d,%d", &t, &m, &f, &k);
            if (m < 0 || m > 4) { m = 5; }
            if (f < 0 || f > 3) { f = 4; }
            if (k < 0 || k > 3) { k = 4; }

            std::printf("Sending command to set AC to: ");
            if (ac->is_celsius() && t >= 16 && t <= 30) {
                std::printf("%dC, Mode: ", t);
            } else if (!ac->is_celsius() && t >= 60 && t <= 88) {
                std::printf("%dF, Mode: ", t);
            } else {
                std::printf("(keep Temp), Mode: ");
            }
            std::printf("%s, Fan Speed: %s, Key Press: %s\n", MODES[m], FANSPEED[f], PRESSED_KEY[k]);

            start_time_ms = now_ms();
            data_pkt = ac->set_to(t, m, f, k);
            std::printf("Sending data: "); print_packet_payload(data_pkt);
            go_back_state = L2State::Step3;
            l2_state = L2State::Step4;
        }
        break;
    case L2State::Step4:
        if (!ac->is_busy() || (now_ms() - start_time_ms > kTxTimeoutMs)) {
            if (now_ms() - start_time_ms > kTxTimeoutMs) {
                std::printf("Transmission timeout\n");
            }
            std::printf("Transmission complete.\n\n");
            clear_console_input();
            l2_state = go_back_state;
            if (l2_state == L2State::Step3) {
                std::printf("Please continue input. Enter 'exit' to return.\n> ");
            } else {
                std::printf("> ");
            }
        }
        break;
    default:
        break;
    }
}


// Save current pairing information to disk.
static void backup_job()
{
    char ch = 0;
    switch (l2_state) {
    case L2State::Step1:
        if (ac->init_ok && save_ac_config()) {
			ir_status = ac->status_byte();
            format_pkt = ac->format_packet();
            data_pkt = ac->data_packet();
			std::printf("Status byte: 0x%02X", ir_status);
            std::printf("\nFormat info: "); print_data(format_pkt, sizeof(bc7215FormatPkt_t));
            std::printf("Data: "); print_packet_payload(data_pkt);
            std::printf("Information saved to %s\n", store_path.c_str());
        } else {
            std::printf("\nSaving failed.\n");
        }
        std::printf("Press Enter to continue.\n> ");
        clear_console_input();
        l2_state = L2State::Step2;
        break;
    case L2State::Step2:
        if (read_menu_char(ch)) { main_state = L1State::MainMenu; l2_state = L2State::Step1; }
        break;
    default:
        break;
    }
}


// Restore pairing information from disk and reinitialize the AC library.
static void restore_job()
{
    char ch = 0;
    bool is_celsius = true;

    switch (l2_state) {
    case L2State::Step1:
        if (!load_ac_config(ir_status, ir_format, ir_data, is_celsius)) {
            std::printf("Restore failed. Press Enter to continue.\n> ");
            clear_console_input();
            l2_state = L2State::Step2;
            break;
        }
        std::printf("\nUsing saved configuration from %s\n", store_path.c_str());
		std::printf("Status byte: 0x%02X\n", ir_status);
        std::printf("Format info: "); print_data(&ir_format, sizeof(bc7215FormatPkt_t));
        std::printf("Data: "); print_data(ir_data.data, (ir_data.bitLen + 7) / 8);
        if (is_celsius) { ac->set_celsius(); } else { ac->set_fahrenheit(); }
        if (ac->init(ir_status, ir_data, ir_format)) {
            std::printf("AC control library initialization SUCCESS.\n");
        } else {
            std::printf("AC control library initialization failed.\n");
        }
        std::printf("Press Enter to continue.\n> ");
        clear_console_input();
        l2_state = L2State::Step2;
        break;
    case L2State::Step2:
        if (read_menu_char(ch)) { main_state = L1State::MainMenu; l2_state = L2State::Step1; }
        break;
    default:
        break;
    }
}


// Ask the AC library to try the next matching protocol candidate.
static void find_next_job()
{
    char ch = 0;
    switch (l2_state) {
    case L2State::Step1:
        if (ac->match_next()) {
            std::printf("Next protocol match successful.\n");
        } else {
            std::printf("No other matching protocols; re-initialization is required.\n");
        }
        std::printf("Press Enter to continue.\n> ");
        clear_console_input();
        l2_state = L2State::Step2;
        break;
    case L2State::Step2:
        if (read_menu_char(ch)) { main_state = L1State::MainMenu; l2_state = L2State::Step1; }
        break;
    default:
        break;
    }
}


// Initialize the AC library from one of the predefined protocols.
static void load_predef_job()
{
    char ch = 0;
    switch (l2_state) {
    case L2State::Step1:
        show_predef_menu();
        clear_console_input();
        l2_state = L2State::Step2;
        break;
    case L2State::Step2:
        if (read_menu_char(ch)) {
            int choice = -1;
            if (ch >= '0' && ch <= '9') { choice = ch - '0'; }
            std::printf("Selected option %d\n", choice);
            if (choice >= 0 && choice <= 255 && ac->init_predefined(static_cast<uint8_t>(choice))) {
                std::printf("Format: "); print_data(ac->format_packet(), sizeof(bc7215FormatPkt_t));
                std::printf("Data: ");
                const bc7215DataVarPkt_t* pkt = ac->data_packet();
                print_data(pkt, data_packet_storage_size(pkt));
                std::printf("Initialization successful. Press Enter to continue.\n> ");
            } else {
                std::printf("Initialization failed. Press Enter to continue.\n> ");
            }
            l2_state = L2State::Step3;
        }
        break;
    case L2State::Step3:
        if (read_menu_char(ch)) { main_state = L1State::MainMenu; l2_state = L2State::Step1; }
        break;
    default:
        break;
    }
}


// Receive IR commands from the paired remote and decode temp/mode/fan/power.
static void ir_parsing_job()
{
    char ch = 0;
    int t = -1, m = -1, f = -1, p = -1;

    switch (l2_state) {
    case L2State::Step1:
        if (ac->init_ok) {
            std::printf("\nReceiving IR signal and parsing Temperature, Mode, Fan Speed, and Power.\n");
            std::printf("BC7215A is now in RX mode. Type anything and press Enter to exit.\n");
            ac->start_capture(0);
            clear_console_input();
            l2_state = L2State::Step2;
        } else {
            std::printf("\nThis function is only available after pairing.\n");
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;
    case L2State::Step2:
        if (ac->signal_captured()) {
            ac->stop_capture();
            std::printf("Parsing Result: Temp: ");
            if (ac->parse(t, m, f, p)) {
                if (ac->is_celsius()) {
                    if (t < 16 || t > 30) { std::printf("n/a, Mode: "); }
                    else { std::printf("%dC, Mode: ", t); }
                } else {
                    if (t < 60 || t > 88) { std::printf("n/a, Mode: "); }
                    else { std::printf("%dF, Mode: ", t); }
                }
                if (m < 0 || m > 4) { m = 6; }
                if (f < 0 || f > 3) { f = 5; }
                if (p < 0 || p > 2) { p = 3; }
                std::printf("%s, Fan Speed: %s, Power: %s\n", MODES[m], FANSPEED[f], PWR_STATUS[p]);
            } else {
                std::printf("Parsing failed\n");
            }
            ac->start_capture(0);
        }
        if (read_menu_char(ch)) {
            ac->stop_capture();
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;
    default:
        break;
    }
}


// Print command-line usage and hardware wiring summary.
static void usage(const char* argv0)
{
    std::printf("Usage: %s [options]\n", argv0);
    std::printf("Options:\n");
    std::printf("  -d, --dev PATH        Serial device, default /dev/ttyUSB0\n");
    std::printf("  -s, --store PATH      Save/restore file, default bc7215_ac_saved.bin\n");
    std::printf("      --mod-reverse     Reverse DTR polarity for BC7215 MOD\n");
    std::printf("  -h, --help            Show this help\n");
    std::printf("\nHardware mapping: TX->BC7215 RX, RX<-BC7215 TX, CTS<-BUSY, DTR->MOD, GND->GND.\n");
}


// Program entry point: parse options, initialize console and BC7215A, then run the menu loop.
int main(int argc, char** argv)
{
    std::string serial_dev = "/dev/ttyUSB0";
    bool mod_reverse = false;

    // Parse command-line options before constructing BC7215AC, because the
    // constructor needs the final serial device path and MOD polarity option.
    for (int i = 1; i < argc; ++i) {
        const char* arg = argv[i];
        if ((std::strcmp(arg, "-d") == 0 || std::strcmp(arg, "--dev") == 0) && i + 1 < argc) {
            serial_dev = argv[++i];
        } else if ((std::strcmp(arg, "-s") == 0 || std::strcmp(arg, "--store") == 0) && i + 1 < argc) {
            store_path = argv[++i];
        } else if (std::strcmp(arg, "--mod-reverse") == 0) {
            mod_reverse = true;
        } else if (std::strcmp(arg, "-h") == 0 || std::strcmp(arg, "--help") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            usage(argv[0]);
            return 1;
        }
    }

    if (!console.begin()) {
        std::fprintf(stderr, "Failed to initialize stdin console.\n");
        return 1;
    }

    // Dynamically create the controller after all runtime options are known.
    // unique_ptr::reset() takes ownership and will delete the object automatically.
    ac.reset(new bc7215::BC7215AC(serial_dev.c_str(), mod_reverse));

    if (!ac->begin()) {
        std::fprintf(stderr, "BC7215AC begin failed: %s\n", ac->last_error());
        return 1;
    }

    // Do a short RX/TX mode round-trip during startup. This verifies that the
    // BC7215A can be switched through the wrapper before the user enters the menu.
    ac->start_capture(0);
    ac->stop_capture();

    std::printf("\nSystem initialized.\n");
    std::printf("Serial device: %s\n", serial_dev.c_str());
    std::printf("MOD control: DTR%s\n", mod_reverse ? " (reversed polarity)" : " (default polarity)");
    std::printf("Store file: %s\n", store_path.c_str());
    std::printf("All menu commands take effect only after Enter. Backspace editing is handled by the terminal.\n");

    // Cooperative main loop. Each job performs only a small step and returns,
    // so serial reception and command completion can be polled regularly.
    while (true) {
        switch (main_state) {
        case L1State::ChooseUnit: choose_unit_job(); break;
        case L1State::MainMenu: main_menu_job(); break;
        case L1State::Capture: capture_job(); break;
        case L1State::AcControl: ac_control_job(); break;
        case L1State::Backup: backup_job(); break;
        case L1State::Restore: restore_job(); break;
        case L1State::FindNext: find_next_job(); break;
        case L1State::LoadPredef: load_predef_job(); break;
        case L1State::IrParsing: ir_parsing_job(); break;
        default: break;
        }
        delay_ms(kLoopIntervalMs);
    }

    return 0;
}
