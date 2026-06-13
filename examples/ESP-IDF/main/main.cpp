#include <cctype>
#include <cstdio>
#include <cstring>
#include <strings.h>
#include <unistd.h>

#include "bc7215.hpp"
#include "bc7215ac.hpp"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/*
 * BC7215A Universal AC Controller demo for ESP-IDF.
 *
 * This file intentionally keeps the application logic in app_main.cpp so the
 * demo is easy to follow from a serial terminal. The UI is implemented as a
 * cooperative two-level state machine:
 *   - L1State selects the current top-level feature/menu.
 *   - L2State records the current step inside that feature.
 *
 * No menu function should block while waiting for keyboard input or IR data.
 * app_main() calls the active job repeatedly, and each job advances only when
 * its required input/event is ready.
 */
static QueueHandle_t console_rx_queue = nullptr;

static constexpr size_t kConsoleQueueLen = 256;
static constexpr uint32_t kConsolePollDelayMs = 10;
// -----------------------------
// Hardware configuration
// -----------------------------
// BC7215 serial: ESP RX=GPIO25, ESP TX=GPIO33, BC7215 BUSY -> ESP CTS=GPIO26, MOD=GPIO27.
// Do not use UART0 here if UART0 is your ESP-IDF console.
static constexpr uart_port_t BC_UART = UART_NUM_1;
static constexpr gpio_num_t  BC_TX_PIN = GPIO_NUM_25;              // ESP TX -> BC7215 RX
static constexpr gpio_num_t  BC_RX_PIN = GPIO_NUM_33;              // ESP RX <- BC7215 TX
static constexpr gpio_num_t  BC_BUSY_PIN = GPIO_NUM_26;        // BC7215 BUSY -> ESP UART CTS
static constexpr gpio_num_t  BC_MOD_PIN = GPIO_NUM_27;             // ESP output -> BC7215 MOD

// Main polling interval for the cooperative state machine.
// Keep this short enough for responsive console input, but long enough to avoid a busy loop.
static constexpr uint32_t kLoopIntervalMs = 10;

// Maximum time to wait for BC7215A to finish one transmission before reporting timeout.
static constexpr uint32_t kTxTimeoutMs = 3000;

// -----------------------------
// NVS backup configuration
// -----------------------------
// One NVS namespace is used for all data needed to restore a paired AC.
static constexpr const char* kNvsNamespace = "bc7215_ac";
static constexpr const char* kNvsKeyFormat = "format";        // bc7215FormatPkt_t
static constexpr const char* kNvsKeyData = "data";            // bc7215DataVarPkt_t-compatible binary blob
static constexpr const char* kNvsKeyUnit = "unit";            // 1=Celsius, 0=Fahrenheit

// -----------------------------
// Menu text
// -----------------------------
// String tables are indexed by library result values. Extra entries represent "keep current"
// or "not available" so out-of-range values can be printed safely after normalization.
static const char* MODES[] = { "Auto", "Cool", "Heat", "Dry", "Fan", "Keep", "n/a" };
static const char* FANSPEED[] = { "Auto", "Low", "Med", "High", "Keep", "n/a" };
static const char* PRESSED_KEY[] = { "Temp +", "Temp -", "Mode", "Fan Speed", "Keep" };
static const char* PWR_STATUS[] = { "OFF", "ON", "TOG", "n/a" };

// -----------------------------
// State machine
// -----------------------------
// Top-level application states. Each value corresponds to one menu feature.
// The actual work is performed by the matching xxx_job() function below.
enum class L1State
{
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

// Sub-state used inside the active L1State. This avoids blocking calls such as
// "wait until Enter is pressed" and makes every operation progress step by step.
enum class L2State
{
    Step1,
    Step2,
    Step3,
    Step4,
    Step5,
    Step6,
    Step7,
};

// -----------------------------
// Global objects
// -----------------------------
// These are global/static so their buffers do not consume app_main task stack.
static bc7215::BC7215AC ac(BC_UART, BC_RX_PIN, BC_TX_PIN, BC_BUSY_PIN, BC_MOD_PIN);

// -----------------------------
// Global runtime variables
// -----------------------------
// Current top-level feature and current step inside that feature.
static L1State main_state = L1State::ChooseUnit;
static L2State l2_state = L2State::Step1;

// Used by AC control after sending a command: when transmission finishes, return
// either to the control menu or to the parameter input screen.
static L2State go_back_state = L2State::Step1;

// Shared runtime data. Packet pointers point to buffers owned by the BC7215AC object,
// while ir_data/ir_format are local copies used when restoring from NVS.
static uint64_t                  start_time_ms = 0;
static const bc7215DataVarPkt_t* data_pkt = nullptr;
static const bc7215FormatPkt_t*  format_pkt = nullptr;
static bc7215DataMaxPkt_t        ir_data = {};
static bc7215FormatPkt_t         ir_format = {};

// -----------------------------
// Console input helpers
// -----------------------------
// Input line buffer for the non-blocking console reader. The code reads from
// stdin byte by byte and assembles a full line only after Enter is pressed.
static char   line_buf[160] = {};
static size_t line_len = 0;

// Return a monotonic millisecond timestamp based on esp_timer.
static uint64_t now_ms() { return static_cast<uint64_t>(esp_timer_get_time() / 1000ULL); }

// Yield the FreeRTOS task instead of using a CPU-burning delay loop.
static void delay_ms(uint32_t ms) { vTaskDelay(pdMS_TO_TICKS(ms)); }

// Console RX task:
//   - reads bytes from the ESP-IDF primary console, whether it is UART or USB Serial/JTAG;
//   - converts CR, LF, and CRLF into one '\n';
//   - does not echo characters. Menu items are accepted immediately, while line input
//     performs its own echo in read_console_line().
static void console_rx_task(void* arg)
{
    bool last_was_cr = false;

    while (true)
    {
        uint8_t ch = 0;
        const ssize_t n = read(STDIN_FILENO, &ch, 1);

        if (n == 1)
        {
            if (ch == '\r')
            {
                const uint8_t nl = '\n';
                xQueueSend(console_rx_queue, &nl, 0);
                last_was_cr = true;
                continue;
            }

            if (ch == '\n')
            {
                if (last_was_cr)
                {
                    last_was_cr = false;
                    continue;
                }

                const uint8_t nl = '\n';
                xQueueSend(console_rx_queue, &nl, 0);
                continue;
            }

            last_was_cr = false;
            xQueueSend(console_rx_queue, &ch, 0);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(kConsolePollDelayMs));
    }
}

static esp_err_t setup_stdio_console()
{
    setvbuf(stdin, nullptr, _IONBF, 0);
    setvbuf(stdout, nullptr, _IONBF, 0);

    console_rx_queue = xQueueCreate(kConsoleQueueLen, sizeof(uint8_t));
    if (console_rx_queue == nullptr)
    {
        return ESP_ERR_NO_MEM;
    }

    const BaseType_t ok = xTaskCreate(
        console_rx_task,
        "console_rx",
        3072,
        nullptr,
        5,
        nullptr
    );

    return ok == pdPASS ? ESP_OK : ESP_FAIL;
}

static int read_console_char()
{
    if (console_rx_queue == nullptr)
    {
        return -1;
    }

    uint8_t ch = 0;
    if (xQueueReceive(console_rx_queue, &ch, 0) == pdTRUE)
    {
        return ch;
    }

    return -1;
}

// Clear the partially typed command line.
static void reset_line_reader()
{
    line_len = 0;
    line_buf[0] = '\0';
}

// Remove any already-buffered keyboard input before showing a new prompt.
// This prevents a previous key press from being consumed by the next menu state.
static void clear_console_input()
{
    if (console_rx_queue != nullptr)
    {
        uint8_t dummy = 0;
        while (xQueueReceive(console_rx_queue, &dummy, 0) == pdTRUE)
        {
            // Drain queued input.
        }
    }

    reset_line_reader();
}

// Line input editor used only where a complete text line is required.
// For example: AC parameter input such as "24,1,2,0".
// This function echoes locally using printf(), which works well with a UART console.
// On USB Serial/JTAG console, character echo may be delayed by the USB console backend.
static bool read_console_line(char* out, size_t out_size)
{
    int ch;

    while ((ch = read_console_char()) >= 0)
    {
        if (ch == '\n')
        {
            std::printf("\r\n");

            line_buf[line_len] = '\0';

            if (out != nullptr && out_size > 0)
            {
                std::snprintf(out, out_size, "%s", line_buf);
            }

            reset_line_reader();
            return true;
        }

        if (ch == '\b' || ch == 0x7F)
        {
            if (line_len > 0)
            {
                --line_len;
                line_buf[line_len] = '\0';
                std::printf("\b \b");
                fflush(stdout);
            }
            continue;
        }

        if (std::isprint(static_cast<unsigned char>(ch)))
        {
            if (line_len + 1 < sizeof(line_buf))
            {
                line_buf[line_len++] = static_cast<char>(ch);
                line_buf[line_len] = '\0';

                std::printf("%c", ch);
                fflush(stdout);
            }
        }
    }

    return false;
}

// Read one menu key immediately. This is used by ordinary menus and "press any key"
// prompts, so it does not wait for Enter. The key is echoed with a newline to keep
// menu output readable on a UART console.
static bool read_menu_char(char& out)
{
    const int ch = read_console_char();
    if (ch < 0)
    {
        return false;
    }

    out = static_cast<char>(ch);

    if (ch == '\n')
    {
        std::printf("\r\n");
    }
    else if (std::isprint(static_cast<unsigned char>(ch)))
    {
        std::printf("%c\r\n", ch);
    }
    else
    {
        std::printf("\r\n");
    }

    return true;
}

// -----------------------------
// Data helpers
// -----------------------------
// Calculate the number of bytes required to store a variable-length data packet.
// The packet contains a 16-bit bitLen field followed by ceil(bitLen / 8) data bytes.
static size_t data_packet_storage_size(const bc7215DataVarPkt_t* pkt)
{
    if (pkt == nullptr)
    {
        return 0;
    }

    return static_cast<size_t>((pkt->bitLen + 7) / 8 + sizeof(uint16_t));
}

// Return the real data packet to print. Some library calls may return a combined
// message wrapper; in that case bitLen is zero and the actual packet is inside it.
static const bc7215DataVarPkt_t* printable_data_packet(const bc7215DataVarPkt_t* pkt)
{
    if (pkt == nullptr)
    {
        return nullptr;
    }

    // bc7215_ac_lib may return a combined message with bitLen == 0.
    if (pkt->bitLen == 0)
    {
        const auto* combined = reinterpret_cast<const bc7215CombinedMsg_t*>(pkt);
        return combined->body.msg.datPkt;
    }

    return pkt;
}

// Print raw bytes in hexadecimal format for debugging and documentation.
static void print_data(const void* data, size_t len)
{
    const auto* bytes = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i)
    {
        std::printf("%02X ", bytes[i]);
    }
    std::printf("\r\n");
}

// Print only the IR payload bytes, not the packet header.
static void print_packet_payload(const bc7215DataVarPkt_t* pkt)
{
    pkt = printable_data_packet(pkt);
    if (pkt == nullptr)
    {
        std::printf("(null)\r\n");
        return;
    }

    print_data(pkt->data, (pkt->bitLen + 7) / 8);
}

// -----------------------------
// NVS helpers
// -----------------------------
// Initialize NVS. If the NVS partition is full or was created by an incompatible
// version, erase it and initialize again.
static esp_err_t init_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

// Save the current pairing result to NVS so the device can restore it after reboot
// without requiring the user to capture the AC remote again.
static bool save_ac_config()
{
    if (!ac.init_ok)
    {
        return false;
    }

    format_pkt = ac.format_packet();
    data_pkt = ac.data_packet();

    if (format_pkt == nullptr || data_pkt == nullptr)
    {
        return false;
    }

    // Store exactly the valid packet length instead of writing the whole maximum buffer.
    const size_t data_len = data_packet_storage_size(data_pkt);
    if (data_len == 0 || data_len > sizeof(bc7215DataMaxPkt_t))
    {
        std::printf("Invalid data packet size: %u\r\n", static_cast<unsigned>(data_len));
        return false;
    }

    nvs_handle_t handle = 0;
    esp_err_t    err = nvs_open(kNvsNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK)
    {
        std::printf("nvs_open failed: %s\r\n", esp_err_to_name(err));
        return false;
    }

    const uint8_t is_celsius = ac.is_celsius() ? 1 : 0;

    err = nvs_set_blob(handle, kNvsKeyFormat, format_pkt, sizeof(bc7215FormatPkt_t));
    if (err == ESP_OK)
    {
        err = nvs_set_blob(handle, kNvsKeyData, data_pkt, data_len);
    }
    if (err == ESP_OK)
    {
        err = nvs_set_u8(handle, kNvsKeyUnit, is_celsius);
    }
    if (err == ESP_OK)
    {
        err = nvs_commit(handle);
    }

    nvs_close(handle);

    if (err != ESP_OK)
    {
        std::printf("NVS save failed: %s\r\n", esp_err_to_name(err));
        return false;
    }

    return true;
}

// Load the saved pairing result from NVS and validate the stored blob sizes before
// passing them back to the BC7215AC library.
static bool load_ac_config(bc7215FormatPkt_t& format, bc7215DataMaxPkt_t& data, bool& is_celsius)
{
    std::memset(&format, 0, sizeof(format));
    std::memset(&data, 0, sizeof(data));
    is_celsius = true;

    nvs_handle_t handle = 0;
    esp_err_t    err = nvs_open(kNvsNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK)
    {
        std::printf("No saved AC configuration found: %s\r\n", esp_err_to_name(err));
        return false;
    }

    size_t format_len = sizeof(bc7215FormatPkt_t);
    err = nvs_get_blob(handle, kNvsKeyFormat, &format, &format_len);
    if (err != ESP_OK || format_len != sizeof(bc7215FormatPkt_t))
    {
        std::printf("Read saved format failed: %s\r\n", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    size_t data_len = 0;
    // Query the saved blob size first, then read it into the fixed maximum packet buffer.
    err = nvs_get_blob(handle, kNvsKeyData, nullptr, &data_len);
    if (err != ESP_OK || data_len == 0 || data_len > sizeof(bc7215DataMaxPkt_t))
    {
        std::printf(
            "Saved data size invalid: err=%s size=%u\r\n", esp_err_to_name(err), static_cast<unsigned>(data_len));
        nvs_close(handle);
        return false;
    }

    err = nvs_get_blob(handle, kNvsKeyData, &data, &data_len);
    if (err != ESP_OK)
    {
        std::printf("Read saved data failed: %s\r\n", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    uint8_t unit = 1;
    err = nvs_get_u8(handle, kNvsKeyUnit, &unit);
    if (err != ESP_OK)
    {
        std::printf("Read saved unit failed: %s\r\n", esp_err_to_name(err));
        nvs_close(handle);
        return false;
    }

    is_celsius = (unit != 0);
    nvs_close(handle);
    return true;
}

// -----------------------------
// Menu display functions
// -----------------------------
// These functions only print menu text. State changes are handled by the xxx_job()
// functions so the UI text remains separate from the control logic.
static void show_unit_menu()
{
    std::printf("Please choose your AC's temperature unit:\r\n");
    std::printf("   1. Celsius\r\n");
    std::printf("   2. Fahrenheit\r\n\r\n");
}

static void show_main_menu()
{
    std::printf("\r\n\r\n*****************************************************\r\n");
    std::printf("* Welcome to BC7215A Universal AC Controller Demo  *\r\n");
    std::printf("*****************************************************\r\n");
    std::printf("AC Library Version: %s\r\n", ac.lib_version());
    std::printf("Current AC control library status: %s\r\n",
        ac.init_ok ? "***INITIALIZED***" : "Not initialized (must be paired with AC before use)");
    std::printf("Temperature Unit: %s\r\n", ac.is_celsius() ? "Celsius" : "Fahrenheit");
    std::printf("Please select:\r\n");
    std::printf("   1. Pairing with AC\r\n");
    std::printf("   2. Control air conditioner\r\n");
    std::printf("   3. Save pairing data\r\n");
    std::printf("   4. Read saved data and pair with it\r\n");
    std::printf("   5. Try next match (if paired successfully but cannot control AC properly)\r\n");
    std::printf("   6. Load predefined protocol\r\n");
    std::printf("   7. Parse IR signal\r\n");
    std::printf("   8. Set AC temp unit\r\n\r\n");
}

static void show_ctrl_menu()
{
    std::printf("\r\nAC Control, please select:\r\n");
    std::printf("   1. Set AC parameters\r\n");
    std::printf("   2. Power on\r\n");
    std::printf("   3. Power off\r\n");
    std::printf("   4. Return to upper menu\r\n\r\n");
}

static void show_param_menu()
{
    if (ac.is_celsius())
    {
        std::printf(" *** AC parameter adjustment ***\r\n");
        std::printf("Format: 'temperature, mode, fan level, pressed-key', e.g.: 24,1,2,0\r\n");
        std::printf("Fewer parameters are allowed, e.g. '18,2' means 18C Heating, fan/key unchanged.\r\n");
        std::printf("Temperature(C)    Mode           Fan              Key\r\n");
        std::printf(" Range: 16~30      0 - Auto       0 - Auto         0 - Temp +\r\n");
    }
    else
    {
        std::printf(" *** AC parameter adjustment ***\r\n");
        std::printf("Format: 'temperature, mode, fan level, pressed-key', e.g.: 75,1,2,0\r\n");
        std::printf("Fewer parameters are allowed, e.g. '75,2' means 75F Heating, fan/key unchanged.\r\n");
        std::printf("Temperature(F)    Mode           Fan              Key\r\n");
        std::printf(" Range: 60~88      0 - Auto       0 - Auto         0 - Temp +\r\n");
    }

    std::printf("                   1 - Cool       1 - Low          1 - Temp -\r\n");
    std::printf("                   2 - Heat       2 - Med          2 - Mode\r\n");
    std::printf("                   3 - Dry        3 - High         3 - Fan\r\n");
    std::printf("                   4 - Fan\r\n\r\n");
    std::printf(" * Values outside above ranges indicate maintaining current state for that item\r\n");
    std::printf("------------------------------------------------------------------------------------\r\n");
    std::printf("(Note: Limited to settings supported by the controlled AC.)\r\n");
    std::printf("Now please enter AC parameter values: (enter 'exit' to return to upper menu)\r\n\r\n");
}

static void show_predef_menu()
{
    std::printf("\r\n");
    std::printf("A few protocols are not supported for direct decoding by BC7215A chip.\r\n");
    std::printf("When direct sampling fails, try using predefined data to control AC.\r\n");
    std::printf("Please select:\r\n");

    for (uint8_t i = 0; i < ac.predefined_count(); ++i)
    {
        std::printf("  %u. %s\r\n", static_cast<unsigned>(i), ac.predefined_name(i));
    }

    std::printf("\r\n");
}

// -----------------------------
// State jobs
// -----------------------------
// Temperature-unit selection. This must be done before pairing because the pairing
// prompt asks the user to set the remote to a specific reference temperature.
static void choose_unit_job()
{
    char ch = 0;

    switch (l2_state)
    {
    case L2State::Step1:
        show_unit_menu();
        clear_console_input();        // Ignore keys left from the previous screen.
        l2_state = L2State::Step2;
        break;

    case L2State::Step2:
        if (read_menu_char(ch))
        {
            switch (ch)
            {
            case '1':
                ac.set_celsius();
                std::printf("AC has been set to Celsius.\r\n");
                l2_state = L2State::Step3;
                break;

            case '2':
                ac.set_fahrenheit();
                std::printf("AC has been set to Fahrenheit.\r\n");
                l2_state = L2State::Step3;
                break;

            default:
                l2_state = L2State::Step1;
                break;
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

// Main menu dispatcher. It validates whether the selected feature is currently
// available and then switches main_state to the matching job.
static void main_menu_job()
{
    char ch = 0;

    switch (l2_state)
    {
    case L2State::Step1:
        show_main_menu();
        clear_console_input();        // Wait for a fresh selection after the menu is printed.
        l2_state = L2State::Step2;
        break;

    case L2State::Step2:
        if (read_menu_char(ch))
        {
            switch (ch)
            {
            case '1':
                main_state = L1State::Capture;
                l2_state = L2State::Step1;
                break;

            case '2':
                if (ac.init_ok)
                {
                    main_state = L1State::AcControl;
                    l2_state = L2State::Step1;
                }
                else
                {
                    std::printf("\r\nAC control library not initialized yet, please pair first\r\n");
                }
                break;

            case '3':
                if (ac.init_ok)
                {
                    main_state = L1State::Backup;
                    l2_state = L2State::Step1;
                }
                else
                {
                    std::printf("\r\nAC control library not initialized yet, data can only be saved after successful "
                                "pairing.\r\n");
                }
                break;

            case '4':
                main_state = L1State::Restore;
                l2_state = L2State::Step1;
                break;

            case '5':
                if (ac.init_ok)
                {
                    main_state = L1State::FindNext;
                    l2_state = L2State::Step1;
                }
                else
                {
                    std::printf("\r\nThis function is only available after pairing\r\n");
                }
                break;

            case '6':
                main_state = L1State::LoadPredef;
                l2_state = L2State::Step1;
                break;

            case '7':
                main_state = L1State::IrParsing;
                l2_state = L2State::Step1;
                break;

            case '8':
                main_state = L1State::ChooseUnit;
                l2_state = L2State::Step1;
                break;

            default:
                main_state = L1State::MainMenu;
                l2_state = L2State::Step1;
                break;
            }

        }
        break;

    default:
        break;
    }
}

// Pairing workflow:
//   1. Ask the user to set the AC remote to a known reference state.
//   2. Capture the Fan-Control IR frame.
//   3. Initialize the AC library from the decoded data.
static void capture_job()
{
    char ch = 0;

    switch (l2_state)
    {
    case L2State::Step1:
        if (ac.is_celsius())
        {
            std::printf("\r\nNow performing IR AC pairing.\r\n");
            std::printf("Please set AC remote to <Cooling mode, 25C>, then press any key to continue...\r\n");
        }
        else
        {
            std::printf("\r\nNow performing IR AC pairing.\r\n");
            std::printf("Please set AC remote to <Cooling mode, 78F>, then press any key to continue...\r\n");
        }
        clear_console_input();
        l2_state = L2State::Step2;
        break;

    case L2State::Step2:
        if (read_menu_char(ch))
        {
            std::printf("Now please aim at IR receiver and press <Fan Control> button on remote.\r\n");
            std::printf("Will automatically proceed to next step after receiving signal...\r\n");
            // Parameter 1 means capture the signal used for AC pairing.
            ac.start_capture(1);
            l2_state = L2State::Step3;
        }
        break;

    case L2State::Step3:
        if (ac.signal_captured())
        {
            ac.stop_capture();

            // Initialize the AC protocol matcher using the latest captured IR data.
            if (ac.init())
            {
                std::printf("Received data: ");
                data_pkt = ac.data_packet();
                print_packet_payload(data_pkt);
                std::printf("AC control library initialization using received data  **SUCCESS** !!!\r\n");
            }
            else
            {
                std::printf("AC control library initialization using received data **FAILED**,\r\n");
                std::printf("possibly due to incorrect remote state settings or receiving decode errors.\r\n");
            }

            l2_state = L2State::Step4;
        }
        break;

    case L2State::Step4:
        std::printf("Now press any key to return to the main menu and AC control can begin...\r\n");
        clear_console_input();
        l2_state = L2State::Step5;
        break;

    case L2State::Step5:
        if (read_menu_char(ch))
        {
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;

    default:
        break;
    }
}

// AC control workflow. Power commands and parameter commands both end in Step4,
// where the code waits until BC7215A is no longer busy or a timeout occurs.
static void ac_control_job()
{
    char ch = 0;
    char input[160] = {};

    switch (l2_state)
    {
    case L2State::Step1:
        show_ctrl_menu();
        clear_console_input();
        l2_state = L2State::Step2;
        break;

    case L2State::Step2:
        if (read_menu_char(ch))
        {
            switch (ch)
            {
            case '1':
                show_param_menu();
                clear_console_input();
                l2_state = L2State::Step3;
                break;

            case '2':
                std::printf("Sending AC power on command\r\n");
                start_time_ms = now_ms();
                data_pkt = ac.on();
                std::printf("Sending data: ");
                print_packet_payload(data_pkt);
                go_back_state = L2State::Step2;
                l2_state = L2State::Step4;
                break;

            case '3':
                std::printf("Sending AC power off command\r\n");
                start_time_ms = now_ms();
                data_pkt = ac.off();
                std::printf("Sending data: ");
                print_packet_payload(data_pkt);
                go_back_state = L2State::Step2;
                l2_state = L2State::Step4;
                break;

            case '4':
                main_state = L1State::MainMenu;
                l2_state = L2State::Step1;
                break;

            default:
                break;
            }

        }
        break;

    case L2State::Step3:
        if (read_console_line(input, sizeof(input)))
        {
            if (input[0] == '\0')
            {
                break;
            }

            if (strcasecmp(input, "exit") == 0)
            {
                std::printf("Exit\r\n");
                l2_state = L2State::Step1;
                break;
            }

            int t = -1;
            int m = -1;
            int f = -1;
            int k = -1;

            // Missing fields keep their default -1 value and will be normalized to "Keep" below.
            std::sscanf(input, "%d,%d,%d,%d", &t, &m, &f, &k);

            if (m < 0 || m > 4)
            {
                m = 5;        // Keep
            }
            if (f < 0 || f > 3)
            {
                f = 4;        // Keep
            }
            if (k < 0 || k > 3)
            {
                k = 4;        // Keep
            }

            std::printf("Sending command to set AC to: ");
            if (ac.is_celsius() && t >= 16 && t <= 30)
            {
                std::printf("%dC, Mode: ", t);
            }
            else if (!ac.is_celsius() && t >= 60 && t <= 88)
            {
                std::printf("%dF, Mode: ", t);
            }
            else
            {
                std::printf("(keep Temp), Mode: ");
            }

            std::printf("%s, Fan Speed: %s, Key Press: %s\r\n", MODES[m], FANSPEED[f], PRESSED_KEY[k]);

            start_time_ms = now_ms();
            // ac.set_to() both updates the desired AC state and sends the corresponding IR command.
            data_pkt = ac.set_to(t, m, f, k);

            std::printf("Sending data: ");
            print_packet_payload(data_pkt);

            go_back_state = L2State::Step3;
            l2_state = L2State::Step4;
        }
        break;

    case L2State::Step4:
        // BUSY/CTS indicates that BC7215A is still transmitting. Timeout prevents
        // the UI from getting stuck if the hardware line or module state is abnormal.
        if (!ac.is_busy() || (now_ms() - start_time_ms > kTxTimeoutMs))
        {
            if (now_ms() - start_time_ms > kTxTimeoutMs)
            {
                std::printf("Transmission timeout\r\n");
            }

            std::printf("Transmission complete!\r\n\r\n");
            std::printf("Please continue input\r\n");
            clear_console_input();
            l2_state = go_back_state;
        }
        break;

    default:
        break;
    }
}

// Save the current pairing result and current temperature unit to NVS.
static void backup_job()
{
    char ch = 0;

    switch (l2_state)
    {
    case L2State::Step1:
        if (ac.init_ok)
        {
            if (save_ac_config())
            {
                format_pkt = ac.format_packet();
                data_pkt = ac.data_packet();

                std::printf("\r\nFormat info: ");
                print_data(format_pkt, sizeof(bc7215FormatPkt_t));
                std::printf("Data: ");
                print_packet_payload(data_pkt);
                std::printf("Information saved to NVS flash\r\n");
            }
            else
            {
                std::printf("\r\nSaving to NVS failed\r\n");
            }
        }
        else
        {
            std::printf("\r\nThis function is only available after pairing\r\n");
        }

        std::printf("Press any key to continue\r\n");
        clear_console_input();
        l2_state = L2State::Step2;
        break;

    case L2State::Step2:
        if (read_menu_char(ch))
        {
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;

    default:
        break;
    }
}

// Restore pairing data from NVS and use it to initialize the AC control library.
static void restore_job()
{
    char ch = 0;
    bool is_celsius = true;

    switch (l2_state)
    {
    case L2State::Step1:
        if (!load_ac_config(ir_format, ir_data, is_celsius))
        {
            std::printf("Restore failed. Press any key to continue\r\n");
            clear_console_input();
            l2_state = L2State::Step2;
            break;
        }

        std::printf("\r\nUsing saved configuration from NVS flash\r\n");
        std::printf("Format info: ");
        print_data(&ir_format, sizeof(bc7215FormatPkt_t));
        std::printf("Data: ");
        print_data(ir_data.data, (ir_data.bitLen + 7) / 8);

        if (is_celsius)
        {
            ac.set_celsius();
        }
        else
        {
            ac.set_fahrenheit();
        }

        if (ac.init(ir_data, ir_format))
        {
            std::printf("AC control library initialization  ***SUCCESS*** !\r\n");
        }
        else
        {
            std::printf("AC control library initialization failed...\r\n");
        }

        std::printf("Press any key to continue\r\n");
        clear_console_input();
        l2_state = L2State::Step2;
        break;

    case L2State::Step2:
        if (read_menu_char(ch))
        {
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;

    default:
        break;
    }
}

// Try the next protocol candidate after a successful pairing. This is useful when
// BC7215A decoded the signal but the first matched protocol does not control the AC correctly.
static void find_next_job()
{
    char ch = 0;

    switch (l2_state)
    {
    case L2State::Step1:
        if (ac.match_next())
        {
            std::printf("Next protocol match successful!\r\n");
        }
        else
        {
            std::printf("No other matching protocols, AC control library needs re-initialization\r\n");
        }

        std::printf("Press any key to continue...\r\n");
        clear_console_input();
        l2_state = L2State::Step2;
        break;

    case L2State::Step2:
        if (read_menu_char(ch))
        {
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;

    default:
        break;
    }
}

// Load a protocol from the library's predefined protocol table. This is a fallback
// for protocols that cannot be decoded directly from the captured signal.
static void load_predef_job()
{
    char ch = 0;

    switch (l2_state)
    {
    case L2State::Step1:
        show_predef_menu();
        clear_console_input();
        l2_state = L2State::Step2;
        break;

    case L2State::Step2:
        if (read_menu_char(ch))
        {
            int choice = -1;

            if (ch >= '0' && ch <= '9')
            {
                choice = ch - '0';
            }

            std::printf("Selected option %d\r\n", choice);

            if (choice >= 0 && choice <= 255 && ac.init_predefined(static_cast<uint8_t>(choice)))
            {
                std::printf("Format: ");
                print_data(ac.format_packet(), sizeof(bc7215FormatPkt_t));

                std::printf("Data: ");
                const bc7215DataVarPkt_t* pkt = ac.data_packet();
                print_data(pkt, data_packet_storage_size(pkt));

                std::printf("Initialization successful !!! Press any key to continue\r\n");
            }
            else
            {
                std::printf("Initialization failed.... Press any key to continue\r\n");
            }

            l2_state = L2State::Step3;
        }
        break;

    case L2State::Step3:
        if (read_menu_char(ch))
        {
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;

    default:
        break;
    }
}

// Continuously capture IR signals and parse them into human-readable AC parameters.
// Any console input exits this mode and returns to the main menu.
static void ir_parsing_job()
{
    char ch = 0;
    int  t = -1;
    int  m = -1;
    int  f = -1;
    int  p = -1;

    switch (l2_state)
    {
    case L2State::Step1:
        if (ac.init_ok)
        {
            std::printf("\r\n\r\nReceiving IR signal and parsing Temperature, Mode, Fan Speed, and Power status.\r\n");
            std::printf("BC7215A is now in RX mode, ready to decode. Enter anything on keyboard to exit\r\n");
            // Parameter 0 means normal receive/decode mode rather than pairing capture.
            ac.start_capture(0);
            clear_console_input();
            l2_state = L2State::Step2;
        }
        else
        {
            std::printf("\r\n\r\nThis function is only available when paired with AC.\r\n");
            std::printf("Please pair first.\r\n");
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;

    case L2State::Step2:
        if (ac.signal_captured())
        {
            ac.stop_capture();

            t = -1;
            m = -1;
            f = -1;
            p = -1;

            std::printf("Parsing Result: Temp: ");
            if (ac.parse(t, m, f, p))
            {
                if (ac.is_celsius())
                {
                    if (t < 16 || t > 30)
                    {
                        std::printf("n/a, Mode: ");
                    }
                    else
                    {
                        std::printf("%dC, Mode: ", t);
                    }
                }
                else
                {
                    if (t < 60 || t > 88)
                    {
                        std::printf("n/a, Mode: ");
                    }
                    else
                    {
                        std::printf("%dF, Mode: ", t);
                    }
                }

                if (m < 0 || m > 4)
                {
                    m = 6;
                }
                if (f < 0 || f > 3)
                {
                    f = 5;
                }
                if (p < 0 || p > 2)
                {
                    p = 3;
                }

                std::printf("%s, Fan Speed: %s, Power: %s\r\n", MODES[m], FANSPEED[f], PWR_STATUS[p]);
            }
            else
            {
                std::printf("Parsing failed\r\n");
            }

            // Restart RX mode so the next remote key press can be decoded.
            ac.start_capture(0);
        }

        if (read_menu_char(ch))
        {
            ac.stop_capture();
            main_state = L1State::MainMenu;
            l2_state = L2State::Step1;
        }
        break;

    default:
        break;
    }
}

// -----------------------------
// app_main
// ----------------------------

extern "C" void app_main(void)
{
    ESP_ERROR_CHECK(setup_stdio_console());

    ESP_ERROR_CHECK(init_nvs());

    // Initialize UART, GPIOs, and BC7215A driver resources.
    ESP_ERROR_CHECK(ac.begin());

    // Wake BC7215A first if it was in shutdown mode, then return to idle/TX-ready state.
    ac.start_capture(0);
    ac.stop_capture();

    std::printf("\r\nSystem initialized.\r\n");
    std::printf("UART=%d TX=%d RX=%d CTS/BUSY=%d MOD=%d\r\n", static_cast<int>(BC_UART), static_cast<int>(BC_TX_PIN),
        static_cast<int>(BC_RX_PIN), static_cast<int>(BC_BUSY_PIN), static_cast<int>(BC_MOD_PIN));

    // Main cooperative loop. Each job performs one small piece of work and returns;
    // delay_ms() yields to FreeRTOS before polling again.
    while (true)
    {
        switch (main_state)
        {
        case L1State::ChooseUnit:
            choose_unit_job();
            break;

        case L1State::MainMenu:
            main_menu_job();
            break;

        case L1State::Capture:
            capture_job();
            break;

        case L1State::AcControl:
            ac_control_job();
            break;

        case L1State::Backup:
            backup_job();
            break;

        case L1State::Restore:
            restore_job();
            break;

        case L1State::FindNext:
            find_next_job();
            break;

        case L1State::LoadPredef:
            load_predef_job();
            break;

        case L1State::IrParsing:
            ir_parsing_job();
            break;

        default:
            break;
        }

        delay_ms(kLoopIntervalMs);
    }
}
