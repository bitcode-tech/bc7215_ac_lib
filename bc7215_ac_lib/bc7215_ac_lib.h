/**
 * @file bc7215_ac_lib.h
 * @brief BC7215 Air Conditioner Control Library Header File
 * @details This library provides functions to control air conditioners using the BC7215 chip.
 *          It supports various AC modes, fan speeds, and temperature control operations.
 * @version See bc7215_ac_get_ver() for current version
 * @date Created: 2025-05-20
 * @author Bitcode
 */

#ifndef BC7215_AC_LIB_H
#define BC7215_AC_LIB_H

#include <stdbool.h>
#include <stddef.h>
#include "bc7215_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ================================================================================================
 * AIR CONDITIONER MODE DEFINITIONS
 * ================================================================================================ */

/** @defgroup AC_Modes Air Conditioner Operating Modes
 * @brief Defines the available operating modes for the air conditioner
 * @{
 */
#define MODE_AUTO   0  /**< Automatic mode - AC adjusts cooling/heating automatically */
#define MODE_COOL   1  /**< Cooling mode - AC operates in cooling mode */
#define MODE_HOT    2  /**< Heating mode - AC operates in heating mode */
#define MODE_DRY    3  /**< Dry mode - AC removes humidity without significant temperature change */
#define MODE_FAN    4  /**< Fan only mode - AC operates as a fan without cooling/heating */
/** @} */

/* ================================================================================================
 * FAN SPEED DEFINITIONS
 * ================================================================================================ */

/** @defgroup Fan_Speeds Fan Speed Settings
 * @brief Defines the available fan speed settings
 * @{
 */
#define FAN_AUTO    0  /**< Automatic fan speed - AC adjusts fan speed automatically */
#define FAN_LOW     1  /**< Low fan speed */
#define FAN_MED     2  /**< Medium fan speed */
#define FAN_HIGH    3  /**< High fan speed */
/** @} */

/* ================================================================================================
 * CONTROL KEY DEFINITIONS
 * ================================================================================================ */

/** @defgroup Control_Keys Control Key Identifiers
 * @brief Defines the available control keys for AC operation
 * @{
 */
#define KEY_PLUS    0  /**< Temperature increase key */
#define KEY_MINUS   1  /**< Temperature decrease key */
#define KEY_MODE    2  /**< Mode selection key */
#define KEY_FAN     3  /**< Fan speed selection key */
/** @} */

/* ================================================================================================
 * DATA STRUCTURES
 * ================================================================================================ */

/**
 * @brief Combined message structure for BC7215 AC control
 * @details This structure combines format and data packets for AC control operations
 */
typedef struct {
    uint16_t    bitLen;     /**< Bit length placeholder, always set to 0 */
    union {
        uint8_t data[1];    /**< Raw data array */
        struct {
            const bc7215FormatPkt_t*    fmt;        /**< Pointer to format packet */
            const bc7215DataVarPkt_t*   datPkt;     /**< Pointer to data packet */
        } msg;              /**< Message structure containing format and data packets */
    } body;                 /**< Union containing either raw data or structured message */
} bc7215CombinedMsg_t;

/* ================================================================================================
 * PUBLIC FUNCTION DECLARATIONS
 * ================================================================================================ */

/**
 * @brief Get the version string of the BC7215 AC library
 * @return Pointer to a constant string containing the version information
 * @note The returned string is statically allocated and should not be modified
 */
const char* bc7215_ac_get_ver(void);

/**
 * @brief Initialize the BC7215 AC control library (Celsius)
 * @param status Status byte from the data packet
 * @param dataPktCool25C Reference data packet for cooling at 25°C
 * @return true if initialization successful, false otherwise
 * @note This function must be called before using any other library functions
 * @warning Ensure the dataPktCool25C parameter points to valid data
 */
bool bc7215_ac_init(uint8_t status, const bc7215DataVarPkt_t* dataPktCool25C);

/**
 * @brief Initialize the BC7215 AC control library (Fahrenheit)
 * @param status Status byte from the data packet
 * @param dataPktCool25C Reference data packet for cooling at 78°F
 * @return true if initialization successful, false otherwise
 * @note This function must be called before using any other library functions
 * @warning Ensure the dataPktCool25C parameter points to valid data
 */
bool bc7215_ac_init_f(uint8_t status, const bc7215DataVarPkt_t* dataPktCool78F);

/**
 * @brief Find the next available AC configuration
 * @return true if next configuration found, false if no more configurations available
 * @note This function is used to iterate through available AC configurations
 */
bool bc7215_ac_find_next(void);

/**
 * @brief Set AC parameters and generate corresponding data packet (Celsius)
 * @param temp Target temperature (0 to 14, referring 16°C to 30°C)
 * @param mode Operating mode (see MODE_* definitions)
 * @param fan Fan speed setting (see FAN_* definitions)
 * @param key Control key being pressed (see KEY_* definitions)
 * @return Pointer to generated data packet
 * @note The returned packet contains the IR signal data for the specified settings
 */
const bc7215DataVarPkt_t* bc7215_ac_set(int8_t temp, int8_t mode, int8_t fan, int8_t key);

/**
 * @brief Set AC parameters and generate corresponding data packet (Fahrenheit)
 * @param temp Target Fahrenheit temperature (0 to 28, referring 60°F to 88°F)
 * @param mode Operating mode (see MODE_* definitions)
 * @param fan Fan speed setting (see FAN_* definitions)
 * @param key Control key being pressed (see KEY_* definitions)
 * @return Pointer to generated data packet
 * @note The returned packet contains the IR signal data for the specified settings
 */
const bc7215DataVarPkt_t* bc7215_ac_set_f(int8_t ftemp, int8_t mode, int8_t fan, int8_t key);

/**
 * @brief Generate data packet to turn AC on
 * @return Pointer to data packet for turning AC on, NULL if no dedicated ON packet is needed
 * @note This function generates the appropriate IR signal to power on the AC
 */
const bc7215DataVarPkt_t* bc7215_ac_on(void);

/**
 * @brief Generate data packet to turn AC off
 * @return Pointer to data packet for turning AC off, NULL if operation failed
 * @note This function generates the appropriate IR signal to power off the AC
 */
const bc7215DataVarPkt_t* bc7215_ac_off(void);

/**
 * @brief Get the number of predefined AC configurations
 * @return Number of available predefined configurations
 * @note Use this to determine the valid range for bc7215_ac_predefined_* functions
 */
uint8_t bc7215_ac_predefined_cnt(void);

/**
 * @brief Get predefined AC configuration data by index (Celsius)
 * @param index Index of the predefined configuration (0 to bc7215_ac_predefined_cnt()-1)
 * @return Pointer to data packet for the specified configuration, NULL if index invalid
 * @warning Ensure index is within valid range to avoid undefined behavior
 */
const bc7215DataVarPkt_t* bc7215_ac_predefined_data(uint8_t index);

/**
 * @brief Get predefined AC configuration data by index (Fahrenheit)
 * @param index Index of the predefined configuration (0 to bc7215_ac_predefined_cnt()-1)
 * @return Pointer to data packet for the specified configuration, NULL if index invalid
 * @warning Ensure index is within valid range to avoid undefined behavior
 */
const bc7215DataVarPkt_t* bc7215_ac_predefined_data_f(uint8_t index);

/**
 * @brief Get predefined AC configuration format packet by index
 * @param index Index of the predefined configuration (0 to bc7215_ac_predefined_cnt()-1)
 * @return Pointer to format packet for the specified configuration, NULL if index invalid
 * @warning Ensure index is within valid range to avoid undefined behavior
 */
const bc7215FormatPkt_t* bc7215_ac_predefined_fmt(uint8_t index);

/**
 * @brief Get predefined AC configuration name by index
 * @param index Index of the predefined configuration (0 to bc7215_ac_predefined_cnt()-1)
 * @return Pointer to string containing the configuration name, NULL if index invalid
 * @note The returned string is statically allocated and should not be modified
 * @warning Ensure index is within valid range to avoid undefined behavior
 */
const char* bc7215_ac_predefined_name(uint8_t index);

/**
 * @brief Check if this protocol needs extra data sample
 * @return An 8-bit number indicating which parameter needs extra sampling
 * @retval 0 No extra sampling is needed
 * @retval 1 Temperature setting command needs extra sampling
 * @retval 2 Mode setting command needs extra sampling  
 * @retval 3 Fan setting command needs extra sampling
 * @note This function should be called after a successful init only
 */
uint8_t bc7215_ac_need_extra_sample(void);

/**
 * @brief Store extra sample data in the library
 * @param status Status byte of the extra sampling output data packet
 * @param message Pointer to combined message containing format and data packets of the extra sampling result
 * @return true if successful, false otherwise
 * @note When extra sample is taken, use this function to store them in library
 * @warning Ensure the message parameter contains valid format and data packet pointers
 */
bool bc7215_ac_save_2nd_base(uint8_t status, const bc7215CombinedMsg_t* message);

/**
 * @brief Read extra sample data in the library
* @return a bc7215CombinedMsg_t variable that contains pointer to Format and Data packets
 * @note Use this function to access second format and data stored in library
 */
bc7215CombinedMsg_t bc7215_ac_get_2nd_base(void);

/**
 * @brief Replace the base IR data packet with alternative data packet
 * @param status Status byte for the alternative data packet
 * @param altDataPkt Pointer to alternative data packet to replace the base packet
 * @return true if successful, false otherwise
 * @note Usually used for controlling other A/C settings
 * @warning Ensure altDataPkt is a valid data packet
 */
bool bc7215_ac_replace_base(uint8_t status, const bc7215DataVarPkt_t* altDataPkt);

/**
 * @brief Get the base format packet used during initialization
 * @return Pointer to the base format packet, NULL if not initialized
 * @note This function returns the format packet that was used during library initialization
 */
const bc7215FormatPkt_t* bc7215_ac_get_base_fmt(void);

/**
 * @brief Get the base data packet used during initialization
 * @return Pointer to the base data packet, NULL if not initialized
 * @note This function returns the data packet that was used during library initialization
 */
const bc7215DataVarPkt_t* bc7215_ac_get_base_data(void);

/**
 * @brief Initialize the BC7215 AC control library with multiple message configurations (Celsius)
 * @param msgCnt Number of combined messages in the msgs array
 * @param msgs Array of combined messages containing format and data pointers
 * @param segGap Gap value between segments in the IR signal timing, 0 for default 60ms
 * @return true if initialization successful, false otherwise
 * @note This is an enhanced version of bc7215_ac_init() that supports multi-segment protocols
 * @warning Ensure the msgs array contains valid combined message pointers
 * @warning Ensure msgCnt accurately reflects the number of elements in msgs array
 */
bool bc7215_ac_init2(uint8_t msgCnt, const bc7215CombinedMsg_t msgs[], uint8_t segGap);

/**
 * @brief Initialize the BC7215 AC control library with multiple message configurations (Fahrenheit)
 * @param msgCnt Number of combined messages in the msgs array
 * @param msgs Array of combined messages containing format and data pointers
 * @param segGap Gap value between segments in the IR signal timing, 0 for default 60ms
 * @return true if initialization successful, false otherwise
 * @note This is an enhanced version of bc7215_ac_init() that supports multi-segment protocols
 * @warning Ensure the msgs array contains valid combined message pointers
 * @warning Ensure msgCnt accurately reflects the number of elements in msgs array
 */
bool bc7215_ac_init2_f(uint8_t msgCnt, const bc7215CombinedMsg_t msgs[], uint8_t segGap);

/**
 * @brief Parse IR data packet and extract AC control parameters (Celsius)
 * @details This function analyzes the current data packet and extracts the air conditioner
 *          control parameters including temperature, operating mode, fan speed, and power state.
 *          The parsed values are written to the variables pointed to by the provided parameters.
 * @param temp Pointer to variable where parsed temperature will be stored (0 to 14, referring 16°C to 30°C)
 * @param mode Pointer to variable where parsed operating mode will be stored (see MODE_* definitions)
 * @param fan Pointer to variable where parsed fan speed will be stored (see FAN_* definitions)
 * @param power Pointer to variable where parsed power state will be stored (0 = off, 1 = on, 2 = toggle)
 * @return true if parsing successful and parameters extracted, false if parsing failed
 * @note The library must be initialized.
 * @note The data to be parsed must be loaded via bc7215_ac_replace_base() before this function is called.
 * @note All pointer parameters must point to valid memory locations
 * @warning Ensure all pointer parameters are not NULL to avoid undefined behavior
 * @warning The function may fail if no valid data packet is available for parsing
 */
bool bc7215_ac_parse(int8_t* temp, int8_t* mode, int8_t* fan, int8_t* power);

/**
 * @brief Parse IR data packet and extract AC control parameters (Fahrenheit)
 * @details This function analyzes the current data packet and extracts the air conditioner
 *          control parameters including temperature, operating mode, fan speed, and power state.
 *          The parsed values are written to the variables pointed to by the provided parameters.
 * @param ftemp Pointer to variable where parsed Fahrenheit temperature will be stored (0 to 28, referring 60°F to 88°F)
 * @param mode Pointer to variable where parsed operating mode will be stored (see MODE_* definitions)
 * @param fan Pointer to variable where parsed fan speed will be stored (see FAN_* definitions)
 * @param power Pointer to variable where parsed power state will be stored (0 = off, 1 = on, 2 = toggle)
 * @return true if parsing successful and parameters extracted, false if parsing failed
 * @note The library must be initialized.
 * @note The data to be parsed must be loaded via bc7215_ac_replace_base() before this function is called.
 * @note All pointer parameters must point to valid memory locations
 * @warning Ensure all pointer parameters are not NULL to avoid undefined behavior
 * @warning The function may fail if no valid data packet is available for parsing
 */
bool bc7215_ac_parse_f(int8_t* ftemp, int8_t* mode, int8_t* fan, int8_t* power);


#ifdef __cplusplus
}
#endif

#endif /* BC7215_AC_LIB_H */
