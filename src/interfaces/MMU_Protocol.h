#pragma once
#include <stdint.h>

/**
 * @file MMU_Protocol.h
 * @brief MMU Communication Protocol Definitions
 * 
 * This file defines the structure of requests sent TO the MMU
 * and responses sent FROM the MMU.
 * 
 * Protocol Format: JSON over newline-delimited text
 * Each message is a single JSON object terminated by '\n'
 */

namespace MMU_Protocol {

//=============================================================================
// COMMAND IDENTIFIERS
//=============================================================================

/**
 * Commands sent FROM Host TO MMU
 */
enum class Command : uint8_t {
    // System
    PING        = 0x01,  ///< Heartbeat/connectivity check
    GET_STATUS  = 0x02,  ///< Request full status report
    RESET       = 0x03,  ///< Reset MMU to idle state
    
    // Motion Control
    MOVE        = 0x10,  ///< Move filament by distance/speed
    STOP        = 0x11,  ///< Stop all motion
    LOAD        = 0x12,  ///< Load filament in lane
    UNLOAD      = 0x13,  ///< Unload filament from lane
    
    // Lane Selection
    SELECT_LANE = 0x20,  ///< Select active lane for operations
    
    // Configuration
    SET_FILAMENT_INFO = 0x30,  ///< Set filament metadata
    GET_FILAMENT_INFO = 0x31,  ///< Get filament metadata
    SET_CONFIG        = 0x32,  ///< Set configuration parameter
    GET_CONFIG        = 0x33,  ///< Get configuration parameter
};

/**
 * Response codes sent FROM MMU TO Host
 */
enum class ResponseCode : uint8_t {
    OK          = 0x00,  ///< Command executed successfully
    ERROR       = 0x01,  ///< Command failed
    BUSY        = 0x02,  ///< MMU is busy, try later
    INVALID_CMD = 0x03,  ///< Unknown command
    INVALID_ARG = 0x04,  ///< Invalid argument
};

//=============================================================================
// MOTION STATES (for status reporting)
//=============================================================================

/**
 * Motion state strings for JSON responses
 */
namespace MotionState {
    constexpr const char* IDLE      = "idle";
    constexpr const char* LOADING   = "loading";
    constexpr const char* UNLOADING = "unloading";
    constexpr const char* FEEDING   = "feeding";
    constexpr const char* RETRACTING= "retracting";
    constexpr const char* STOPPED   = "stopped";
}

//=============================================================================
// JSON COMMAND STRINGS
//=============================================================================

/**
 * Command name strings for JSON parsing
 */
namespace CommandStr {
    constexpr const char* PING            = "PING";
    constexpr const char* STATUS          = "STATUS";
    constexpr const char* GET_SENSORS     = "GET_SENSORS";
    constexpr const char* RESET           = "RESET";
    constexpr const char* MOVE            = "MOVE";
    constexpr const char* STOP            = "STOP";
    constexpr const char* LOAD            = "LOAD_FILAMENT";
    constexpr const char* UNLOAD          = "UNLOAD_FILAMENT";
    constexpr const char* SELECT_LANE     = "SELECT_LANE";
    constexpr const char* SET_FILAMENT_INFO = "SET_FILAMENT_INFO";
    constexpr const char* GET_FILAMENT_INFO = "GET_FILAMENT_INFO";
}

//=============================================================================
// LANE STATUS STRUCTURE
//=============================================================================

/**
 * Per-lane status data for STATUS response
 */
struct LaneStatus {
    uint8_t id;           ///< Lane index (0-3)
    bool present;         ///< Filament presence detected
    const char* motion;   ///< Motion state string
    float meters;         ///< Filament length remaining (meters)
    float pressure;       ///< Pressure sensor reading (volts)
    
    // Filament info
    const char* name;     ///< Filament name
    const char* filament_id; ///< Filament ID/RFID
    uint8_t color_r;      ///< Color RED
    uint8_t color_g;      ///< Color GREEN
    uint8_t color_b;      ///< Color BLUE
    uint16_t temp_min;    ///< Min print temperature
    uint16_t temp_max;    ///< Max print temperature
};

//=============================================================================
// PROTOCOL CONSTANTS
//=============================================================================

constexpr uint16_t MAX_MESSAGE_LEN = 512;  ///< Maximum JSON message length
constexpr char MESSAGE_TERMINATOR = '\n';  ///< End of message delimiter
constexpr uint32_t TIMEOUT_MS = 5000;      ///< Command timeout

} // namespace MMU_Protocol
