#include "KlipperCLI.h"
#include "Hardware.h"
#include "ControlLogic.h"
#include "UnitState.h"
#include <ArduinoJson.h>
#include <string.h>

// Define correct serial port if not defined
#ifndef SERIAL_PORT
#define SERIAL_PORT Serial1
#endif

namespace KlipperCLI {

    static char rx_buffer[1024]; // Large buffer for JSON
    static int rx_idx = 0;
    
    // Shared Document buffer to avoid stack overflow/heap fragmentation
    // 20KB SRAM is tight. 2KB for JSON is reasonable (rx is 1KB).
    static JsonDocument doc; // Elastic, but we clear it. ArduinoJson 7 manages its own heap.

    // Response Helper
    void SendResponse(JsonDocument& d) {
        // Serialize to buffer
        static char output[1200]; // Increased buffer slightly
        size_t len = serializeJson(d, output, sizeof(output));
        
        // Send via Hardware UART
        Hardware::UART_Send((const uint8_t*)output, len);
        Hardware::UART_Send((const uint8_t*)"\r\n", 2);
    }
    
    void SendError(int id, const char* code, const char* msg) {
        doc.clear();
        doc["id"] = id;
        doc["ok"] = false;
        doc["code"] = code;
        doc["msg"] = msg;
        SendResponse(doc);
    }
    
    void SendOk(int id, const char* code = nullptr, const char* msg = nullptr) {
        doc.clear();
        doc["id"] = id;
        doc["ok"] = true;
        if(code) doc["code"] = code;
        if(msg) doc["msg"] = msg;
        SendResponse(doc);
    }

    // --- Command Handlers ---
    
    void HandlePing(int id, JsonObject args) {
        doc.clear();
        doc["id"] = id;
        doc["cmd"] = "PING";
        doc["result"] = "ok";
        
        JsonObject t = doc["telemetry"].to<JsonObject>();
        t["version"] = UnitState::GetVersion();
        t["uptime"] = Hardware::GetTime();
        
        SendResponse(doc);
    }

    void HandleStatus(int id, JsonObject args) {
         // Manual JSON construction to avoid Heap Fragmentation/Overflow
         // 2048 bytes static buffer for full status
         static char output[2048]; 
         int offset = 0;
         
         // Header
         offset += snprintf(output + offset, sizeof(output) - offset, 
             "{\"id\":%d,\"cmd\":\"STATUS\",\"ok\":true,\"lanes\":[", id);
             
         uint16_t sensors = ControlLogic::GetSensorState();
         
         for(int i=0; i<4; i++) {
            // Separator
            if(i > 0) offset += snprintf(output + offset, sizeof(output) - offset, ",");
            
            int m_val = ControlLogic::GetLaneMotion(i);
            const char* m_str = "Idle";
            switch(m_val) {
                case 0: m_str = "Idle"; break;
                case 1: m_str = "Feed"; break;
                case 2: m_str = "Retract"; break;
                case 3: m_str = "SlowFeed"; break;
                case 5: m_str = "AutoFeed"; break; 
                case 7: m_str = "VelCtrl"; break;
                default: m_str = "Idle"; break;
            }
            
            
            FilamentState &f = UnitState::GetFilament(i);
            
            // Safe Strings (Null-terminated)
            char safe_id[9]; memcpy(safe_id, f.ID, 8); safe_id[8]=0;
            char safe_name[21]; memcpy(safe_name, f.name, 20); safe_name[20]=0;
            
            // Manual Float formatting (printf %f not supported)
            int m_int = (int)f.meters;
            int m_dec = (int)((f.meters - m_int) * 100); 
            if(m_dec < 0) m_dec = -m_dec; // Handle negative decimal part if any
            
            int p_int = f.pressure / 1000;
            int p_dec = f.pressure % 1000;
            
            offset += snprintf(output + offset, sizeof(output) - offset, 
                "{\"id\":%d,\"present\":%s,\"motion\":\"%s\",\"meters\":%d.%02d,\"pressure\":%d.%03d,\"rfid\":\"%s\",\"name\":\"%s\",\"temp_min\":%d,\"temp_max\":%d,\"color\":[%d,%d,%d,%d]}",
                i, 
                (sensors & (1<<i)) ? "true" : "false",
                m_str,
                m_int, m_dec,
                p_int, p_dec,
                safe_id,
                safe_name,
                f.temperature_min,
                f.temperature_max,
                f.color_R, f.color_G, f.color_B, f.color_A
            );
            
            // Safety break
            if (offset >= (int)sizeof(output) - 100) break;
         }
         
         // Footer
         offset += snprintf(output + offset, sizeof(output) - offset, "]}");
         
         // Send
         Hardware::UART_Send((const uint8_t*)output, offset);
         Hardware::UART_Send((const uint8_t*)"\r\n", 2);
    }
    
    void HandleGetSensors(int id, JsonObject args) {
         doc.clear();
         doc["id"] = id;
         doc["cmd"] = "GET_SENSORS";
         
         uint16_t state = ControlLogic::GetSensorState();
         JsonArray lanes = doc["lane"].to<JsonArray>();
         for(int i=0; i<4; i++) {
             lanes.add((state & (1<<i)) ? 1 : 0);
         }
         SendResponse(doc);
    }
    
    void HandleMove(int id, JsonObject args) {
        // Args extracted before clearing doc
        // BUT wait, 'args' is a reference to 'doc' internals (if passed from ProcessPacket).
        // If we clear 'doc' inside HandleMove, 'args' becomes invalid!
        // CRITICAL BUG: accessing 'args' after doc.clear() would crash.
        // We MUST extract args BEFORE clearing doc.
        
        // However, we need to send OK/Error.
        // Option 1: Copy args.
        // Option 2: Use a SECOND static doc for Response?
        // Or Option 3: Use 'SendOk/SendError' which clears doc, AFTER using args.
        
        if(!args["axis"].is<const char*>() || !args["dist_mm"].is<float>() || !args["speed"].is<float>()) {
             SendError(id, "BAD_ARGS", "Missing axis, dist, or speed"); // Clears doc
             return;
        }
        
        // Copy values to stack variables
        char axis[16]; strncpy(axis, args["axis"], 15); axis[15]=0;
        float dist = args["dist_mm"];
        float speed = args["speed"];
        
        // Now safe to use logic that might call SendOk/SendError (clearing doc)
        
        int motor_idx = -1;
        if(strcmp(axis, "FEED") == 0) {
            motor_idx = UnitState::GetCurrentFilamentIndex();
        } else if(strcmp(axis, "SELECTOR") == 0) {
        } else if (strncmp(axis, "SPOOL", 5) == 0) {
        } else if (isdigit(axis[0])) {
             motor_idx = atoi(axis);
        }
        
        if (motor_idx >= 0 && motor_idx < 4) {
             ControlLogic::MoveAxis(motor_idx, dist, speed);
             SendOk(id, "MOVING", "Motion started");
        } else {
             SendError(id, "BAD_AXIS", "Invalid or unknown axis");
        }
    }

    void HandleStop(int id, JsonObject args) {
        ControlLogic::StopAll();
        SendOk(id, "STOPPED", "All motion stopped");
    }

    void HandleSelectLane(int id, JsonObject args) {
        if(!args["lane"].is<int>()) {
            SendError(id, "BAD_ARGS", "Missing lane");
            return;
        }
        int lane = args["lane"];
        if(lane < 0 || lane >= 4) {
            SendError(id, "BAD_LANE", "Lane must be 0-3");
            return;
        }
        UnitState::SetCurrentFilamentIndex(lane);
        SendOk(id);
    }

    void HandleSetAutoFeed(int id, JsonObject args) {
        if(!args["lane"].is<int>() || !args["enable"].is<bool>()) {
             SendError(id, "BAD_ARGS", "Missing lane or enable");
             return;
        }
        int lane = args["lane"];
        bool enable = args["enable"];
        ControlLogic::SetAutoFeed(lane, enable);
        SendOk(id);
    }

    void HandleGetFilamentInfo(int id, JsonObject args) {
         if(!args["lane"].is<int>()) { SendError(id, "BAD_ARGS", "Missing lane"); return; }
         int lane = args["lane"];
         if(lane < 0 || lane >= 4) { SendError(id, "BAD_ARGS", "Invalid lane"); return; }
         
         FilamentState &f = UnitState::GetFilament(lane);
         
         static char output[1024];
         int offset = 0;
         
         // Safe Strings
         char safe_id[9]; memcpy(safe_id, f.ID, 8); safe_id[8]=0;
         char safe_name[21]; memcpy(safe_name, f.name, 20); safe_name[20]=0;
         
         // Color
         int r = f.color_R;
         int g = f.color_G;
         int b = f.color_B;
         int a = f.color_A;
         
         // Manual Float
         int m_int = (int)f.meters;
         int m_dec = (int)((f.meters - m_int) * 100);
         if(m_dec<0) m_dec = -m_dec;
         
         int p_int = f.pressure / 1000;
         int p_dec = f.pressure % 1000;
         
         offset += snprintf(output + offset, sizeof(output) - offset,
             "{\"id\":%d,\"cmd\":\"GET_FILAMENT_INFO\",\"ok\":true,\"info\":{\"lane\":%d,\"id_str\":\"%s\",\"name\":\"%s\",\"temp_min\":%d,\"temp_max\":%d,\"meters\":%d.%02d,\"color\":[%d,%d,%d,%d]}}",
             id, lane, safe_id, safe_name, f.temperature_min, f.temperature_max, m_int, m_dec, r,g,b,a 
         );
         
         Hardware::UART_Send((const uint8_t*)output, offset);
         Hardware::UART_Send((const uint8_t*)"\r\n", 2);
    }

    void HandleSetFilamentInfo(int id, JsonObject args) {
         // Need to extract ALL args to stack/struct before clearing doc
         // This is tricky for complex objects like Color array.
         // Actually, do we assume ProcessPacket uses the SAME 'doc'?
         // YES. ProcessPacket uses 'doc'. 'args' points to inside 'doc'.
         // So we MUST NOT clear 'doc' until we are done reading 'args'.
         
         // Solution: Create a temporary FilamentInfo struct on stack
         
         if(!args["lane"].is<int>()) { SendError(id, "BAD_ARGS", "Missing lane"); return; }
         int lane = args["lane"];
         
         // ... (Logic to read args into stack vars)
         FilamentInfo info;
         FilamentState &current = UnitState::GetFilament(lane);
         memcpy(info.ID, current.ID, sizeof(info.ID));
         memcpy(info.name, current.name, sizeof(info.name));
         info.color_R = current.color_R;
         info.color_G = current.color_G;
         info.color_B = current.color_B;
         info.color_A = current.color_A;
         info.temperature_min = current.temperature_min;
         info.temperature_max = current.temperature_max;

         if(args["id_str"].is<const char*>()) info.SetID(args["id_str"]);
         if(args["name"].is<const char*>()) info.SetName(args["name"]);
         if(args["temp_min"].is<int>()) info.temperature_min = args["temp_min"];
         if(args["temp_max"].is<int>()) info.temperature_max = args["temp_max"];
         
         if(args["color"].is<JsonArray>()) {
             JsonArray c = args["color"];
             if(c.size() >= 3) {
                 info.color_R = c[0]; info.color_G = c[1]; info.color_B = c[2];
                 if(c.size() > 3) info.color_A = c[3]; else info.color_A = 255;
             }
         }
         
         float meters = -1.0f;
         if(args["meters"].is<float>()) meters = args["meters"];
         
         // Now we are done with args.
         ControlLogic::SetFilamentInfoAction(lane, info, meters);
         SendOk(id);
    }



    void ProcessPacket(char* json_str) {
        doc.clear();
        DeserializationError error = deserializeJson(doc, json_str);

        if (error) {
            // Malformed JSON, maybe echo back error if possible, or ignore
            const char* err_msg = "{\"ok\":false,\"msg\":\"JSON Parse Error\"}\n";
            Hardware::UART_Send((const uint8_t*)err_msg, strlen(err_msg));
            return;
        }

        int id = doc["id"] | 0;
        const char* cmd = doc["cmd"];
        JsonObject args = doc["args"];

        if (!cmd) return;

        if (strcmp(cmd, "PING") == 0) HandlePing(id, args);
        else if (strcmp(cmd, "STATUS") == 0) HandleStatus(id, args);
        else if (strcmp(cmd, "GET_SENSORS") == 0) HandleGetSensors(id, args);
        else if (strcmp(cmd, "MOVE") == 0) HandleMove(id, args);
        else if (strcmp(cmd, "STOP") == 0) HandleStop(id, args);
        else if (strcmp(cmd, "SELECT_LANE") == 0) HandleSelectLane(id, args);
        else if (strcmp(cmd, "SET_AUTO_FEED") == 0) HandleSetAutoFeed(id, args);
        else if (strcmp(cmd, "GET_FILAMENT_INFO") == 0) HandleGetFilamentInfo(id, args);
        else if (strcmp(cmd, "SET_FILAMENT_INFO") == 0) HandleSetFilamentInfo(id, args);
        else {
            SendError(id, "UNKNOWN_CMD", cmd);
        }
    }

    static volatile bool packet_ready = false;

    void Init() {
        Hardware::UART_Send((const uint8_t*)"{\"event\":\"STARTUP\",\"msg\":\"KlipperCLI Ready\"}\r\n", 52);
    }

    void Run() {
        if (packet_ready) {
             ProcessPacket(rx_buffer);
             // Reset buffer
             rx_idx = 0;
             packet_ready = false; 
        }
    }
    
    // Helper to feed data (called from ISR)
    void FeedByte(uint8_t b) {
        // Prevent buffer overrun
        if (packet_ready) return; 

        if (b == '\n' || b == '\r') {
             if (rx_idx > 0) {
                 rx_buffer[rx_idx] = '\0';
                 packet_ready = true;
             }
             // Ignore empty lines (e.g. \r then \n)
        } else {
            if (rx_idx < 1023) {
                rx_buffer[rx_idx++] = (char)b;
            } else {
                rx_idx = 0; // Overflow reset
            }
        }
    }
}
