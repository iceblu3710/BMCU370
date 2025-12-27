/*
* DEVELOPMENT STATE: TESTING
* This file implements the Klipper JSON protocol which is currently in a testing state.
*/
#include "KlipperCLI.h"
#include "I_MMU_Transport.h"
#include "MMU_Logic.h"
#include "UnitState.h"
#include "LiteJSON.h"
#include "Hardware.h"
#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include <limits.h>

namespace KlipperCLI {

    static MMU_Logic* _mmu = nullptr;
    static I_MMU_Transport* _transport = nullptr;
    static char rx_buffer[1024]; // Synced to UART_Transport size
    static int rx_idx = 0;
    static bool last_was_cr = false;
    static JsonDocument doc;
    static char global_json_buf[1024]; // Shared buffer for all responses
    static uint64_t last_activity_time = 0; // Track last serial activity for smart save timing

    // Response Helper
    void WaitTX() {
        if (!_transport) return;
        uint32_t timeout = 200; 
        uint64_t start = millis();
        while (_transport->IsBusy() && (millis() - start < timeout)) {
            Hardware::DelayUS(100); // Reduced from 1ms to improve throughput
        }
    }

    void SendResponse(JsonDocument& d) {
        if (!_transport) return;
        WaitTX();
        size_t len = serializeJson(d, global_json_buf, sizeof(global_json_buf) - 2);
        global_json_buf[len++] = '\r';
        global_json_buf[len++] = '\n';
        _transport->Write((const uint8_t*)global_json_buf, len);
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
        doc["ok"] = true;
        doc["result"] = "ok";
        
        LiteObject& t = doc["telemetry"].makeObject();
        t["version"] = "00.00.05.00"; 
        t["uptime"] = (int)millis();
        
        SendResponse(doc);
    }

    void HandleStatus(int id, JsonObject args) {
         if (!_mmu) return;
         WaitTX();
         int offset = 0;
         offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, 
             "{\"id\":%d,\"cmd\":\"STATUS\",\"ok\":true,\"lanes\":[", id);
             
         uint16_t sensors = _mmu->GetSensorState();
         
         for(int i=0; i<4; i++) {
            if(i > 0) offset += snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, ",");
            
            int m_val = _mmu->GetLaneMotion(i);
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
            
            FilamentState &f = _mmu->GetFilament(i);
            
            // Sanitize strings - replace non-printable chars with null terminator
            char safe_id[9]; 
            for(int j=0; j<8; j++) {
                safe_id[j] = (f.ID[j] >= 32 && f.ID[j] < 127) ? f.ID[j] : '\0';
                if(safe_id[j] == '\0') { safe_id[j] = '\0'; break; } // Stop at first non-printable
            }
            safe_id[8] = '\0';
            
            char safe_name[21];
            for(int j=0; j<20; j++) {
                safe_name[j] = (f.name[j] >= 32 && f.name[j] < 127) ? f.name[j] : '\0';
                if(safe_name[j] == '\0') break;
            }
            safe_name[20] = '\0';
            
             float meters_f = f.meters;
             if (!isfinite(meters_f)) meters_f = 0;
             if (meters_f > 2000000000.0f) meters_f = 2000000000.0f;
             if (meters_f < -2000000000.0f) meters_f = -2000000000.0f;

             int m_int = (int)meters_f;
             int m_dec = (int)((meters_f - m_int) * 100); 
             if(m_dec < 0) m_dec = -m_dec; 
            
             int p_int = f.pressure / 1000;
             int p_dec = f.pressure % 1000;
            
             if (offset >= (int)sizeof(global_json_buf) - 256) {
                 // Danger zone: not enough space for a full lane record
                 break; 
             }
             
             // Precision Fix: Handle negative sign for meters between -1.0 and 0.0
             const char* sign = (meters_f < 0 && m_int == 0) ? "-" : "";

             int n = snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, 
                "{\"id\":%d,\"present\":%s,\"motion\":\"%s\",\"meters\":%s%d.%02d,\"pressure\":%d.%03d,\"rfid\":\"%s\",\"name\":\"%s\",\"temp_min\":%d,\"temp_max\":%d,\"color\":[%d,%d,%d,%d]}",
                i, 
                (sensors & (1<<i)) ? "true" : "false",
                m_str, sign, m_int, m_dec, p_int, p_dec, safe_id, safe_name, f.temperature_min, f.temperature_max, f.color_R, f.color_G, f.color_B, f.color_A);
             
             if (n > 0) {
                 if (offset + n >= (int)sizeof(global_json_buf)) {
                     SendError(id, "BUFFER_OVERFLOW", "Status too large");
                     return;
                 }
                 offset += n;
             }
         }
         
         // Finalize manually built JSON with closing array and object
         // Finalize manually built JSON with closing array and object
         int trailing = snprintf(global_json_buf + offset, sizeof(global_json_buf) - offset, "]}\r\n");
         
         if (trailing < 0 || offset + trailing >= (int)sizeof(global_json_buf)) {
             SendError(id, "BUFFER_OVERFLOW", "Status too large");
             return;
         }
         offset += trailing;

         if (_transport) _transport->Write((const uint8_t*)global_json_buf, offset);
    }
    
    void HandleGetSensors(int id, JsonObject args) {
         if (!_mmu) return;
         doc.clear();
         doc["id"] = id;
         doc["cmd"] = "GET_SENSORS";
         doc["ok"] = true;
         
         uint16_t state = _mmu->GetSensorState();
         LiteArray& lanes = doc["lane"].makeArray();
         for(int i=0; i<4; i++) {
             lanes.add((state & (1<<i)) ? 1 : 0);
         }
         SendResponse(doc);
    }
    
    void HandleMove(int id, JsonObject args) {
        if (!_mmu) return;
        if(!args["axis"].isString() || !args["dist_mm"].isFloat() || !args["speed"].isFloat()) {
             SendError(id, "BAD_ARGS", "Missing axis, dist, or speed"); 
             return;
        }
        
        char axis[16]; strncpy(axis, args["axis"], 15); axis[15]=0;
        float dist = args["dist_mm"];
        float speed = args["speed"];
        
        int motor_idx = -1;
        if(strcmp(axis, "FEED") == 0) {
            motor_idx = _mmu->GetCurrentFilamentIndex();
        } else if(strcmp(axis, "SELECTOR") == 0) {
        } else if (isdigit(axis[0])) {
             motor_idx = atoi(axis);
        }
        
        if (motor_idx >= 0 && motor_idx < 4) {
             _mmu->MoveAxis(motor_idx, dist, speed);
             SendOk(id, "MOVING", "Motion started");
        } else {
             SendError(id, "BAD_AXIS", "Invalid or unknown axis");
        }
    }

    void HandleStop(int id, JsonObject args) {
        if (!_mmu) return;
        _mmu->StopAll();
        SendOk(id, "STOPPED", "All motion stopped");
    }

    void HandleSelectLane(int id, JsonObject args) {
        if (!_mmu) return;
        if(!args["lane"].isInt()) {
            SendError(id, "BAD_ARGS", "Missing lane");
            return;
        }
        int lane = args["lane"];
        if(lane < 0 || lane >= 4) {
            SendError(id, "BAD_LANE", "Lane must be 0-3");
            return;
        }
        // UnitState::SetCurrentFilamentIndex(lane); -> Not available on MMU_Logic public interface?
        // Logic check: "data_save.BambuBus_now_filament_num = index;" logic is internal?
        // MMU_Logic has `GetCurrentFilamentIndex`. Does it have Set?
        // I missed adding `SetCurrentFilamentIndex` to MMU_Logic public interface!
        // But `SelectLane` implies just selecting it for context?
        // BambuBus protocol used "process..." to set it.
        // I should ADD `SelectLane(int)` to MMU_Logic or just handle it.
        // Actually, `StartLoadFilament` sets it.
        // Does Klipper utilize "SelectLane" without moving?
        // If so, I need to add `SetActiveLane(int)` to MMU_Logic.
        // I will implement `_mmu->data_save.BambuBus_now_filament_num = lane` if I could access it.
        // But it's private.
        // I will add a method in MMU_Logic.h/cpp later? Or assume SelectLane is not critical for now?
        // It is used for "FEED" axis target.
        // I will omit it for now or implement `SetAutoFeed` which sets it?
        // Actually `HandleSelectLane` sets "BambuBus_now_filament_num".
        // I'll skip it and reply OK, but warn myself.
        // Wait, Klipper `SELECT_LANE` is likely important.
        // I should add `SetCurrentFilamentIndex` to `MMU_Logic`.
        // I'll add it to `MMU_Logic` via `replace_file_content` after this.
        
        _mmu->SetCurrentFilamentIndex(lane);
        SendOk(id);
    }

    void HandleSetAutoFeed(int id, JsonObject args) {
        if (!_mmu) return;
        if(!args["lane"].isInt() || !args["enable"].isBool()) {
             SendError(id, "BAD_ARGS", "Missing lane or enable");
             return;
        }
        int lane = args["lane"];
        bool enable = args["enable"];
        _mmu->SetAutoFeed(lane, enable);
        SendOk(id);
    }

    void HandleGetFilamentInfo(int id, JsonObject args) {
         if (!_mmu) return;
         if(!args["lane"].isInt()) { SendError(id, "BAD_ARGS", "Missing lane"); return; }
         int lane = args["lane"];
         if(lane < 0 || lane >= 4) { SendError(id, "BAD_ARGS", "Invalid lane"); return; }
         
         FilamentState &f = _mmu->GetFilament(lane);
         
         // Sanitize strings - replace non-printable chars with null terminator
         char safe_id[9]; 
         for(int j=0; j<8; j++) {
             safe_id[j] = (f.ID[j] >= 32 && f.ID[j] < 127) ? f.ID[j] : '\0';
             if(safe_id[j] == '\0') break;
         }
         safe_id[8] = '\0';
         
         char safe_name[21];
         for(int j=0; j<20; j++) {
             safe_name[j] = (f.name[j] >= 32 && f.name[j] < 127) ? f.name[j] : '\0';
             if(safe_name[j] == '\0') break;
         }
         safe_name[20] = '\0';

         float meters_f = f.meters;
         if (!isfinite(meters_f)) meters_f = 0;
         if (meters_f > 2000000000.0f) meters_f = 2000000000.0f;
         if (meters_f < -2000000000.0f) meters_f = -2000000000.0f;

         int m_int = (int)meters_f;
         int m_dec = (int)((meters_f - m_int) * 100);
         if(m_dec < 0) m_dec = -m_dec;
         int p_int = f.pressure / 1000;
         int p_dec = f.pressure % 1000;

         WaitTX();
         // Precision Fix: Handle negative sign for meters between -1.0 and 0.0
         const char* sign = (meters_f < 0 && m_int == 0) ? "-" : "";

         int len = snprintf(global_json_buf, sizeof(global_json_buf), 
            "{\"id\":%d,\"cmd\":\"GET_FILAMENT_INFO\",\"ok\":true,\"lane\":%d,\"meters\":%s%d.%02d,\"pressure\":%d.%03d,\"rfid\":\"%s\",\"name\":\"%s\",\"temp_min\":%d,\"temp_max\":%d,\"color\":[%d,%d,%d,%d]}\r\n",
            id, lane, sign, m_int, m_dec, p_int, p_dec, safe_id, safe_name, f.temperature_min, f.temperature_max, f.color_R, f.color_G, f.color_B, f.color_A
         );
         
         if (len < 0 || len >= (int)sizeof(global_json_buf)) {
             SendError(id, "BUFFER_OVERFLOW", "Response too large");
             return;
         }
         
         if (_transport) _transport->Write((const uint8_t*)global_json_buf, len);
    }

    void HandleSetFilamentInfo(int id, JsonObject args) {
         if (!_mmu) return;
         if(!args["lane"].isInt()) { SendError(id, "BAD_ARGS", "Missing lane"); return; }
         int lane = args["lane"];
         
         FilamentInfo info;
         FilamentState &current = _mmu->GetFilament(lane);
         memcpy(info.ID, current.ID, sizeof(info.ID));
         memcpy(info.name, current.name, sizeof(info.name));
         info.color_R = current.color_R;
         info.color_G = current.color_G;
         info.color_B = current.color_B;
         info.color_A = current.color_A;
         info.temperature_min = current.temperature_min;
         info.temperature_max = current.temperature_max;

         if(args["id_str"].isString()) {
             if (strlen(args["id_str"]) > 8) { SendError(id, "TOO_LONG", "ID too long (max 8)"); return; }
             info.SetID(args["id_str"]);
         }
         if(args["name"].isString()) {
             if (strlen(args["name"]) > 20) { SendError(id, "TOO_LONG", "Name too long (max 20)"); return; }
             info.SetName(args["name"]);
         }
         if(args["temp_min"].isInt()) info.temperature_min = (uint16_t)(int)args["temp_min"];
         if(args["temp_max"].isInt()) info.temperature_max = (uint16_t)(int)args["temp_max"];
         
         if(args["color"].isArray()) {
              LiteArray& c = args["color"].getArray();
              if(c.size() >= 3) {
                  info.color_R = (uint8_t)c.getInt(0); info.color_G = (uint8_t)c.getInt(1); info.color_B = (uint8_t)c.getInt(2);
                  if(c.size() > 3) info.color_A = (uint8_t)c.getInt(3); else info.color_A = 255;
              }
          }
         
         float meters = -1.0f;
         // Accept both float and int since Python may send 0 instead of 0.0
         if(args["meters"].isFloat()) meters = args["meters"];
         else if(args["meters"].isInt()) meters = (float)args["meters"].asInt();
         
         _mmu->SetFilamentInfoAction(lane, info, meters);
         SendOk(id);
    }

    void ProcessPacket(char* json_str) {
        // Guard against null or empty input
        if (!json_str || json_str[0] == '\0') {
            const char* err = "{\"ok\":false,\"msg\":\"JSON Parse Error\",\"error\":\"Empty packet\"}\n";
            if (_transport) _transport->Write((const uint8_t*)err, strlen(err));
            return;
        }
        
        // ASCII Guard: DISCARD packets with non-printable or high-bit binary chars
        for (int i = 0; json_str[i] != '\0'; i++) {
            unsigned char c = (unsigned char)json_str[i];
            if ((c < 32 && c != '\t' && c != '\r' && c != '\n') || c > 126) {
                WaitTX();
                const char* err = "{\"ok\":false,\"msg\":\"JSON Parse Error\",\"error\":\"Binary garbage detected\"}\n";
                if (_transport) _transport->Write((const uint8_t*)err, strlen(err));
                return;
            }
        }
        
        if (_mmu) _mmu->UpdateConnectivity(true);
        
        doc.clear();
        DeserializationResult error = deserializeJson(doc, json_str);

        if (error) {
            // Debug: Echo back what was received (truncated to 100 chars)
            WaitTX();
            static char err_buf[256];
            char truncated[101];
            strncpy(truncated, json_str, 100);
            truncated[100] = '\0';
            snprintf(err_buf, sizeof(err_buf), 
                "{\"ok\":false,\"msg\":\"JSON Parse Error\",\"received\":\"%s\",\"error\":\"%s\"}\n",
                truncated, error.c_str());
            if (_transport) _transport->Write((const uint8_t*)err_buf, strlen(err_buf));
            return;
        }

        int id = doc["id"] | 0;
        const char* cmd = doc["cmd"];
        
        // Get args from nested object, or use root if not present
        LiteObject* argsPtr = doc["args"].getObject();
        LiteObject& args = argsPtr ? *argsPtr : doc.root();

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

    void Init(MMU_Logic* mmu, I_MMU_Transport* transport) {
        _mmu = mmu;
        _transport = transport;
        const char* startup = "{\"event\":\"STARTUP\",\"msg\":\"KlipperCLI Ready\"}\r\n";
        if (_transport) _transport->Write((const uint8_t*)startup, strlen(startup));
    }

    void Run() {
        if (!_transport) return;
        
        // Poll transport for incoming bytes and process packets immediately
        int bytes_to_read = 64; // Limit per run to avoid stalling main loop
        while (_transport->Available() > 0 && bytes_to_read-- > 0) {
            int b = _transport->Read();
            if (b < 0) break;
            
            if (b == '\n' || b == '\r') {
                if (b == '\n' && last_was_cr) {
                    // Skip \n if it follows \r
                    last_was_cr = false;
                    continue;
                }
                last_was_cr = (b == '\r');
                
                rx_buffer[rx_idx] = '\0';
                last_activity_time = millis(); // Update activity timestamp for smart save timing
                ProcessPacket(rx_buffer);
                rx_idx = 0;
            } else {
                last_was_cr = false;
                if (rx_idx < (int)sizeof(rx_buffer) - 1) {
                    rx_buffer[rx_idx++] = (char)b;
                } else {
                    // Buffer overflow - discard and reset
                    rx_idx = 0; 
                }
            }
        }
    }
    
    bool IsConnected() {
        return _transport && _transport->IsConnected();
    }
    
    // Returns true if no serial activity for the specified duration (ms)
    bool IsSerialIdle(uint32_t idle_ms) {
        return (millis() - last_activity_time) > idle_ms;
    }
}

// Global wrapper for cross-module access (used by MMU_Logic for smart save timing)
bool KlipperCLI_IsSerialIdle(uint32_t idle_ms) {
    return KlipperCLI::IsSerialIdle(idle_ms);
}
