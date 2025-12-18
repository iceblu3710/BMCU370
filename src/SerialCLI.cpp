#include "SerialCLI.h"
#include "CommandRouter.h" // For SendPacket if needed or generic defines
#include "ControlLogic.h"
#include "Hardware.h" // For UART_Send
#include "UnitState.h" // For FilamentInfo
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>



namespace SerialCLI {

    static char line_buffer[1024];
    static int line_idx = 0;

    void Reset() {
        line_idx = 0;
        line_buffer[0] = '\0';
    }

    void Init() {
        // Unit Info is now compile-time constant in Hardware.h / UnitState

        const char* msg = "\r\nStandardSerial CLI Ready (v2)\r\nType HELP for commands\r\n> ";
        Hardware::UART_Send((const uint8_t*)msg, strlen(msg));
    }

    void PrintHelp() {
        const char* help = "\r\nAvailable Commands:\r\n"
                           "  MOTION_SHORT <AMS_HEX> <FLAGS_HEX> <READ_HEX> <MO_FLAG_HEX>\r\n"
                           "  SET_FILAMENT <ID> <INFO_STRUCT_HEX?>\r\n"
                           "  SET_ONLINE\r\n"
                           "  SET_OFFLINE\r\n"
                           "  READ_STATUS\r\n"
                           "  READ_FILAMENT\r\n"
                           "  READ_SERIAL\r\n"
                           "  READ_ID\r\n"
                           "  SET_ID\r\n"
                           "  HELP\r\n"
                           "> ";
        Hardware::UART_Send((const uint8_t*)help, strlen(help));
    }

    char* GetInitBuffer() { return line_buffer; }
    int GetLength() { return line_idx; }

    bool Accumulate(uint8_t byte) {
        // Echo input safely
        Hardware::UART_SendByte(byte);

        if (byte == '\n' || byte == '\r') {
             if (line_idx > 0) {
                 line_buffer[line_idx] = '\0';
                 // If triggered by \r, next char might be \n. 
                 // If we return true here, reset happens.
                 // Next char \n will result in line_idx=0, caught by below.
                 return true;
             }
             // Empty line or LF after CR
             return false; 
        } else if (byte == 0x08 || byte == 0x7F) { // Backspace
            if (line_idx > 0) {
                line_idx--;
                const uint8_t bs[] = {0x08, ' ', 0x08};
                for(int i=0; i<3; i++) Hardware::UART_SendByte(bs[i]);
            }
            return false;
        } else {
            if (line_idx < 1023) {
                line_buffer[line_idx++] = (char)byte;
            } else {
                line_idx = 0; // Reset on overflow
            }
        }
        return false;
    }

    void ToUpperLine(char* str) {
        while(*str) {
            *str = toupper((unsigned char)*str);
            str++;
        }
    }

    void Parse(char* line) {
        // Case Insensitive: Upper case the whole line before parsing
        ToUpperLine(line);

        char* token = strtok(line, " ");
        if (!token) {
             Hardware::UART_Send((const uint8_t*)"> ", 2);
             return;
        }

        if (strcmp(token, "MOTION_SHORT") == 0) {
            char* ams_s = strtok(NULL, " ");
            char* flags_s = strtok(NULL, " ");
            char* read_s = strtok(NULL, " ");
            char* mo_s = strtok(NULL, " ");
            
            if (ams_s && flags_s && read_s && mo_s) {
                uint8_t ams = (uint8_t)strtol(ams_s, NULL, 16);
                uint8_t flags = (uint8_t)strtol(flags_s, NULL, 16);
                uint8_t read = (uint8_t)strtol(read_s, NULL, 16);
                uint8_t mo = (uint8_t)strtol(mo_s, NULL, 16);
                
                ControlLogic::ProcessMotionShortLogic(ams, flags, read, mo);
                Hardware::UART_Send((const uint8_t*)"\r\nOK\r\n", 6);
            } else {
                Hardware::UART_Send((const uint8_t*)"\r\nERR: Args\r\n", 11);
            }

        } else if (strcmp(token, "LOAD_FILAMENT") == 0) {
            char* tray_s = strtok(NULL, " ");
            char* len_s = strtok(NULL, " ");
            if (tray_s) {
                int tray = atoi(tray_s);
                int length = -1;
                if (len_s) length = atoi(len_s);
                ControlLogic::StartLoadFilament(tray, length);
                Hardware::UART_Send((const uint8_t*)"\r\nOK: Loading...\r\n", 18);
            } else {
                Hardware::UART_Send((const uint8_t*)"\r\nERR: Args\r\n", 11);
            }

        } else if (strcmp(token, "UNLOAD_FILAMENT") == 0) {
            char* tray_s = strtok(NULL, " ");
            char* len_s = strtok(NULL, " ");
            if (tray_s) {
                int tray = atoi(tray_s);
                int length = -1;
                if (len_s) {
                    length = atoi(len_s);
                }
                ControlLogic::StartUnloadFilament(tray, length);
                Hardware::UART_Send((const uint8_t*)"\r\nOK: Unloading...\r\n", 20);
            } else {
                Hardware::UART_Send((const uint8_t*)"\r\nERR: Args\r\n", 11);
            }

        } else if (strcmp(token, "SET_OFFLINE") == 0) {
            ControlLogic::UpdateConnectivity(false);
            Hardware::UART_Send((const uint8_t*)"\r\nOK: Offline\r\n", 15);
            
        } else if (strcmp(token, "SET_ONLINE") == 0) {
            ControlLogic::UpdateConnectivity(true);
            Hardware::UART_Send((const uint8_t*)"\r\nOK: Online\r\n", 14);
            
        } else if (strcmp(token, "READ_STATUS") == 0) {
             char buf[128];
             for(int i=0; i<4; i++) {
                 FilamentState& f = UnitState::GetFilament(i);
                 // Using simple int cast for pressure to print as decimal
                 // Assuming meters is float
                 int p_whole = (int)f.meters;
                 int p_frac = (int)((f.meters - p_whole) * 100);
                 sprintf(buf, "Tray %d: %d.%02dm P:%u Status:%d\r\n", i, p_whole, p_frac, f.pressure, (int)f.status);
                 Hardware::UART_Send((const uint8_t*)buf, strlen(buf));
             }

        } else if (strcmp(token, "SET_FILAMENT") == 0) {
            // SET_FILAMENT <ID> <R> <G> <B> <A> <TempMin> <TempMax> <TypeString> <RFID>
            // Example: SET_FILAMENT 0 FF 00 00
            char* id_s = strtok(NULL, " ");
            if (id_s) {
                 int id = atoi(id_s);
                 // Manually copy existing fields since types differ or use clean init
                 FilamentState& f = UnitState::GetFilament(id);
                 FilamentInfo info;
                 memcpy(info.ID, f.ID, 8);
                 info.color_R = f.color_R;
                 info.color_G = f.color_G;
                 info.color_B = f.color_B;
                 info.color_A = f.color_A;
                 info.temperature_min = f.temperature_min;
                 info.temperature_max = f.temperature_max;
                 memcpy(info.name, f.name, 20);

                 // Simple Color update example
                 char* r_s = strtok(NULL, " ");
                 char* g_s = strtok(NULL, " ");
                 char* b_s = strtok(NULL, " ");
                 if (r_s && g_s && b_s) {
                     info.color_R = (uint8_t)strtol(r_s, NULL, 16);
                     info.color_G = (uint8_t)strtol(g_s, NULL, 16);
                     info.color_B = (uint8_t)strtol(b_s, NULL, 16);
                     ControlLogic::SetFilamentInfoAction(id, info);
                     Hardware::UART_Send((const uint8_t*)"\r\nOK: Set Color\r\n", 17);
                 }
            }
            
        } else if (strcmp(token, "READ_FILAMENT") == 0) {
             char buf[64];
             int idx = UnitState::GetCurrentFilamentIndex();
             sprintf(buf, "\r\nCurrent Tray: %d\r\n", idx);
             Hardware::UART_Send((const uint8_t*)buf, strlen(buf));
             
             for(int i=0; i<4; i++) {
                 FilamentState& f = UnitState::GetFilament(i);
                 sprintf(buf, "Tray %d: %s [%02X%02X%02X]\r\n", i, f.name, f.color_R, f.color_G, f.color_B);
                 Hardware::UART_Send((const uint8_t*)buf, strlen(buf));
             }

        } else if (strcmp(token, "READ_SERIAL") == 0) {
             char buf[128];
             sprintf(buf, "\r\nModel: %s\r\nVersion: %s\r\nSerial: %s\r\n", 
                     UnitState::GetModel(), UnitState::GetVersion(), UnitState::GetSerialNumber());
             Hardware::UART_Send((const uint8_t*)buf, strlen(buf));

        } else if (strcmp(token, "READ_ID") == 0) {
             char buf[32];
             sprintf(buf, "\r\nBus Address: 0x%04X\r\n", UnitState::GetBusAddress());
             Hardware::UART_Send((const uint8_t*)buf, strlen(buf));

        } else if (strcmp(token, "SET_ID") == 0) {
            char* addr_s = strtok(NULL, " ");
            if (addr_s) {
                uint16_t addr = (uint16_t)strtol(addr_s, NULL, 16);
                UnitState::SetBusAddress(addr);
                Hardware::UART_Send((const uint8_t*)"\r\nOK: Set Bus ID\r\n", 18);
            } else {
                 Hardware::UART_Send((const uint8_t*)"\r\nERR: Args\r\n", 11);
            }

        } else if (strcmp(token, "HELP") == 0 || strcmp(token, "?") == 0) {
            PrintHelp();
        } else {
            Hardware::UART_Send((const uint8_t*)"\r\nUnknown Command: ", 19);
            Hardware::UART_Send((const uint8_t*)token, strlen(token));
            Hardware::UART_Send((const uint8_t*)"\r\n", 2);
        }
        
        Hardware::UART_Send((const uint8_t*)"> ", 2);
    }
}
