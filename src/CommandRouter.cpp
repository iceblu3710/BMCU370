#include "CommandRouter.h"
#include "BambuBusProtocol.h"
#ifdef STANDARD_SERIAL
#include "SerialCLI.h"
#endif
#ifdef KLIPPER_SERIAL
#include "KlipperCLI.h"
#endif
#include "Hardware.h" 
#include <stdio.h> 
#include "ControlLogic.h"
#include <string.h> 

namespace CommandRouter {

    static uint64_t last_packet_time = 0;
    static volatile uint16_t process_len = 0;
    static volatile bool process_ready = false;

    // --- Routing Table Definition ---
    typedef void (*PacketHandler)(uint8_t* buffer, uint16_t length);

    struct RouteEntry {
        UnifiedCommandType type;
        PacketHandler handler;
        const char* name;
    };

    // Table of Routes
    static const RouteEntry routeTable[] = {
        { UnifiedCommandType::FilamentMotionShort, ControlLogic::ProcessMotionShort, "MotionShort" },
        { UnifiedCommandType::FilamentMotionLong,  ControlLogic::ProcessMotionLong,  "MotionLong"  },
        { UnifiedCommandType::OnlineDetect,        ControlLogic::ProcessOnlineDetect,"OnlineDetect"},
        { UnifiedCommandType::REQx6,               ControlLogic::ProcessREQx6,       "REQx6"       },
        { UnifiedCommandType::SetFilamentInfo,     ControlLogic::ProcessSetFilamentInfo, "SetFilament" },
        { UnifiedCommandType::Heartbeat,           ControlLogic::ProcessHeartbeat,   "Heartbeat"   },
        // Mapped Long Packets (Handler calls helper that parses long packet)
        // CommandRouter::Route logic for these was effectively parsing long packet then calling ControlLogic::ProcessLongPacket
        // We can make a helper in CommandRouter or ControlLogic. 
        // Let's use a generic handler or specific wrapper if needed.
        // For now, mapping to a specialized handler or generic one.
     };

    static const int routeTableSize = sizeof(routeTable) / sizeof(RouteEntry);

    // --- Long Packet Handlers Wrapper ---
    // Since ControlLogic::ProcessLongPacket expects parsed struct, we need a wrapper if we route raw buffer.
    void HandleLongPacket(uint8_t* buffer, uint16_t length) {
         long_packge_data data;
         BambuBusProtocol::ParseLongPacket(buffer, length, &data);
         ControlLogic::ProcessLongPacket(data);
    }

    void Init() {
#ifndef STANDARD_SERIAL
        BambuBusProtocol::Init();
#endif
        
#ifdef STANDARD_SERIAL
        Hardware::UART_SetRxCallback([](uint8_t byte) {
             if (SerialCLI::Accumulate(byte)) {
                 // Line is ready in SerialCLI internal buffer
                 char* line = SerialCLI::GetInitBuffer();
                 int len = SerialCLI::GetLength();
                 if (len < 1000) {
                     memcpy(process_buffer, line, len);
                     process_buffer[len] = 0; // Null terminate locally
                     process_len = len; 
                     process_ready = true;
                     // Reset CLI for next line
                     SerialCLI::Reset();
                 }
             }
        });
        SerialCLI::Init();
#elif defined(KLIPPER_SERIAL)
        Hardware::UART_SetRxCallback([](uint8_t byte) {
            KlipperCLI::FeedByte(byte);
        });
        KlipperCLI::Init();
#else
        Hardware::UART_SetRxCallback([](uint8_t byte) {
            if (BambuBusProtocol::ParseByte(byte)) {
                uint8_t* buffer = BambuBusProtocol::GetRxBuffer();
                uint16_t length = BambuBusProtocol::GetRxLength();
                if (length < 1000) {
                    memcpy(process_buffer, buffer, length);
                    process_len = length;
                    process_ready = true;
                }
            }
        });
#endif
    }

    void Route(UnifiedCommandType type, uint8_t* buffer, uint16_t length) {
        // Generic Connectivity Update
        if (type != UnifiedCommandType::None && type != UnifiedCommandType::Error) {
             ControlLogic::UpdateConnectivity(true);
        } else if (type == UnifiedCommandType::Error) {
             ControlLogic::UpdateConnectivity(false);
             return; 
        }

        // Table Lookup
        bool found = false;
        for(int i=0; i<routeTableSize; i++) {
            if (routeTable[i].type == type) {
                routeTable[i].handler(buffer, length);
                found = true;
                break;
            }
        }
        
        if (!found) {
            // Handle special Long Packet types that map to same logic?
            // UnifiedCommandType for Long Packets: MCOnline, ReadFilamentInfo, etc.
            // These all need parsing via ParseLongPacket.
            // We can add them to table pointing to HandleLongPacket wrapper.
            switch(type) {
                case UnifiedCommandType::MCOnline:
                case UnifiedCommandType::ReadFilamentInfo:
                case UnifiedCommandType::SetFilamentInfoType2:
                case UnifiedCommandType::Version:
                case UnifiedCommandType::SerialNumber:
                case UnifiedCommandType::FilamentMotionLong: // Also a long packet
                    HandleLongPacket(buffer, length);
                    break;
                default:
                    break;
            }
        }
    }

    void Run() {
#if defined(KLIPPER_SERIAL)
        KlipperCLI::Run();
#endif

        if (process_ready) {
            process_ready = false; // Clear flag early to allow ISR to set it again for NEXT packet if needed
#ifdef STANDARD_SERIAL
            // CLI Mode: Process Text Line
            SerialCLI::Parse((char*)process_buffer);
#elif defined(KLIPPER_SERIAL)
            KlipperCLI::Run();
#else
            // Bus Mode: Process Binary Packet
            Hardware::DelayMS(1); 
            BambuBus_package_type bb_type = BambuBusProtocol::IdentifyPacket(process_buffer, process_len);
            UnifiedCommandType unified_type = BambuBusProtocol::GetUnifiedType(bb_type);
            
            if (unified_type != UnifiedCommandType::None) {
                 last_packet_time = Hardware::GetTime();
                 Route(unified_type, process_buffer, process_len);
            } else if (bb_type == BambuBus_package_type::ERROR) {
                 Route(UnifiedCommandType::Error, process_buffer, process_len);
            }
#endif
        }

        uint64_t now = Hardware::GetTime();
        if (now - last_packet_time > 3000) { 
            ControlLogic::UpdateConnectivity(false);
        }
    }
    
    // Helper to send packet (Abstraction for Hardware)
    void SendPacket(uint8_t *data, uint16_t length) {
#ifdef STANDARD_SERIAL
        // In StandardSerial, we might want to print Debug Hex IF configured, 
        // OR we might want to convert the binary response to a human readable response?
        // But ControlLogic generated a binary packet response.
        // For SerialCLI, we probably want to intercept this and print text.
        // But since we are refactoring, `SerialCLI` should handle responses itself?
        // However, `ControlLogic` calls `CommandRouter::SendPacket`.
        // So we interpret it here.
        
        // TODO: Decode packet and print text?
        // Reuse DecodeAndPrintPacket logic from previous version or simplify?
        // For now, let's just print a "TxPacket" debug line so we see what logic sent.
        
        const char* prefix = "[TX] ";
        Hardware::UART_Send((const uint8_t*)prefix, 5);
        for(uint16_t i=0; i<length; i++) {
             char tmp[4];
             sprintf(tmp, "%02X ", data[i]);
             Hardware::UART_Send((const uint8_t*)tmp, strlen(tmp));
        }
        Hardware::UART_Send((const uint8_t*)"\r\n", 2);

#else
        Hardware::UART_Send(data, length);
#endif
    }
}
