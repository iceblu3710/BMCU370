#include "CommandRouter.h"
#include "BambuBusProtocol.h"
#include "Hardware.h" 
#include "ControlLogic.h"
#include <string.h> // For memcpy

namespace CommandRouter {

    static uint64_t last_packet_time = 0;
    
    // Double Buffering for Main Loop Process
    static uint8_t process_buffer[1000];
    static volatile uint16_t process_len = 0;
    static volatile bool process_ready = false;

    void Init() {
        BambuBusProtocol::Init();
        // Hardware UART callback now bridges to Route via BambuBusProtocol identification
        Hardware::UART_SetRxCallback([](uint8_t byte) {
            if (BambuBusProtocol::ParseByte(byte)) {
                // ISR Context: Copy to process buffer immediately to free up Protocol logic
                uint8_t* buffer = BambuBusProtocol::GetRxBuffer();
                uint16_t length = BambuBusProtocol::GetRxLength();
                
                if (length < 1000) {
                    memcpy(process_buffer, buffer, length);
                    process_len = length;
                    process_ready = true;
                }
            }
        });
    }

    void Route(UnifiedCommandType type, uint8_t* buffer, uint16_t length) {
        
        // Connectivity Logic (Generic)
        if (type != UnifiedCommandType::None && type != UnifiedCommandType::Error) {
             ControlLogic::UpdateConnectivity(true);
        } else if (type == UnifiedCommandType::Error) {
             ControlLogic::UpdateConnectivity(false);
             return; // Stop processing
        }

        switch (type) {
            case UnifiedCommandType::FilamentMotionShort:
                ControlLogic::ProcessMotionShort(buffer, length);
                break;
                
            case UnifiedCommandType::FilamentMotionLong:
                 ControlLogic::ProcessMotionLong(buffer, length);
                 break;

            case UnifiedCommandType::OnlineDetect:
                 ControlLogic::ProcessOnlineDetect(buffer, length);
                 break;

            case UnifiedCommandType::REQx6:
                 ControlLogic::ProcessREQx6(buffer, length);
                 break;

            case UnifiedCommandType::SetFilamentInfo:
                 ControlLogic::ProcessSetFilamentInfo(buffer, length);
                 break;
                 
            case UnifiedCommandType::Heartbeat:
                 ControlLogic::ProcessHeartbeat(buffer, length);
                 break;
            
            // Long packets need parsing logic.
            //Ideally, the parsing of long packet data should happen in the Protocol layer or an adapter,
            // passing a struct to Route. But for now to keep ControlLogic untouched, we pass the buffer
            // and let ControlLogic (or a small helper here) parse it if necessary.
            // However, ControlLogic::ProcessLongPacket expects 'long_packge_data'.
            // So we MUST parse it here or in the caller.
            // But this function is generic 'Route(type, buffer, len)'.
            // Specific types might need specific handling.
            
            case UnifiedCommandType::MCOnline:
            case UnifiedCommandType::ReadFilamentInfo:
            case UnifiedCommandType::SetFilamentInfoType2:
            case UnifiedCommandType::Version:
            case UnifiedCommandType::SerialNumber:
                 {
                     // This part is still protocol specific because it relies on "BambuBusProtocol::ParseLongPacket"
                     // To make it truly agnostic, the 'buffer' passed in should ideally be already parsed or standard.
                     // But since we are only refactoring the Router, we can keep the parsing here for now,
                     // acknowledging that "IdentifyPacket" was done by the caller (BambuBus specific).
                     // Ideally, the caller should have passed a generic data object.
                     // For this step, we'll keep the dependency on BambuBusProtocol::ParseLongPacket 
                     // OR we can move ParseLongPacket to be a utility that uses the raw buffer.
                     
                     // We know it's a BambuBus Long Packet if we are here.
                     // But if a Serial interface calls this with a textual command, it won't be a long packet.
                     // So this logic implies the buffer is in BambuBus format.
                     // This is acceptable for the "BambuBus" flow.
                     // A new "Serial" flow would use different CommandTypes or standardized data structs.
                     // Since we reused UnifiedCommandType for these specific BambuPacket types,
                     // it's fair to assume the payload matches that type.
                     
                     long_packge_data data;
                     // We need to know it's a BambuBus packet to parse as one.
                     // Since we are standardizing, maybe we should have "RouteBambuBusPacket(...)" vs "RouteSerial(...)"?
                     // Or just check if it parses.
                     
                     BambuBusProtocol::ParseLongPacket(buffer, length, &data);
                     ControlLogic::ProcessLongPacket(data);
                 }
                 break; // Correction: 'break' was inside block in thought, putting it out.
                 
            default:
                break;
        }
    }


    void Run() {
        // 1. Process Pending Packets (Main Loop Context)
        if (process_ready) {
            // Atomic check/clear not strictly necessary for bool on this architecture if single consumer/producer flow guarded by logic,
            // but we reset first to safely capture next one? Or allow overwrite?
            // Original code blindly overwrites. We will process then clear.
            
            Hardware::DelayMS(1); // Match legacy timing (allow turnaround or processing gap)

            // Decode to unified type
            BambuBus_package_type bb_type = BambuBusProtocol::IdentifyPacket(process_buffer, process_len);
            UnifiedCommandType unified_type = BambuBusProtocol::GetUnifiedType(bb_type);
            
            // Route it
            if (unified_type != UnifiedCommandType::None) {
                 last_packet_time = Hardware::GetTime();
                 Route(unified_type, process_buffer, process_len);
            } else if (bb_type == BambuBus_package_type::ERROR) {
                 Route(UnifiedCommandType::Error, process_buffer, process_len);
            }
            
            process_ready = false;
        }

        // 2. Check for timeout
        uint64_t now = Hardware::GetTime();
        if (now - last_packet_time > 3000) { // 3 seconds timeout
            ControlLogic::UpdateConnectivity(false);
        }
    }
    
    // Allow ControlLogic to send
    void SendPacket(uint8_t *data, uint16_t length) {
        Hardware::UART_Send(data, length);
    }
}
