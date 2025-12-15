#include "CommandRouter.h"
#include "BambuBusProtocol.h"
#include "Hardware.h" // For UART and Time
#include "ControlLogic.h" // Will be created next

namespace CommandRouter {

    // Forward declaration
    void HandlePacket(uint8_t* buffer, uint16_t length);

    void Init() {
        BambuBusProtocol::Init();
        Hardware::UART_SetRxCallback([](uint8_t byte) {
            if (BambuBusProtocol::ParseByte(byte)) {
                HandlePacket(BambuBusProtocol::GetRxBuffer(), BambuBusProtocol::GetRxLength());
            }
        });
    }

    void HandlePacket(uint8_t* buffer, uint16_t length) {
        BambuBus_package_type type = BambuBusProtocol::IdentifyPacket(buffer, length);
        
        // Notify ControlLogic about connectivity/heartbeat if needed
        if (type != BambuBus_package_type::NONE && type != BambuBus_package_type::ERROR) {
             ControlLogic::UpdateConnectivity(true);
        } else if (type == BambuBus_package_type::ERROR) {
             ControlLogic::UpdateConnectivity(false);
        }

        switch (type) {
            case BambuBus_package_type::filament_motion_short:
                // Handle 0xC5 0x03
                // Call ControlLogic to process motion short
                ControlLogic::ProcessMotionShort(buffer, length);
                break;
                
            case BambuBus_package_type::filament_motion_long:
                 // 0xC5 0x04
                 ControlLogic::ProcessMotionLong(buffer, length);
                 break;

            case BambuBus_package_type::online_detect:
                 // 0xC5 0x05
                 ControlLogic::ProcessOnlineDetect(buffer, length);
                 break;

            case BambuBus_package_type::REQx6:
                 // 0xC5 0x06
                 ControlLogic::ProcessREQx6(buffer, length);
                 break;

            case BambuBus_package_type::NFC_detect:
                 // 0xC5 0x07
                 // Not implemented in original code? or just return
                 break;

            case BambuBus_package_type::set_filament_info:
                 // 0xC5 0x08
                 ControlLogic::ProcessSetFilamentInfo(buffer, length);
                 break;
                 
            case BambuBus_package_type::heartbeat:
                 // 0xC5 0x20
                 ControlLogic::ProcessHeartbeat(buffer, length);
                 break;
            
            // Long packets (0x05 ...)
            case BambuBus_package_type::MC_online: // 0x21A
            case BambuBus_package_type::read_filament_info: // 0x211
            case BambuBus_package_type::set_filament_info_type2: // 0x218
            case BambuBus_package_type::version: // 0x103
            case BambuBus_package_type::serial_number: // 0x402
                 {
                     long_packge_data data;
                     BambuBusProtocol::ParseLongPacket(buffer, length, &data);
                     ControlLogic::ProcessLongPacket(data);
                 }
                 break;
                 
            default:
                break;
        }
    }

    void Run() {
        // Polling logic is replaced by Interrupt callback mechanism.
    }
    
    // Allow ControlLogic to send
    void SendPacket(uint8_t *data, uint16_t length) {
        Hardware::UART_Send(data, length);
    }
}
