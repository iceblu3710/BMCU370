#pragma once

#include <stdint.h>
#include <string.h>

enum class AMS_filament_stu
{
    offline,
    online,
    NFC_waiting
};

enum class AMS_filament_motion
{
    before_pull_back, // 0
    need_pull_back,   // 1
    need_send_out,    // 2
    in_use,           // 3
    idle              // 4
};

enum class BambuBus_package_type
{
    ERROR = -1,
    NONE = 0,
    filament_motion_short,
    filament_motion_long,
    online_detect,
    REQx6,
    NFC_detect,
    set_filament_info,
    MC_online,
    read_filament_info,
    set_filament_info_type2,
    version,
    serial_number,
    heartbeat,
    ETC,
    __BambuBus_package_packge_type_size
};

enum BambuBus_device_type
{
    BambuBus_none = 0x0000,
    BambuBus_AMS = 0x0700,
    BambuBus_AMS_lite = 0x1200,
};

#pragma pack(push, 1)
struct long_packge_data
{
    uint16_t package_number;
    uint16_t package_length;
    uint8_t crc8;
    uint16_t target_address;
    uint16_t source_address;
    uint16_t type;
    uint8_t *datas; // Pointer to data in buffer
    uint16_t data_length;
};
#pragma pack(pop)

class BambuBusProtocol
{
public:
    static void Init();
    
    // Feed one byte into the parser. Returns true if a full packet was just received.
    static bool ParseByte(uint8_t byte);
    
    // Retrieve the raw buffer of the last received packet
    static uint8_t* GetRxBuffer();
    static uint16_t GetRxLength();
    
    // Analyze the packet in the buffer
    static BambuBus_package_type IdentifyPacket(uint8_t* buffer, uint16_t length);
    static void ParseLongPacket(uint8_t* buffer, uint16_t length, long_packge_data *out_data);
    
    // Packet Construction
    static void BuildPacketWithCRC(uint8_t *data, uint16_t &length); // Modifies buffer in place
    static void BuildLongPacket(long_packge_data *data, uint8_t *out_buffer, uint16_t &out_length);

    // Helpers
    static bool CheckCRC16(uint8_t *data, int length);

private:
   static  uint8_t rx_buf[1000];
   static  uint16_t rx_len;
   static  bool packet_ready;
};
