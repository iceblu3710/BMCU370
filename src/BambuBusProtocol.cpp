#include "BambuBusProtocol.h"
#include "CRC16.h"
#include "CRC8.h"

// Static members
uint8_t BambuBusProtocol::rx_buf[1000];
uint16_t BambuBusProtocol::rx_len = 0;
bool BambuBusProtocol::packet_ready = false;

static CRC16 crc_16;
static CRC8 crc_8;
static CRC8 _RX_IRQ_crcx(0x39, 0x66, 0x00, false, false);

void BambuBusProtocol::Init() {
    crc_8.reset(0x39, 0x66, 0, false, false);
    crc_16.reset(0x1021, 0x913D, 0, false, false);
}

// Re-implementation of RX_IRQ logic
bool BambuBusProtocol::ParseByte(uint8_t byte) {
    static int _index = 0;
    static int length = 999;
    static uint8_t data_length_index;
    static uint8_t data_CRC8_index;

    packet_ready = false;

    if (_index == 0) // waiting for first data
    {
        if (byte == 0x3D) // 0x3D-start
        {
            rx_buf[0] = 0x3D;
            _RX_IRQ_crcx.restart();       
            _RX_IRQ_crcx.add(0x3D);       
            data_length_index = 4;        
            length = data_CRC8_index = 6; 
            _index = 1;
        }
        return false;
    }
    else // have 0x3D, normal data
    {
        rx_buf[_index] = byte;
        if (_index == 1) // package type byte
        {
            if (byte & 0x80) // short head package
            {
                data_length_index = 2;
                data_CRC8_index = 3;
            }
            else // long head package
            {
                data_length_index = 4;
                data_CRC8_index = 6;
            }
        }
        if (_index == data_length_index) // the length byte
        {
            length = byte;
        }
        if (_index < data_CRC8_index) // before CRC8 byte, add data
        {
            _RX_IRQ_crcx.add(byte);
        }
        else if (_index == data_CRC8_index) // the CRC8 byte, check
        {
            if (byte != _RX_IRQ_crcx.calc()) // check error, return to waiting 0x3D
            {
                _index = 0;
                return false;
            }
        }
        ++_index;
        if (_index >= length) // recv over
        {
            _index = 0;
            rx_len = length;
            packet_ready = true;
            return true;
        }
        if (_index >= 999) // buffer overflow protection
        {
            _index = 0;
        }
    }
    return false;
}

uint8_t* BambuBusProtocol::GetRxBuffer() {
    return rx_buf;
}

uint16_t BambuBusProtocol::GetRxLength() {
    return rx_len;
}

bool BambuBusProtocol::CheckCRC16(uint8_t *data, int data_length)
{
    crc_16.restart();
    data_length -= 2;
    for (auto i = 0; i < data_length; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    if ((data[(data_length)] == (num & 0xFF)) && (data[(data_length + 1)] == ((num >> 8) & 0xFF)))
        return true;
    return false;
}

BambuBus_package_type BambuBusProtocol::IdentifyPacket(uint8_t* buf, uint16_t length) {
    if (CheckCRC16(buf, length) == false)
    {
        return BambuBus_package_type::NONE;
    }
    if (buf[1] == 0xC5)
    {
        switch (buf[4])
        {
        case 0x03: return BambuBus_package_type::filament_motion_short;
        case 0x04: return BambuBus_package_type::filament_motion_long;
        case 0x05: return BambuBus_package_type::online_detect;
        case 0x06: return BambuBus_package_type::REQx6;
        case 0x07: return BambuBus_package_type::NFC_detect;
        case 0x08: return BambuBus_package_type::set_filament_info;
        case 0x20: return BambuBus_package_type::heartbeat;
        default: return BambuBus_package_type::ETC;
        }
    }
    else if (buf[1] == 0x05)
    {
        long_packge_data info;
        ParseLongPacket(buf, length, &info);
        
        switch (info.type)
        {
        case 0x21A: return BambuBus_package_type::MC_online;
        case 0x211: return BambuBus_package_type::read_filament_info;
        case 0x218: return BambuBus_package_type::set_filament_info_type2;
        case 0x103: return BambuBus_package_type::version;
        case 0x402: return BambuBus_package_type::serial_number;
        default: return BambuBus_package_type::ETC;
        }
    }
    return BambuBus_package_type::NONE;
}

void BambuBusProtocol::ParseLongPacket(uint8_t *buf, uint16_t data_length, long_packge_data *data)
{
    memcpy(data, buf + 2, 11);
    data->datas = buf + 13;
    data->data_length = data_length - 15; // +2byte CRC16
}

void BambuBusProtocol::BuildPacketWithCRC(uint8_t *data, uint16_t &data_length)
{
    crc_8.restart();
    if (data[1] & 0x80)
    {
        for (auto i = 0; i < 3; i++) crc_8.add(data[i]);
        data[3] = crc_8.calc();
    }
    else
    {
        for (auto i = 0; i < 6; i++) crc_8.add(data[i]);
        data[6] = crc_8.calc();
    }
    crc_16.restart();
    // length calculation logic for CRC16
    // In original code, data_length was passed IN as the target length including CRC16 bytes placeholders?
    // "data_length += 2;" at the end of original function suggests the input length EXCLUDED the CRC16.
    // "data_length -= 2;" in loop suggests it iterated up to CRC pos.
    
    // Let's assume input data_length is the length of payload + headers, WITHOUT CRC16.
    // Wait, original code:
    // package_send_with_crc(uint8_t *data, int data_length)
    //   data_length -= 2; ... loop i < data_length ... 
    //   data[data_length] = ...
    //   data_length += 2;
    // So the input data_length INCLUDED the 2 bytes valid space for CRC16.
    
    int loop_len = data_length - 2;
    for (auto i = 0; i < loop_len; i++)
    {
        crc_16.add(data[i]);
    }
    uint16_t num = crc_16.calc();
    data[loop_len] = num & 0xFF;
    data[loop_len + 1] = num >> 8;
    // Length is already correct (it was passed in including the 2 bytes space)
}

void BambuBusProtocol::BuildLongPacket(long_packge_data *data, uint8_t *out_buffer, uint16_t &out_length)
{
    out_buffer[0] = 0x3D;
    out_buffer[1] = 0x00;
    data->package_length = data->data_length + 15; // 13 header + 2 crc16 ? 
    // Header is: 3D 00 (2) + struct(11) = 13?
    // struct is 11 bytes: num(2)+len(2)+crc8(1)+target(2)+source(2)+type(2) = 11.
    // total header = 3D 00 + 11 = 13.
    // + data
    // + crc16 (2)
    // = 15 + data_length. Correct.
    
    memcpy(out_buffer + 2, data, 11);
    memcpy(out_buffer + 13, data->datas, data->data_length);
    out_length = data->data_length + 15;
    BuildPacketWithCRC(out_buffer, out_length);
}
