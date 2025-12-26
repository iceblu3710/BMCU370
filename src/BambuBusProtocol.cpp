/*
* DEVELOPMENT STATE: EXPERIMENTAL
* This file implements the BambuBus protocol which is currently in an experimental state.
*/
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

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Initialize the static CRC engines.
 * 
 * Resets CRC8 and CRC16 instances to their initial states.
 */
void BambuBusProtocol::Init() {
    crc_8.reset(0x39, 0x66, 0, false, false);
    crc_16.reset(0x1021, 0x913D, 0, false, false);
}

// Re-implementation of RX_IRQ logic
/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Parse a single incoming byte into a protocol packet.
 * 
 * This is a state machine that accumulates bytes into `rx_buf`.
 * It handles framing (0x3D start byte), length extraction, and CRC8 validation for the header.
 * 
 * @param byte The incoming byte from the bus.
 * @return true If a full, valid packet has been received (CRC8 checked, length met). Note: CRC16 of body is not checked here.
 * @return false If packet is incomplete or invalid.
 */
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

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Get the pointer to the Receive Buffer.
 * 
 * @return uint8_t* Pointer to `rx_buf`.
 */
uint8_t* BambuBusProtocol::GetRxBuffer() {
    return rx_buf;
}

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Get the length of the last received packet.
 * 
 * @return uint16_t Length of valid data in `rx_buf`.
 */
uint16_t BambuBusProtocol::GetRxLength() {
    return rx_len;
}

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Verify the CRC16 for a complete packet.
 * 
 * Calculates CRC16 over the payload (excluding the last 2 bytes) and compares
 * it with the last 2 bytes of the packet.
 * 
 * @param data Pointer to the packet buffer.
 * @param data_length Total length of the packet.
 * @return true CRC matches.
 * @return false CRC mismatch.
 */
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

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Identify the type of BambuBus packet.
 * 
 * First validates the CRC16 using `CheckCRC16`. If valid, inspects the
 * packet headers (short 0xC5 vs long 0x05) to determine the specific message type.
 * 
 * @param buf Pointer to the packet data.
 * @param length Total length of the packet.
 * @return BambuBus_package_type Enum identifying the packet type.
 */
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

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Extract data from a "Long Packet" (0x05 type).
 * 
 * Parses the struct header from a long packet.
 * 
 * @param buf Pointer to the raw packet buffer.
 * @param data_length Total length of the raw packet.
 * @param data Pointer to a `long_packge_data` struct to populate.
 */
void BambuBusProtocol::ParseLongPacket(uint8_t *buf, uint16_t data_length, long_packge_data *data)
{
    memcpy(data, buf + 2, 11);
    data->datas = buf + 13;
    data->data_length = data_length - 15; // +2byte CRC16
}

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Finalize a packet for transmission by calculating and appending CRCs.
 * 
 * Calculates CRC8 for the header (index 3 for short, index 6 for long) and
 * CRC16 for the entire packet, appending it at the end.
 * 
 * @param data Pointer to the packet buffer constructing the message.
 * @param data_length Total length of the packet including CRC bytes placeholder.
 */
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

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Construct a "Long Packet" from a data structure.
 * 
 * Serializes the `long_packge_data` struct into `out_buffer`, adds the `3D 05`
 * preamble, and calls `BuildPacketWithCRC`.
 * 
 * @param data Pointer to the source `long_packge_data`.
 * @param out_buffer Pointer to the destination buffer.
 * @param out_length Output reference for the total generated length.
 */
void BambuBusProtocol::BuildLongPacket(long_packge_data *data, uint8_t *out_buffer, uint16_t &out_length)
{
    out_buffer[0] = 0x3D;
    out_buffer[1] = 0x05;
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

/* DEVELOPMENT STATE: EXPERIMENTAL */
/**
 * @brief Map internal BambuBus package types to a unified command type.
 * 
 * @param type The raw `BambuBus_package_type`.
 * @return UnifiedCommandType The simplified enum for logic handling.
 */
UnifiedCommandType BambuBusProtocol::GetUnifiedType(BambuBus_package_type type) {
    switch (type) {
        case BambuBus_package_type::filament_motion_short: return UnifiedCommandType::FilamentMotionShort;
        case BambuBus_package_type::filament_motion_long: return UnifiedCommandType::FilamentMotionLong;
        case BambuBus_package_type::online_detect: return UnifiedCommandType::OnlineDetect;
        case BambuBus_package_type::REQx6: return UnifiedCommandType::REQx6;
        case BambuBus_package_type::NFC_detect: return UnifiedCommandType::NFCDetect;
        case BambuBus_package_type::set_filament_info: return UnifiedCommandType::SetFilamentInfo;
        case BambuBus_package_type::heartbeat: return UnifiedCommandType::Heartbeat;
        case BambuBus_package_type::MC_online: return UnifiedCommandType::MCOnline;
        case BambuBus_package_type::read_filament_info: return UnifiedCommandType::ReadFilamentInfo;
        case BambuBus_package_type::set_filament_info_type2: return UnifiedCommandType::SetFilamentInfoType2;
        case BambuBus_package_type::version: return UnifiedCommandType::Version;
        case BambuBus_package_type::serial_number: return UnifiedCommandType::SerialNumber;
        case BambuBus_package_type::ERROR: return UnifiedCommandType::Error;
        default: return UnifiedCommandType::None;
    }
}

