#pragma once
#include <Arduino.h>

class AS5600_soft_IIC
{
public:
    AS5600_soft_IIC();
    ~AS5600_soft_IIC();
    void init(uint32_t *GPIO_SCL, uint32_t *GPIO_SDA, int num);

    bool *online;
    enum _AS5600_magnet_stu
    {
        low = 1,
        high = 2,
        offline = -1,
        normal = 0
    } *
        magnet_stu;
    uint16_t *raw_angle;

    void updata_stu();
    void updata_angle();
    int numbers;
    uint16_t *data;

private:
    int *error;
    uint32_t *IO_SDA;
    uint32_t *IO_SCL;
    GPIO_TypeDef **port_SDA;
    uint16_t *pin_SDA;
    GPIO_TypeDef **port_SCL;
    uint16_t *pin_SCL;

    void init_iic();
    void start_iic(unsigned char ADR);
    void stop_iic();
    void write_iic(uint8_t byte);
    void read_iic(bool ack);
    void wait_ack_iic();
    void clear_datas();
    void read_reg8(uint8_t reg);
    void read_reg16(uint8_t reg);
};