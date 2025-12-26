
#include "AS5600.h"

#define I2C_DELAY 1
#define SET_H(port, pin)                   \
    {                                      \
        for (auto i = 0; i < numbers; i++) \
        {                                  \
            if (error[i] == 0)             \
                port[i]->BSHR = pin[i];    \
        }                                  \
    }
#define SET_L(port, pin)                   \
    {                                      \
        for (auto i = 0; i < numbers; i++) \
        {                                  \
            if (error[i] == 0)             \
                port[i]->BCR = pin[i];     \
        }                                  \
    }
#define iic_delay()            \
    {                          \
        delayMicroseconds(10); \
    }

// address
#define AS5600_write_address (0x36 << 1)
#define AS5600_read_address ((0x36 << 1) + 1)
// angle
#define AS5600_raw_angle 0x0C
#define AS5600_angle 0x0E

// status reg
#define AS5600_status 0x0B
#define AS5600_agc 0x1A
#define AS5600_magnitude 0x1B
#define AS5600_burn 0xFF

/**
 * @brief Construct a new AS5600_soft_IIC object.
 * 
 * Initialized with zero numbers. Call init() to set up.
 */
AS5600_soft_IIC::AS5600_soft_IIC()
{
    numbers = 0;
}
/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Destroy the AS5600_soft_IIC object.
 * 
 * Frees all dynamically allocated memory for arrays.
 */

AS5600_soft_IIC::~AS5600_soft_IIC()
{
    if (numbers > 0)
    {
        delete IO_SDA;
        delete IO_SCL;
        delete port_SDA;
        delete port_SCL;
        delete pin_SDA;
        delete pin_SCL;
        delete online;
        delete magnet_stu;
        delete error;
        delete raw_angle;
        delete data;
    }
}

/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Initialize the Soft I2C instance for multiple AS5600 sensors.
 * 
 * Allocates memory for arrays based on the number of sensors and initializes
 * the GPIO pins for Software I2C.
 * 
 * @param GPIO_SCL Array of SCL pin numbers.
 * @param GPIO_SDA Array of SDA pin numbers.
 * @param num Number of sensors (pairs of SCL/SDA).
 */
void AS5600_soft_IIC::init(uint32_t *GPIO_SCL, uint32_t *GPIO_SDA, int num)
{
    numbers = num;
    online = (new bool[numbers]);
    magnet_stu = (new _AS5600_magnet_stu[numbers]);
    error = (new int[numbers]);
    raw_angle = (new uint16_t[numbers]);
    data = (new uint16_t[numbers]);
    IO_SDA = (new uint32_t[numbers]);
    IO_SCL = (new uint32_t[numbers]);
    port_SDA = (new GPIO_TypeDef *[numbers]);
    port_SCL = (new GPIO_TypeDef *[numbers]);
    pin_SDA = (new uint16_t[numbers]);
    pin_SCL = (new uint16_t[numbers]);
    for (auto i = 0; i < numbers; i++)
    {
        IO_SDA[i] = GPIO_SDA[i];
        IO_SCL[i] = GPIO_SCL[i];
        port_SDA[i] = get_GPIO_Port(CH_PORT(digitalPinToPinName(IO_SDA[i])));
        pin_SDA[i] = CH_GPIO_PIN(digitalPinToPinName(IO_SDA[i]));
        port_SCL[i] = get_GPIO_Port(CH_PORT(digitalPinToPinName(IO_SCL[i])));
        pin_SCL[i] = CH_GPIO_PIN(digitalPinToPinName(IO_SCL[i]));
        magnet_stu[i] = offline;
        online[i] = false;
        raw_angle[i] = 0;
    }

    init_iic();
    updata_stu();
}
/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Clear internal data and error flags.
 * 
 * Resets the error and data arrays to zero.
 */
void AS5600_soft_IIC::clear_datas()
{
    for (int i = 0; i < numbers; i++)
    {
        error[i] = 0;
        data[i] = 0;
    }
}
/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Update the status of all sensors.
 * 
 * Reads the status register of each AS5600 sensor to determine if the magnet
 * is detected and if the signal strength is appropriate (low/high/normal).
 * Updates the `online` and `magnet_stu` arrays.
 */
void AS5600_soft_IIC::updata_stu()
{
    read_reg8(AS5600_status);
    for (int i = 0; i < numbers; i++)
    {
        if (error[i])
            online[i] = false;
        else
            online[i] = true;
        if (!(data[i] & 0x20))
        {
            magnet_stu[i] = offline;
        }
        else
        {
            if (data[i] & 0x10)
                magnet_stu[i] = low;
            else if (data[i] & 0x08)
                magnet_stu[i] = high;
            else
                magnet_stu[i] = normal;
        }
    }
}
/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Read the raw angle from all sensors.
 * 
 * Reads the raw angle register (12-bit) from each sensor. Validates validity
 * via the `online` status. Updates `raw_angle`.
 */
void AS5600_soft_IIC::updata_angle()
{
    read_reg16(AS5600_raw_angle);
    for (auto i = 0; i < numbers; i++)
    {
        if (error[i] == 0)
        {
            raw_angle[i] = data[i];
            online[i] = true;
        }
        else
        {
            raw_angle[i] = 0;
            online[i] = false;
        }
    }
}
/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Initialize the low-level I2C pins.
 * 
 * Sets SCL/SDA to OUTPUT/OUTPUT_OD and writes High to idle the bus.
 */
void AS5600_soft_IIC::init_iic()
{
    for (auto i = 0; i < numbers; i++)
    {
        digitalWrite(IO_SCL[i], 1);
        digitalWrite(IO_SDA[i], 1);
        pinMode(IO_SCL[i], OUTPUT); // to open clock
        pinMode(IO_SDA[i], OUTPUT_OD);
        error[i] = 0;
    }
}
/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Generate I2C Start Condition.
 * 
 * @param ADR The 7-bit address shifted left by 1, plus R/W bit.
 */
void AS5600_soft_IIC::start_iic(unsigned char ADR)
{
    iic_delay();
    SET_H(port_SDA, pin_SDA);
    SET_H(port_SCL, pin_SCL);
    iic_delay();
    SET_L(port_SDA, pin_SDA);
    iic_delay();
    SET_L(port_SCL, pin_SCL);
    write_iic(ADR);
}

/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Generate I2C Stop Condition.
 */
void AS5600_soft_IIC::stop_iic()
{
    SET_L(port_SCL, pin_SCL);
    SET_L(port_SDA, pin_SDA);
    iic_delay();
    SET_H(port_SCL, pin_SCL);
    iic_delay();
    SET_H(port_SDA, pin_SDA);
    iic_delay();
}

/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Write a single byte to the I2C bus.
 * 
 * Bit-bangs the byte onto the SDA lines while toggling SCL.
 * 
 * @param byte The byte to write.
 */
void AS5600_soft_IIC::write_iic(uint8_t byte)
{
    for (uint8_t i = 0x80; i; i >>= 1)
    {
        iic_delay();
        if (byte & i)
        {
            SET_H(port_SDA, pin_SDA);
        }
        else
        {
            SET_L(port_SDA, pin_SDA);
        }
        SET_H(port_SCL, pin_SCL);
        iic_delay();
        SET_L(port_SCL, pin_SCL);
    }
    wait_ack_iic();
}

/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Read a single byte from the I2C bus.
 * 
 * Reads bits from SDA lines.
 * 
 * @param ack If true, send ACK after reading. If false, send NACK (used for last byte).
 */
void AS5600_soft_IIC::read_iic(bool ack)
{
    SET_H(port_SDA, pin_SDA);

    for (int i = 0; i < 8; i++)
    {

        iic_delay();
        SET_H(port_SCL, pin_SCL);
        iic_delay();
        for (int j = 0; j < numbers; j++)
        {
            data[j] <<= 1;
            if (port_SDA[j]->INDR & pin_SDA[j])
            {
                data[j] |= 0x01;
            }
        }
        SET_L(port_SCL, pin_SCL);
    }
    iic_delay();
    if (ack)
    {
        SET_L(port_SDA, pin_SDA);
    }
    else
    {
        SET_H(port_SDA, pin_SDA);
    }
    iic_delay();
    SET_H(port_SCL, pin_SCL);
    iic_delay();
    SET_L(port_SCL, pin_SCL);
    iic_delay();
}

/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Wait for ACK from the slave.
 * 
 * Checks if the slave pulls SDA low during the ACK clock pulse.
 * Sets the `error` flag for the specific channel if valid ACK is not received.
 */
void AS5600_soft_IIC::wait_ack_iic()
{
    SET_H(port_SDA, pin_SDA);
    iic_delay();
    SET_H(port_SCL, pin_SCL);
    iic_delay();
    for (auto i = 0; i < numbers; i++)
    {
        pinMode(IO_SDA[i], INPUT_PULLUP);
        if (port_SDA[i]->INDR & pin_SDA[i])
            error[i] = 1;
        pinMode(IO_SDA[i], OUTPUT_OD);
    }
    SET_L(port_SCL, pin_SCL);
    return;
}

/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Read an 8-bit register from the AS5600.
 * 
 * Performs a write-read sequence: START, Write Address, Write Register, START, Read Address, Read Data (NACK), STOP.
 * 
 * @param reg The register address to read.
 */
void AS5600_soft_IIC::read_reg8(uint8_t reg)
{
    if (!numbers)
        return;
    clear_datas();
    start_iic(AS5600_write_address);
    write_iic(reg);
    start_iic(AS5600_read_address);
    read_iic(false);
}

/* DEVELOPMENT STATE: FUNCTIONAL */
/**
 * @brief Read a 16-bit register from the AS5600.
 * 
 * Reads two sequential bytes (High byte, Low byte).
 * 
 * @param reg The starting register address to read.
 */
void AS5600_soft_IIC::read_reg16(uint8_t reg)
{
    if (!numbers)
        return;
    clear_datas();
    start_iic(AS5600_write_address);
    write_iic(reg);
    start_iic(AS5600_read_address);
    read_iic(true);
    read_iic(false);
}