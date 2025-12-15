# BMCU370 API Reference

This document outlines the API for the refactored BMCU370 codebase, separated into Hardware, Protocol, Command Router, and Control Logic layers.

## 1. Hardware Layer (`Hardware.h`)

The Hardware layer abstracts device-specific peripherals (GPIO, UART, ADC, PWM, LEDs).

### Initialization

- **`void Hardware::Init()`**
  Initializes all hardware subsystems (System clocks, Watchdog, UART, ADC, PWM, LEDs).

### Time

- **`void Hardware::DelayUS(uint32_t us)`**
  Delay for specified microseconds.
- **`void Hardware::DelayMS(uint32_t ms)`**
  Delay for specified milliseconds.
- **`uint64_t Hardware::GetTime()`**
  Returns system uptime in milliseconds (approx).

### UART (Serial)

- **`void Hardware::UART_Init()`**
  Configures UART1 at 1,250,000 baud, 9-bit data (8N1 equivalent with parity handling in hardware or logic?). _Note: Code configures 9-bit WordLength._
- **`void Hardware::UART_SetRxCallback(void (*callback)(uint8_t))`**
  Sets the callback function to be called when a byte is received via Interrupt.
- **`void Hardware::UART_Send(const uint8_t *data, uint16_t length)`**
  Sends data via DMA.

### ADC

- **`void Hardware::ADC_Init()`**
  Configures ADC1 with DMA on 8 channels.
- **`float* Hardware::ADC_GetValues()`**
  Returns a pointer to an array of 8 float values (Voltages).

### PWM (Motor Control)

- **`void Hardware::PWM_Init()`**
  Configures Timers (TIM2, TIM3, TIM4) for PWM output.
- **`void Hardware::PWM_Set(uint8_t channel, int pwm_value)`**
  Sets the PWM duty cycle for a motor channel (0-3). `pwm_value` range: -1000 to 1000.

### LED

- **`void Hardware::LED_Init()`**
  Initializes NeoPixel strips.
- **`void Hardware::LED_SetColor(uint8_t channel, int led_idx, uint8_t r, uint8_t g, uint8_t b)`**
  Sets color for a specific LED. `channel` 0-3 are filaments, 4 is mainboard.
- **`void Hardware::LED_Show()`**
  Updates the LED strips.
- **`void Hardware::LED_SetBrightness(uint8_t brightness)`**
  Sets global brightness.

---

## 2. Protocol Layer (`BambuBusProtocol.h`)

Pure logic for parsing and building BambuBus packets.

### Parsing

- **`bool BambuBusProtocol::ParseByte(uint8_t byte)`**
  Feeds a byte into the state machine. Returns `true` if a full valid packet is received.
- **`uint8_t* BambuBusProtocol::GetRxBuffer()`**
  Returns pointer to the structured packet buffer.
- **`BambuBus_package_type BambuBusProtocol::IdentifyPacket(uint8_t* buffer, uint16_t length)`**
  Identifies the type of packet in the buffer (validates CRC16).

### packet Construction

- **`void BambuBusProtocol::BuildPacketWithCRC(uint8_t *data, uint16_t &length)`**
  Appends CRC8 (header) and CRC16 (packet) to the buffer.
- **`void BambuBusProtocol::BuildLongPacket(long_packge_data *data, uint8_t *out_buffer, uint16_t &out_length)`**
  Constructs a long packet from the struct.

---

## 3. Command Router (`CommandRouter.h`)

Bridges Communication and Control Logic.

- **`void CommandRouter::Init()`**
  Initializes Protocol and registers RX callback with Hardware.
- **`void CommandRouter::Run()`**
  Main loop function (currently mostly passive as RX is interrupt-driven, but can handle TX queues).

---

## 4. Control Logic (`ControlLogic.h`)

Manages the state of the AMS/Device and Motor control loops.

### Lifecycle

- **`void ControlLogic::Init()`**
  Initializes hardware dependent logic (ADC, Motors) and loads settings from Flash.
- **`void ControlLogic::Run()`**
  Main business loop. Handles PID loops, sensor polling, and LED status updates.

### Interaction

- **`void ControlLogic::UpdateConnectivity(bool online)`**
  Updates the connection state.
- **`void ControlLogic::ProcessMotionShort(...)`**
  Handles `0xC5 0x03` packets (Motion control requests).
- **`void ControlLogic::Process...(...)`**
  Handlers for various packet types.

### State

- **`uint16_t ControlLogic::GetDeviceType()`**
  Returns the detected device type (AMS or AMS Lite).
