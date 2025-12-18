# Hardware Operations per Unified Command

The backend `ControlLogic` translates these abstract commands into physical actions, primarily driven by an asynchronous loop interacting with the `Hardware` layer.

## Command List

1.  **`UnifiedCommandType::None`**

    - **Action**: No operation.
    - **Hardware Effect**: None.

2.  **`UnifiedCommandType::FilamentMotionShort` (0x03)**

    - **Action**: Short move command for filament. Updates `motors[x].motion` state.
    - **Hardware Effect**:
      - `ControlLogic::Run` calculates PID speed/position.
      - Calls `Hardware::PWM_Set(channel, value)` to drive motor.
      - Checks `Hardware::ADC_GetValues()` for stall/load.
      - Reads `MC_AS5600.updata_angle()` for position feedback.

3.  **`UnifiedCommandType::FilamentMotionLong` (0x04)**

    - **Action**: Long move command with more parameters.
    - **Hardware Effect**: Same as `FilamentMotionShort`, but target parameters (distance, speed) might differ. Drives PWM and reads encoders.

4.  **`UnifiedCommandType::OnlineDetect` (0x05)**

    - **Action**: Detects if AMS is connected/online.
    - **Hardware Effect**:
      - May trigger `Hardware::LED_Show()` if specific status LED logic is tied to connectivity.
      - Generates a response packet via `Hardware::UART_Send`.

5.  **`UnifiedCommandType::REQx6` (0x06)**

    - **Action**: Request for status or handshake (implementation specific).
    - **Hardware Effect**: Triggers immediate `Hardware::UART_Send` to reply.

6.  **`UnifiedCommandType::SetFilamentInfo` (0x08)**

    - **Action**: Writes filament attributes (Color, Material) to `data_save.filament[x]`.
    - **Hardware Effect**:
      - Calls `Hardware::LED_SetColor(channel, ...)` to update slot LEDs.
      - Persists data to Flash memory via `Flash_saves` (writes to internal Flash controller).

7.  **`UnifiedCommandType::Heartbeat` (0x20)**

    - **Action**: Updates `last_heartbeat_time` timestamp.
    - **Hardware Effect**: Prevents safety timeout. If timeout occurs, motors might be disabled (PWM=0).

8.  **`UnifiedCommandType::MCOnline`**

    - **Action**: Main Controller online announcement.
    - **Hardware Effect**: Updates `device_type_addr` to identify as AMS or AMS Lite.

9.  **`UnifiedCommandType::ReadFilamentInfo`**

    - **Action**: Request to read back filament data.
    - **Hardware Effect**: Triggers `Hardware::UART_Send` containing the `filament_info` data structure from memory.

10. **`UnifiedCommandType::SetFilamentInfoType2`**

    - **Action**: Alternative command to set filament info (possibly bulk or different format).
    - **Hardware Effect**: Similar to `SetFilamentInfo`. Updates memory, Flash, and LEDs.

11. **`UnifiedCommandType::Version`**

    - **Action**: Request/Report firmware version.
    - **Hardware Effect**: Triggers `Hardware::UART_Send` with version data.

12. **`UnifiedCommandType::SerialNumber`**

    - **Action**: Request/Report device Serial Number.
    - **Hardware Effect**: Triggers `Hardware::UART_Send` with SN data.

13. **`UnifiedCommandType::NFCDetect`**

    - **Action**: NFC tag detection (if hardware supported).
    - **Hardware Effect**:
      - If I2C/SPI NFC reader exists, it would be polled.
      - Currently, mostly a placeholder or triggers a state change `AMS_filament_status::NFC_waiting`.

14. **`UnifiedCommandType::Error`**

    - **Action**: Protocol error or invalid packet.
    - **Hardware Effect**:
      - May set connectivity to false (`UpdateConnectivity(false)`).
      - Could trigger red LED status via `Hardware::LED_SetColor`.

15. **`UnifiedCommandType::ETC`**
    - **Action**: Unknown/Other commands.
    - **Hardware Effect**: Generally ignored or logged.

# Hardware Execution Loop (`ControlLogic.cpp` / `Hardware.cpp`)

- **Asynchronous Loop**: `ControlLogic::Run` loops continuously (via main loop or timer).
- **Inputs**:
  - `Hardware::ADC_GetValues()`: Read current/voltage sensors.
  - `MC_AS5600.updata_angle()`: Read magnetic encoders (I2C).
  - `Hardware::GetTime()`: System ticks.
- **Logic**: PID calculation, state machine transitions.
- **Outputs**:
  - `Hardware::PWM_Set()`: Drive motor H-Bridges.
  - `Hardware::LED_SetColor()`: Update WS2812 LEDs.
  - `Hardware::UART_Send()`: Transmit response packets.
