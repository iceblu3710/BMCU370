#pragma once

#include <stdint.h>
#include <math.h>
#include "MMU_Defs.h"
#include "UnitState.h" // For FilamentState and FilamentInfo
#include "I_MMU_Hardware.h"

// --- Internal Configuration Constants ---
// (Could be moved to a config file)
#define MOTOR_SPEED_SEND 1500
#define MOTOR_SPEED_AMS_LITE_SEND 1000
#define MOTOR_SPEED_SLOW_SEND 800
#define MOTOR_SPEED_PULL 2000

// --- PID Helper Class ---
class MOTOR_PID
{
    float P = 0, I = 0, D = 0, I_save = 0, E_last = 0;
    float pid_MAX = 1000, pid_MIN = -1000, pid_range = 1000;
public:
    MOTOR_PID(float P_set, float I_set, float D_set) { Init(P_set, I_set, D_set); }
    MOTOR_PID() {}
    
    void Init(float P_set, float I_set, float D_set) { P = P_set; I = I_set; D = D_set; I_save = 0; E_last = 0; }
    void Clear() { I_save = 0; E_last = 0; }
    
    float Calculate(float E, float time_E) {
        I_save += I * E * time_E;
        if (I_save > pid_range) I_save = pid_range;
        else if (I_save < -pid_range) I_save = -pid_range;
        
        float output;
        if (time_E != 0) output = P * E + I_save + D * (E - E_last) / time_E;
        else output = P * E + I_save;
        
        if (output > pid_MAX) output = pid_MAX;
        if (output < pid_MIN) output = pid_MIN;
        E_last = E;
        return output;
    }
};

enum class filament_motion_enum
{
    stop,
    send,
    pull,
    slow_send,
    pressure_ctrl_idle,
    pressure_ctrl_in_use, 
    pressure_ctrl_on_use,
    velocity_control
};

enum class pressure_control_enum { less_pressure, all, over_pressure };

// --- Motor Channel Class ---
class MotorChannel {
public:
    int CHx;
    filament_motion_enum motion = filament_motion_enum::stop;
    uint64_t motor_stop_time = 0;
    MOTOR_PID PID_speed; 
    MOTOR_PID PID_pressure; 
    float pwm_zero = 500;
    float dir = 0; 
    float target_velocity = 0; 
    float target_distance = 0; 
    float accumulated_distance = 0; 
    
    MotorChannel() : CHx(0) {} // Default
    MotorChannel(int ch) : CHx(ch) {
        PID_speed.Init(2, 20, 0);
        PID_pressure.Init(1500, 0, 0);
    }
    
    void Init(int ch) {
        CHx = ch;
        PID_speed.Init(2, 20, 0);
        PID_pressure.Init(1500, 0, 0);
    }

    void SetMotion(filament_motion_enum m) {
        if (motion != m) {
            motion = m;
            PID_speed.Clear();
            accumulated_distance = 0; 
        }
    }
    
    // Logic specific calculation - needs access to sensor data?
    // In ControlLogic, GetXByPressure accessed global MC_PULL_stu_raw.
    // If we move this to MMU_Logic, we should pass the sensor value in.
    float CalculatePressureOutput(float current_pressure, float control_voltage, float time_E, pressure_control_enum control_type, float sign);
    
    // Main Run requires logic context (sensors). 
    // We will separate logic: MMU_Logic updates Motors.
    // So MotorChannel is just state + PID. The "Run" logic should belong to MMU_Logic or passed dependencies.
    // For simplicity, we'll keep simple state here.
};

enum filament_now_position_enum
{
    filament_idle,
    filament_sending_out,
    filament_using,
    filament_pulling_back,
    filament_redetect,
    filament_loading, 
    filament_unloading,
};

struct alignas(4) flash_save_struct
{
    FilamentState filament[4]; 
    int BambuBus_now_filament_num = 0;
    uint8_t filament_use_flag = 0x00;
    uint32_t boot_mode = 1; // Default Klipper
    uint32_t version = 5; 
    uint32_t check = 0x40614061;
};

struct Motion_control_save_struct {
    uint32_t check;
    int Motion_control_dir[4]; 
    uint8_t padding[64]; 
};

// --- MMU Logic Class ---
class MMU_Logic {
public:
    MMU_Logic(I_MMU_Hardware* hal);
    
    void Init();
    void Run();
    
    // Connectivity
    void UpdateConnectivity(bool online);
    
    // Actions
    void SetFilamentInfoAction(int id, const FilamentInfo& info, float meters = -1.0f);
    void StartLoadFilament(int tray, int length_mm = -1);
    void StartUnloadFilament(int tray, int length_mm = -1);
    void SetAutoFeed(int lane, bool enable);
    void SetCurrentFilamentIndex(int index);
    
    // Klipper Primitives
    void MoveAxis(int axis, float dist_mm, float speed);
    void StopAll(); 
    uint16_t GetSensorState();  
    int GetLaneMotion(int lane);
    
    // Accessors (Replacement for UnitState)
    FilamentState& GetFilament(int index);
    int GetCurrentFilamentIndex();
    uint16_t GetDeviceType();
    
    // Persistence
    void SaveSettings();
    void SetNeedToSave();

private:
    I_MMU_Hardware* _hal;
    
    // State
    flash_save_struct data_save;
    Motion_control_save_struct mc_save;
    MotorChannel motors[4];
    
    filament_now_position_enum filament_now_position[4];
    
    // Sensor Cache
    float speed_as5600[4];
    float MC_PULL_stu_raw[4];
    int MC_PULL_stu[4];
    float MC_ONLINE_key_stu_raw[4];
    int MC_ONLINE_key_stu[4];
    
    bool Assist_send_filament[4];
    bool pull_state_old;
    bool is_backing_out;
    float last_total_distance[4];
    int32_t as5600_distance_save[4];
    
    int32_t unload_target_dist[4];
    float unload_start_meters[4];
    
    bool Bambubus_need_to_save;
    uint64_t save_timer;
    
    bool is_connected;
    uint64_t last_heartbeat_time;
    uint16_t device_type_addr;
    
    // Constants
    const bool is_two = true; // AMS Lite logic
    const float PULL_voltage_up = 1.85f;
    const float PULL_voltage_down = 1.45f;
    const uint32_t use_flash_addr = 0x0800F000;
    const uint32_t Motion_control_save_flash_addr = 0x0800E000;

    // Internal Methods
    void motor_motion_switch();
    void MC_PULL_ONLINE_read();
    void AS5600_Update(float time_E);
    bool Prepare_For_filament_Pull_Back(float_t OUT_filament_meters);
    void UpdateLEDStatus(int channel);
    void RunMotorChannel(int channel, float time_E);
    void LoadSettings();
    
    // Helper
    uint64_t get_time64() { return _hal->GetTimeMS(); }
};
