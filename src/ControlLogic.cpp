#include "ControlLogic.h"
#include "Hardware.h"
#include "Flash_saves.h"
#include "CommandRouter.h"
#include "many_soft_AS5600.h"

// --- Data Structures (from BambuBus.cpp) ---

struct _filament
{
    // AMS status
    char ID[8] = "GFG00";
    uint8_t color_R = 0xFF;
    uint8_t color_G = 0xFF;
    uint8_t color_B = 0xFF;
    uint8_t color_A = 0xFF;
    int16_t temperature_min = 220;
    int16_t temperature_max = 240;
    char name[20] = "PETG";

    float meters = 0;
    uint64_t meters_virtual_count = 0;
    AMS_filament_stu statu = AMS_filament_stu::online;
    // printer_set
    AMS_filament_motion motion_set = AMS_filament_motion::idle;
    uint16_t pressure = 0xFFFF;
};

#define Bambubus_version 5
#define use_flash_addr ((uint32_t)0x0800F000)

struct alignas(4) flash_save_struct
{
    _filament filament[4];
    int BambuBus_now_filament_num = 0xFF;
    uint8_t filament_use_flag = 0x00;
    uint32_t version = Bambubus_version;
    uint32_t check = 0x40614061;
};

// --- Data Structures (from Motion_control.cpp) ---

#define Motion_control_save_flash_addr ((uint32_t)0x0800E000)

struct alignas(4) Motion_control_save_struct
{
    int Motion_control_dir[4];
    int check = 0x40614061;
};

// --- Internal State ---

static flash_save_struct data_save;
static Motion_control_save_struct mc_save;
static uint16_t device_type_addr = BambuBus_device_type::BambuBus_none; // Address 0=None? Original used BambuBus_address.
static bool is_connected = false;
static uint64_t last_heartbeat_time = 0;

// Motor Control State
static AS5600_soft_IIC_many MC_AS5600; 
// ... PID and Motor logic ...
// For brevity in this refactor, I will need to port the PID and Motor classes.
// I'll assume they are defined here or I should move them to a helper if they are complex.
// They were in Motion_control.cpp. I'll include them here.

#define PWM_lim 1000

class MOTOR_PID
{
    float P = 0, I = 0, D = 0, I_save = 0, E_last = 0;
    float pid_MAX = PWM_lim, pid_MIN = -PWM_lim, pid_range = PWM_lim;

public:
    MOTOR_PID(float P_set, float I_set, float D_set) { Init(P_set, I_set, D_set); }
    void Init(float P_set, float I_set, float D_set) { P = P_set; I = I_set; D = D_set; I_save = 0; E_last = 0; }
    void Clear() { I_save = 0; E_last = 0; }
    float Calculate(float E, float time_E) {
        I_save += I * E * time_E;
        if (I_save > pid_range) I_save = pid_range;
        if (I_save < -pid_range) I_save = -pid_range;
        float output = P * E + I_save + (time_E != 0 ? D * (E - E_last) / time_E : 0);
        if (output > pid_MAX) output = pid_MAX;
        if (output < pid_MIN) output = pid_MIN;
        E_last = E;
        return output;
    }
};

enum class filament_motion_enum
{
    send, redetect, slow_send, pull, stop, pressure_ctrl_in_use, pressure_ctrl_idle
};

class MotorChannel {
public:
    int CHx;
    MOTOR_PID PID_speed = MOTOR_PID(2, 20, 0);
    MOTOR_PID PID_pressure = MOTOR_PID(1500, 0, 0);
    filament_motion_enum motion = filament_motion_enum::stop;
    
    MotorChannel(int ch) : CHx(ch) {}
    
    void SetMotion(filament_motion_enum m) {
        if (motion != m) {
            motion = m;
            PID_speed.Clear();
        }
    }
    
    void Run(float time_E) {
        // ... (Logic from Motion_control.cpp implementation goes here) ...
        // Simplified placeholder for now:
        float output = 0;
        // Logic to read ADC via Hardware::ADC_GetValues()
        // Logic to read AS5600 via MC_AS5600
        // Calculate output
        Hardware::PWM_Set(CHx, output);
    }
};

static MotorChannel motors[4] = {0, 1, 2, 3};

namespace ControlLogic {

    void LoadSettings() {
        flash_save_struct *ptr = (flash_save_struct *)(use_flash_addr);
        if ((ptr->check == 0x40614061) && (ptr->version == Bambubus_version)) {
            memcpy(&data_save, ptr, sizeof(data_save));
        } else {
            // Default constants
            data_save.filament[0].color_R = 0xFF; // etc
        }
        
        Motion_control_save_struct *mc_ptr = (Motion_control_save_struct *)(Motion_control_save_flash_addr);
        if (mc_ptr->check == 0x40614061) {
            memcpy(&mc_save, mc_ptr, sizeof(mc_save));
        }
    }
    
    void Init() {
        Hardware::ADC_Init();
        Hardware::PWM_Init();
        LoadSettings();
        // Init AS5600?
        // Note: Motion_control.cpp line 3 MC_AS5600 instantiation.
        // It uses soft IIC.
    }
    
    void Run() {
        static uint64_t last_run = 0;
        uint64_t now = Hardware::GetTime();
        float time_E = (now - last_run) / 1000.0f;
        last_run = now;
        
        // Update ADC
        // Update AS5600
        MC_AS5600.updata_angle();
        
        // Run Motor Logic
        for(int i=0; i<4; i++) {
            motors[i].Run(time_E);
        }
        
        // Update LEDs
        // ... Logic from main.cpp Show_SYS_RGB ...
    }
    
    void UpdateConnectivity(bool online) {
        is_connected = online;
        if (online) last_heartbeat_time = Hardware::GetTime();
    }
    
    void ProcessMotionShort(uint8_t* buffer, uint16_t length) {
        // Logic from BambuBus.cpp send_for_motion_short
        // ...
        // Need to construct response Cxx_res
        // CommandRouter::SendPacket(response, len);
    }
    
    void ProcessMotionLong(uint8_t* buffer, uint16_t length) {
        // Logic for Dxx_res
    }

    void ProcessOnlineDetect(uint8_t* buffer, uint16_t length) {
        // Logic
    }
    
    void ProcessREQx6(uint8_t* buffer, uint16_t length) {
    }
    
    void ProcessSetFilamentInfo(uint8_t* buffer, uint16_t length) {
        // Logic
    }
    
    void ProcessHeartbeat(uint8_t* buffer, uint16_t length) {
        // Logic
    }
    
    void ProcessLongPacket(long_packge_data &data) {
        // Logic
        if (data.target_address == BambuBus_AMS) device_type_addr = BambuBus_AMS;
        else if (data.target_address == BambuBus_AMS_lite) device_type_addr = BambuBus_AMS_lite;
    }
    
    uint16_t GetDeviceType() {
        return device_type_addr;
    }

}
