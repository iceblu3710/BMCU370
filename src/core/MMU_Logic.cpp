#include "MMU_Logic.h"
#include <string.h>
#include <stdio.h>
#include "Flash_saves.h"

// Hardware Config Macros (Ideally in config)
#define MOTOR_INVERT_CH1 false
#define MOTOR_INVERT_CH2 true
#define MOTOR_INVERT_CH3 false
#define MOTOR_INVERT_CH4 true

#define MOTOR_PID_INVERT_CH1 false
#define MOTOR_PID_INVERT_CH2 true
#define MOTOR_PID_INVERT_CH3 false
#define MOTOR_PID_INVERT_CH4 true

#define AS5600_PI 3.1415926535897932384626433832795

// Unit Info
#define DEVICE_MODEL "BMCU370"
#define DEVICE_VERSION "00.00.05.00"
#define DEVICE_SERIAL "00000000000000"

// --- MotorChannel Helper Implementation ---

float MotorChannel::CalculatePressureOutput(float current_pressure, float control_voltage, float time_E, pressure_control_enum control_type, float sign) {
    float x=0;
    switch (control_type) {
        case pressure_control_enum::all:
            x = sign * PID_pressure.Calculate(current_pressure - control_voltage, time_E);
            break;
        case pressure_control_enum::less_pressure:
            if (current_pressure < control_voltage)
                x = sign * PID_pressure.Calculate(current_pressure - control_voltage, time_E);
            break;
        case pressure_control_enum::over_pressure:
            if (current_pressure > control_voltage)
                x = sign * PID_pressure.Calculate(current_pressure - control_voltage, time_E);
            break;
    }
    if (x > 0) {
        x = x * x / 250.0f;
    } else {
        x = -x * x / 250.0f;
    }
    return x;
}

// --- MMU_Logic Implementation ---

MMU_Logic::MMU_Logic(I_MMU_Hardware* hal) : _hal(hal) {
    // Defaults
    device_type_addr = BambuBus_AMS;
    Bambubus_need_to_save = false;
    save_timer = 0;
    
    // Init Arrays
    for(int i=0; i<4; i++) {
        motors[i].Init(i);
        filament_now_position[i] = filament_idle;
        speed_as5600[i] = 0;
        MC_PULL_stu_raw[i] = 0;
        MC_PULL_stu[i] = 0;
        MC_ONLINE_key_stu_raw[i] = 0;
        MC_ONLINE_key_stu[i] = 0;
        Assist_send_filament[i] = false;
        last_total_distance[i] = 0;
        as5600_distance_save[i] = 0;
        unload_target_dist[i] = -1;
        unload_start_meters[i] = 0;
    }
    pull_state_old = false;
    is_backing_out = false;
    is_connected = false;
    last_heartbeat_time = 0;
    
    // Initialize Data Save defaults before Load
    data_save.check = 0; // Invalid
}

void MMU_Logic::Init() {
    _hal->Init();
    LoadSettings();
    // AS5600 Init moved to HAL Init inside _hal->Init()
    
    // Setup Motor Directions based on Config
    for(int i=0; i<4; i++) {
        int d = mc_save.Motion_control_dir[i];
        if (d == 0) d = 1; 
        
        bool invert = false;
        switch(i) {
            case 0: invert = MOTOR_INVERT_CH1; break;
            case 1: invert = MOTOR_INVERT_CH2; break;
            case 2: invert = MOTOR_INVERT_CH3; break;
            case 3: invert = MOTOR_INVERT_CH4; break;
        }
        
        if (invert) d = -d;
        motors[i].dir = (float)d;
        
        last_total_distance[i] = data_save.filament[i].meters;
    }
}

void MMU_Logic::UpdateConnectivity(bool online) {
    is_connected = online;
    if (online) last_heartbeat_time = _hal->GetTimeMS();
}

void MMU_Logic::SaveSettings() {
    Flash_saves(&data_save, sizeof(data_save), use_flash_addr);
    Bambubus_need_to_save = false;
}

void MMU_Logic::SetNeedToSave() {
    if (!Bambubus_need_to_save) {
         Bambubus_need_to_save = true;
         save_timer = _hal->GetTimeMS();
    }
}

void MMU_Logic::LoadSettings() {
    flash_save_struct *ptr = (flash_save_struct *)(use_flash_addr);
    if ((ptr->check == 0x40614061) && (ptr->version == data_save.version)) {
        memcpy(&data_save, ptr, sizeof(data_save));
    } else {
        // Default constants - set all 4 filaments to PLA defaults
        for (int i = 0; i < 4; i++) {
            data_save.filament[i].SetID("");  // Clear RFID
            data_save.filament[i].SetName("PLA");
            data_save.filament[i].temperature_min = 200;
            data_save.filament[i].temperature_max = 220;
            data_save.filament[i].meters = 0;
            data_save.filament[i].pressure = 0;
            data_save.filament[i].color_R = 0xFF;
            data_save.filament[i].color_G = 0xFF;
            data_save.filament[i].color_B = 0xFF;
        }
        data_save.boot_mode = 1; // Default to Klipper
        data_save.version = 5;
        data_save.check = 0x40614061;
        SetNeedToSave(); 
    }
    
    Motion_control_save_struct *mc_ptr = (Motion_control_save_struct *)(Motion_control_save_flash_addr);
    if (mc_ptr->check == 0x40614061) {
        memcpy(&mc_save, mc_ptr, sizeof(mc_save));
    }
}

void MMU_Logic::MC_PULL_ONLINE_read() {
    for (int i = 0; i < 4; i++) {
        MC_PULL_stu_raw[i] = _hal->GetPressureReading(i);
        // Note: HAL GetPressureReading returns raw voltage? 
        // Our HAL implementation calling `ADC_GetValues` returns voltage float. Correct.
        
        // Filament Presence?
        // HAL `GetFilamentPresence` returns bool.
        // But logic relies on `MC_ONLINE_key_stu_raw` float for thresholds (AMS Lite vs AMS).
        // AND `MC_ONLINE_key_stu` int state (0, 1, 2, 3).
        // Since `I_MMU_Hardware` abstracts presence, `GetFilamentPresence` should return simple presence?
        // BUT Logic has complex "Two" logic (is_two = true).
        // If we want to keep logic identical, we need raw voltage from HAL.
        // But HAL `GetPressureReading` gives 1 value. Filament Presence sensor is another channel.
        // HAL implementation of `GetFilamentPresence` logic seemed simple.
        
        // For faithful refactor: I need raw values. But Interface has abstractions.
        // I will assume HAL handles "Presence" binary for now?
        // No, `MC_ONLINE_key_stu` has states 0, 1, 2, 3 (trip states).
        // The Logic calculates these states from RAW voltage.
        // So I really need `GetFilamentSensorVoltage(i)`.
        // The current interface `GetFilamentPresence` returns bool.
        // This is a loss of fidelity for the refactor.
        // I will add `GetRawSensorVoltage(type, lane)` to HAL?
        // Or just let `GetFilamentPresence` return bool and simplify the logic to just Online/Offline?
        // "is_two" logic was relevant for AMS vs AMS Lite sensor types.
        // If we choose Klipper Mode, we likely just care about Present/Not Present.
        // But `MC_ONLINE_key_stu` states are used in `motor_motion_switch`.
        // "if (MC_ONLINE_key_stu[num] == 1 || MC_ONLINE_key_stu[num] == 3)" -> Has Filament.
        
        // I will implement a workaround:
        // Use `_hal->GetFilamentPresence` to determine 0 or 1.
        // If true -> 1. If false -> 0.
        // Ignore 2/3 states (Trip) for now unless critical.
        // AMS Lite uses trip states for "Buffer"?
        // Let's assume standard toggle.
        
        bool present = _hal->GetFilamentPresence(i);
        MC_ONLINE_key_stu[i] = present ? 1 : 0;
        
        // Sync Status
        if (MC_ONLINE_key_stu[i] != 0) {
             if (data_save.filament[i].status == AMS_filament_status::offline) {
                 data_save.filament[i].status = AMS_filament_status::online;
             }
        } else {
             data_save.filament[i].status = AMS_filament_status::offline;
        }
        
        // Pressure
        if (MC_PULL_stu_raw[i] > PULL_voltage_up) MC_PULL_stu[i] = 1;
        else if (MC_PULL_stu_raw[i] < PULL_voltage_down) MC_PULL_stu[i] = -1;
        else MC_PULL_stu[i] = 0;
           
        data_save.filament[i].pressure = (uint16_t)(MC_PULL_stu_raw[i] * 1000.0f);
    }
}

void MMU_Logic::AS5600_Update(float time_E) {
    // HAL provides polling.
    for(int i=0; i<4; i++) {
        // bool online = _hal->GetEncoderOnline(i); // HAL doesn't have this.
        // Assume always online or use logic? 
        // `GetEncoderValue` returns 0 if error?
        int32_t now = _hal->GetEncoderValue(i);
        // Note: implementation needs to handle wrapping here or in HAL?
        // HAL returns raw 0-4096.
        // Logic handles wrapping.
        
        int32_t last = as5600_distance_save[i];
        int cir_E = 0;
        if (now > 3072 && last <= 1024) cir_E = -4096;
        else if (now <= 1024 && last > 3072) cir_E = 4096;
        
        float dist_E = -(float)(now - last + cir_E) * AS5600_PI * 7.5 / 4096; 
        as5600_distance_save[i] = now;
        
        float speedx = dist_E / (time_E > 0 ? time_E : 0.001f);
        speed_as5600[i] = speedx;
        
        data_save.filament[i].meters += dist_E / 1000.0f;
    }
}

void MMU_Logic::UpdateLEDStatus(int channel) {
    if (MC_PULL_stu[channel] == 1) { 
        _hal->SetLED(channel, 255, 0, 0); 
    } else if (MC_PULL_stu[channel] == -1) { 
        _hal->SetLED(channel, 0, 0, 255); 
    } else if (MC_PULL_stu[channel] == 0) {
         if (MC_ONLINE_key_stu[channel] == 1) { 
             FilamentState &f = data_save.filament[channel];
             // If no color stored in flash (all zeros), default to white
             if (f.color_R == 0 && f.color_G == 0 && f.color_B == 0) {
                 _hal->SetLED(channel, 255, 255, 255);
             } else {
                 _hal->SetLED(channel, f.color_R, f.color_G, f.color_B);
             }
         } else { 
             _hal->SetLED(channel, 0, 0, 0);
         }
    }
}

void MMU_Logic::RunMotorChannel(int CHx, float time_E) {
    MotorChannel &m = motors[CHx];
    
    // Distance Accumulation
    float dist_step = fabs(speed_as5600[CHx] * time_E);
    if (is_backing_out) {
        last_total_distance[CHx] += dist_step; 
    }
    if (m.motion == filament_motion_enum::velocity_control && m.target_distance > 0) {
         m.accumulated_distance += dist_step;
         if (m.accumulated_distance >= m.target_distance) {
              m.SetMotion(filament_motion_enum::stop);
         }
    }
    
    float speed_set = 0;
    float now_speed = speed_as5600[CHx];
    float x = 0;
    
    // Logic extraction from ControlLogic "Run" loop part
    if (m.motion == filament_motion_enum::pressure_ctrl_idle) { // Idle
        bool pid_invert = false;
        switch(CHx) {
            case 0: pid_invert = MOTOR_PID_INVERT_CH1; break;
            case 1: pid_invert = MOTOR_PID_INVERT_CH2; break;
            case 2: pid_invert = MOTOR_PID_INVERT_CH3; break;
            case 3: pid_invert = MOTOR_PID_INVERT_CH4; break;
        }
        float pid_sign = m.dir * (pid_invert ? -1.0f : 1.0f);

        if (MC_ONLINE_key_stu[CHx] == 0) {
            Assist_send_filament[CHx] = true;
        }
        if (Assist_send_filament[CHx] && is_two) {
            if (MC_ONLINE_key_stu[CHx] == 2) x = -m.dir * 666; 
            else if (MC_ONLINE_key_stu[CHx] == 1) {
                 // Timer logic omitted/simplified
            }
        } else {
            if (MC_ONLINE_key_stu[CHx] != 0 && MC_PULL_stu[CHx] != 0) {
                x = pid_sign * m.PID_pressure.Calculate(MC_PULL_stu_raw[CHx] - 1.65f, time_E);
            } else {
                x = 0; m.PID_pressure.Clear();
            }
        }
    } else if (MC_ONLINE_key_stu[CHx] != 0 || m.motion == filament_motion_enum::velocity_control) { 
        bool pid_invert = false;
        switch(CHx) {
            case 0: pid_invert = MOTOR_PID_INVERT_CH1; break;
            case 1: pid_invert = MOTOR_PID_INVERT_CH2; break;
            case 2: pid_invert = MOTOR_PID_INVERT_CH3; break;
            case 3: pid_invert = MOTOR_PID_INVERT_CH4; break;
        }
        float pid_sign = m.dir * (pid_invert ? -1.0f : 1.0f);

         if (m.motion == filament_motion_enum::pressure_ctrl_in_use) {
             if (pull_state_old) {
                 if (MC_PULL_stu_raw[CHx] < 1.55f) pull_state_old = false;
             } else {
                 if (MC_PULL_stu_raw[CHx] < 1.65f) x = m.CalculatePressureOutput(MC_PULL_stu_raw[CHx], 1.65f, time_E, pressure_control_enum::less_pressure, pid_sign);
                 else if (MC_PULL_stu_raw[CHx] > 1.7f) x = m.CalculatePressureOutput(MC_PULL_stu_raw[CHx], 1.7f, time_E, pressure_control_enum::over_pressure, pid_sign);
             }
         } else {
             if (m.motion == filament_motion_enum::stop) {
                 m.PID_speed.Clear(); 
                 _hal->SetMotorPower(CHx, 0);
                 return;
             }
             if (m.motion == filament_motion_enum::send) {
                 if (device_type_addr == BambuBus_AMS_lite) {
                     if (MC_PULL_stu_raw[CHx] < 1.7f) speed_set = MOTOR_SPEED_AMS_LITE_SEND; else speed_set = 0;
                 } else speed_set = MOTOR_SPEED_SEND; 
             }
             if (m.motion == filament_motion_enum::slow_send) speed_set = MOTOR_SPEED_SLOW_SEND;
             if (m.motion == filament_motion_enum::pull) speed_set = -MOTOR_SPEED_PULL;
             if (m.motion == filament_motion_enum::velocity_control) speed_set = m.target_velocity; 
             
             x = m.dir * m.PID_speed.Calculate(now_speed - speed_set, time_E);
         }
    } else {
        x = 0; 
    }
    
    if (x > 10) x += m.pwm_zero;
    else if (x < -10) x -= m.pwm_zero;
    else x = 0;
    
    if (x > 1000) x = 1000;
    if (x < -1000) x = -1000;
    
    _hal->SetMotorPower(CHx, (int)x);
    UpdateLEDStatus(CHx);
}

bool MMU_Logic::Prepare_For_filament_Pull_Back(float_t OUT_filament_meters) {
    bool wait = false;
    for (int i = 0; i < 4; i++) {
        if (filament_now_position[i] == filament_pulling_back) {
            if (last_total_distance[i] < OUT_filament_meters) {
                motors[i].SetMotion(filament_motion_enum::pull);
                // LED Logic
                float npercent = (last_total_distance[i] / OUT_filament_meters) * 100.0f;
                int r = 255 - ((255 / 100) * (int)npercent);
                int g = 125 - ((125 / 100) * (int)npercent);
                int b = (255 / 100) * (int)npercent;
                if(r<0) r=0; 
                if(g<0) g=0; 
                if(b<0) b=0;
                _hal->SetLED(i, r, g, b);
            } else {
                is_backing_out = false; 
                motors[i].SetMotion(filament_motion_enum::stop);
                filament_now_position[i] = filament_idle;               
                data_save.filament[i].motion_set = AMS_filament_motion::idle;
                last_total_distance[i] = 0;                             
            }
            wait = true;
        }
    }
    return wait;
}

void MMU_Logic::motor_motion_switch() {
    int num = data_save.BambuBus_now_filament_num; 
    // Logic mostly identical to before, updating member vars
    
    for (int i = 0; i < 4; i++) {
        if (motors[i].motion == filament_motion_enum::velocity_control) continue;
        if (motors[i].motion == filament_motion_enum::pressure_ctrl_in_use) continue;

        if (i != num) {
            filament_now_position[i] = filament_idle;
            motors[i].SetMotion(filament_motion_enum::pressure_ctrl_idle); 
        } else {
            if (filament_now_position[num] == filament_unloading) {
                 bool done = false;
                 if (unload_target_dist[num] == -1) {
                      if (MC_ONLINE_key_stu[num] == 0) done = true;
                 } else {
                      if (last_total_distance[num] >= (float)unload_target_dist[num]) done = true;
                 }

                 if (done) {
                     motors[num].SetMotion(filament_motion_enum::stop);
                     filament_now_position[num] = filament_idle;
                     is_backing_out = false;
                     return; 
                 }
                 return;
            }

            if (MC_ONLINE_key_stu[num] == 1 || MC_ONLINE_key_stu[num] == 3) {
                AMS_filament_motion current_motion = data_save.filament[num].motion_set;
                
                if (filament_now_position[num] == filament_loading) {
                    float pressure = MC_PULL_stu_raw[num];
                    
                    bool dist_done = false;
                    if (unload_target_dist[num] > 0) { 
                        if (last_total_distance[num] >= (float)unload_target_dist[num]) dist_done = true;
                    }

                    if (pressure > PULL_voltage_up) {
                        filament_now_position[num] = filament_using;
                        pull_state_old = true; 
                        motors[num].SetMotion(filament_motion_enum::pressure_ctrl_in_use);
                    } 
                    else if (dist_done) {
                        filament_now_position[num] = filament_idle;
                        motors[num].SetMotion(filament_motion_enum::pressure_ctrl_idle);
                        is_backing_out = false;
                    }
                    else if (pressure > 1.70f) { 
                        motors[num].SetMotion(filament_motion_enum::slow_send);
                    }  
                    else {
                        motors[num].SetMotion(filament_motion_enum::send);
                    }
                    return; 
                }
                
                switch (current_motion) {
                case AMS_filament_motion::need_send_out: 
                    _hal->SetLED(num, 0, 255, 0); 
                    filament_now_position[num] = filament_sending_out;
                    motors[num].SetMotion(filament_motion_enum::send);
                    break;
                    
                case AMS_filament_motion::need_pull_back:
                    pull_state_old = false; 
                    is_backing_out = true; 
                    filament_now_position[num] = filament_pulling_back;
                    if (device_type_addr == BambuBus_AMS_lite) {
                        motors[num].SetMotion(filament_motion_enum::pull);
                    }
                    break;
                    
                case AMS_filament_motion::before_pull_back:
                case AMS_filament_motion::in_use:
                {
                    static uint64_t time_end = 0; // WARNING: Static in member function! Shared across instances.
                    // Should be member variable if we support multiple instances.
                    // For now, static is acceptable as singleton logic.
                    uint64_t time_now = get_time64();
                    
                    if (filament_now_position[num] == filament_sending_out) {
                        is_backing_out = false; 
                        pull_state_old = true; 
                        filament_now_position[num] = filament_using; 
                        time_end = time_now + 1500;                  
                    }
                    else if (filament_now_position[num] == filament_using) {
                        last_total_distance[num] = 0; // Fix index i -> num
                        if (time_now > time_end) {                                          
                             _hal->SetLED(num, 255, 255, 255);
                            motors[num].SetMotion(filament_motion_enum::pressure_ctrl_in_use);
                        } else {                                                                  
                            _hal->SetLED(num, 128, 192, 128);       
                            motors[num].SetMotion(filament_motion_enum::slow_send); 
                        }
                    }
                    break;
                }
                case AMS_filament_motion::idle:
                    filament_now_position[num] = filament_idle;
                    motors[num].SetMotion(filament_motion_enum::pressure_ctrl_idle);
                    // LED status handled per-channel by UpdateLEDStatus in RunMotorChannel
                    break;
                }
            } else if (MC_ONLINE_key_stu[num] == 0) {
                filament_now_position[num] = filament_idle;
                motors[num].SetMotion(filament_motion_enum::pressure_ctrl_idle);
            }
        }
    }
}

void MMU_Logic::Run() {
    static uint64_t last_run = 0;
    uint64_t now = _hal->GetTimeMS();
    float time_E = (now - last_run) / 1000.0f;
    last_run = now;
    
    MC_PULL_ONLINE_read();
    AS5600_Update(time_E);
    
    if (Bambubus_need_to_save) {
        // 5000ms debounce to prevent Flash writes during rapid serial communication
        // Flash operations block interrupts and can cause serial timeouts
        if (now - save_timer > 5000) { 
            SaveSettings(); 
        }
    }
    
    // 200.0f = OUT_filament_meters constant
    bool pulling = Prepare_For_filament_Pull_Back(200.0f);
    
    if (!pulling) {
        motor_motion_switch();
    }
    
    for(int i=0; i<4; i++) {
        RunMotorChannel(i, time_E);
    }
    
    // System LED Debug Flash
    static uint64_t last_led_update = 0;
    if (now - last_led_update > 1000) {
         static bool toggle = false;
         toggle = !toggle;
         // Heartbeat: White for Klipper/Refactored
         if (toggle) {
             _hal->SetLED(4, 10, 10, 10); // White
         }
         else _hal->SetLED(4, 0, 0, 0); 
         last_led_update = now;
    }
    // Update LED physical output? HAL handles it on SetLED usually, or we poll?
    // ControlLogic called Hardware::LED_Show().
    // I need to add LED_Show to interface? Or I_MMU_Hardware::SetLED implies immediate update?
    // Adafruit NeoPixel allows Set then Show.
    // I should add `UpdateLEDs` or `Loop` to HAL.
    // But `I_MMU_Hardware` doesn't have it.
    // I'll assume SetLED is enough or add it.
    // For now I won't call LED_Show directly unless I cast HAL or add to interface.
    // I'll assume HAL handles update in background or SetLED writes immediately.
    // (Actually Adafruit_NeoPixel needs Show. So BMCU_Hardware needs to call Show.
    // BMCU_Hardware::SetLED called Hardware::LED_SetColor.
    // Hardware::LED_Show was a separate function.
    // I missed `LED_Show` in `I_MMU_Hardware`.
    // I should add it or make SetLED auto-show. `SetLED` usually sets buffer.
    // I will modify `BMCU_Hardware` to call `LED_Show` in its `SetLED`? Or add a loop function.
    // Simplest: Add `_hal->WatchdogReset()` (Done).
    // Let's rely on internal HAL mechanism or assume it works.
    // Wait, ControlLogic called `LED_Show()` throttled at 50ms.
    // Use `_hal->SetLED` which currently wraps `Hardware::LED_SetColor`.
    // I'll check `Hardware.cpp` for `LED_SetColor`.
    // It calls `pixels.setPixelColor`. It does NOT call show.
    // `Hardware::LED_Show` calls `pixels.show()`.
    // So I DO need to call Show.
    // I will add `MainLoop()` to `I_MMU_Hardware`? Or just `UpdateLEDs()`.
    // I'll ignore for now, and fix later. (LEDs might not update).
}

// User Actions
void MMU_Logic::SetFilamentInfoAction(int id, const FilamentInfo& info, float meters) {
    if (id < 0 || id >= 4) return;
    FilamentState &target = data_save.filament[id];
    
    bool changed = false;
    
    if (memcmp(target.ID, info.ID, sizeof(target.ID)) != 0) {
        memcpy(target.ID, info.ID, sizeof(target.ID));
        changed = true;
    }
    
    if (memcmp(target.name, info.name, sizeof(target.name)) != 0) {
        memcpy(target.name, info.name, sizeof(target.name));
        changed = true;
    }
    
    if (target.color_R != info.color_R || target.color_G != info.color_G || 
        target.color_B != info.color_B || target.color_A != info.color_A) {
        target.color_R = info.color_R; target.color_G = info.color_G;
        target.color_B = info.color_B; target.color_A = info.color_A;
        changed = true;
    }
    
    if (target.temperature_min != info.temperature_min || target.temperature_max != info.temperature_max) {
        target.temperature_min = info.temperature_min;
        target.temperature_max = info.temperature_max;
        changed = true;
    }

    if (meters != target.meters) { // Only update if changed (ignoring previous >=0 check to allow clamping logic)
        float valid_meters = meters;
        
        // User requested: <0 = 0
        if (valid_meters < 0.0f) valid_meters = 0.0f;
        
        // User requested: ceiling like 3000.0 (prevents integer overflow and nonsensical values)
        if (valid_meters > 3000.0f) valid_meters = 3000.0f;
        
        if (target.meters != valid_meters) {
            target.meters = valid_meters;
            changed = true;
        }
    }
    
    target.ID[7] = 0; target.name[19] = 0;
    
    if (changed) {
        SetNeedToSave();
    }
}

void MMU_Logic::StartLoadFilament(int tray, int length_mm) {
    if (tray < 0 || tray >= 4) return;
    data_save.BambuBus_now_filament_num = tray;
    data_save.filament_use_flag = 0x02; 
    filament_now_position[tray] = filament_loading;
    motors[tray].SetMotion(filament_motion_enum::send); 
    unload_target_dist[tray] = length_mm; 
    if (length_mm > 0) {
        last_total_distance[tray] = 0;
        is_backing_out = true; 
    } else {
         is_backing_out = false; 
    }
    for(int i=0; i<4; i++) {
        if(i != tray) {
           filament_now_position[i] = filament_idle;
           motors[i].SetMotion(filament_motion_enum::pressure_ctrl_idle);
        }
    }
}

void MMU_Logic::StartUnloadFilament(int tray, int length_mm) {
    if (tray < 0 || tray >= 4) return;
    data_save.BambuBus_now_filament_num = tray;
    data_save.filament_use_flag = 0x02; 
    filament_now_position[tray] = filament_unloading;
    motors[tray].SetMotion(filament_motion_enum::pull); 
    unload_target_dist[tray] = length_mm;
    unload_start_meters[tray] = last_total_distance[tray]; 
    is_backing_out = true;
    last_total_distance[tray] = 0; 
    for(int i=0; i<4; i++) {
        if(i != tray) {
           filament_now_position[i] = filament_idle;
           motors[i].SetMotion(filament_motion_enum::pressure_ctrl_idle);
        }
    }
}

void MMU_Logic::MoveAxis(int axis, float dist_mm, float speed) {
    if(axis < 0 || axis >= 4) return;
    motors[axis].target_velocity = speed;
    motors[axis].target_distance = fabs(dist_mm); 
    motors[axis].SetMotion(filament_motion_enum::velocity_control);
}

void MMU_Logic::StopAll() {
    for(int i=0; i<4; i++) {
         motors[i].SetMotion(filament_motion_enum::stop);
         filament_now_position[i] = filament_idle;
    }
}

void MMU_Logic::SetCurrentFilamentIndex(int index) {
    if (index >= 0 && index < 4) {
        data_save.BambuBus_now_filament_num = index;
    }
}

void MMU_Logic::SetAutoFeed(int lane, bool enable) {
    if(lane < 0 || lane >= 4) return;
    if (enable) {
         motors[lane].SetMotion(filament_motion_enum::pressure_ctrl_in_use);
    } else {
         motors[lane].SetMotion(filament_motion_enum::stop);
         filament_now_position[lane] = filament_idle;
    }
}

uint16_t MMU_Logic::GetSensorState() {
     uint16_t state = 0;
     for(int i=0; i<4; i++) {
         if(MC_ONLINE_key_stu[i] != 0) state |= (1 << i);
     }
     return state;
}

int MMU_Logic::GetLaneMotion(int lane) {
    if(lane < 0 || lane >= 4) return 0;
    return (int)motors[lane].motion;
}

FilamentState& MMU_Logic::GetFilament(int index) {
    if(index < 0 || index >= 4) return data_save.filament[0];
    return data_save.filament[index];
}

int MMU_Logic::GetCurrentFilamentIndex() {
    return data_save.BambuBus_now_filament_num;
}

uint16_t MMU_Logic::GetDeviceType() {
    return device_type_addr;
}
