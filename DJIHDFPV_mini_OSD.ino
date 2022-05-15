
/* DJI HD FPV simple OSD
 *  displays voltage on OSD and allows arming via a PWM channel
 *  
 *  v003  2022-04-07  enable auto-cell detection, cleanups
 *
 *  Softwareserial TX is digital pin 9 connected to ardupilot RX telemetry port(57600)
 *  R1 22K
 *  R2 4K7
*/

#define SERIAL_TYPE                                                 0       //0==SoftSerial(Arduino_Nano), 1==HardSerial(others)

#include <MSP.h>
#include "MSP_OSD.h"

const uint16_t osd_avg_cell_voltage_pos  = 2445;
const uint16_t osd_main_batt_voltage_pos = 2477;

msp_battery_state_t battery_state = {0};
msp_osd_config_t msp_osd_config = {0};
msp_analog_t analog = {0};
msp_status_BF_t status_BF = {0};


#if SERIAL_TYPE == 0
  #include <AltSoftSerial.h>
  HardwareSerial &mspSerial = Serial1;  //Serial for Nano, Serial1 for Pro Micro
#elif SERIAL_TYPE == 1
  HardwareSerial &mspSerial = Serial2;
#endif

MSP msp;

uint32_t previousMillis_MSP = 0;
const uint32_t next_interval_MSP = 100;

uint8_t vbat = 0;
uint8_t batteryCellCount = 0;
uint8_t legacyBatteryVoltage = 0;
uint8_t batteryState = 0;             // voltage color 0==white, 1==red
uint16_t batteryVoltage = 0;
uint32_t flightModeFlags = 0;


// analog voltage reading vars
#define VOLTAGE_NUM_SAMPLES 10   
int voltage_raw_sum = 0;                 // sum of samples taken
unsigned char voltage_samples = 0;       // current sample number
float voltage = 0.0;                     // calculated voltage

float voltage_divider_scale = 5.74;      // vbat voltage / voltage measured between R1 and R2 i.e. 11.30V / 1.968 = 5.7389537836



// PWM arming stuff
byte PWM_PIN = 10;                    // digital pin 10 on Pro Micro
int pwm_value;

void setup()
{
    mspSerial.begin(115200);
    msp.begin(mspSerial);

Serial.begin(9600);
Serial.println("helo");
    pinMode(PWM_PIN, INPUT);
    
    readvoltage();
    
}

void readvoltage()
{
    while (voltage_samples < VOLTAGE_NUM_SAMPLES) {
        voltage_raw_sum += analogRead(A1);
        voltage_samples++;
        delay(10);
    }
    
    voltage = ((float)voltage_raw_sum / (float)VOLTAGE_NUM_SAMPLES * 5.015) / 1024.0;     // at a 11.6V pack its 3.01 => 3.75 => this didnt work later after soldering
    voltage = voltage * voltage_divider_scale;                                         // 11.6V , measured 1.958 at the divider = 5,92
    
    voltage_samples = 0;
    voltage_raw_sum = 0;

    vbat = voltage*10;                                                // we had 11.8V but we need 118
}

void doArm(){
    flightModeFlags |= (1 << 0);
    
    status_BF.flightModeFlags = flightModeFlags;
    msp.send(MSP_STATUS, &status_BF, sizeof(status_BF));    
}

void loop()
{

    
    readvoltage();
    if(batteryCellCount == 0 && vbat > 0) set_battery_cells_number();

    //send MSP data
    uint32_t currentMillis_MSP = millis();
    if ((uint32_t)(currentMillis_MSP - previousMillis_MSP) >= next_interval_MSP) {

    if(flightModeFlags == 0){
      pwm_value = pulseIn(PWM_PIN, HIGH);
Serial.println(pwm_value);
      if (pwm_value > 1800) {
        doArm();
      }
    }

Serial.println(vbat);

        previousMillis_MSP = currentMillis_MSP;
        send_msp_to_airunit();
    }

}

void set_battery_cells_number()
{
      

     if(vbat < 43)batteryCellCount = 1;
     else if(vbat < 85)batteryCellCount = 2;
     else if(vbat < 127)batteryCellCount = 3;
     else if(vbat < 169)batteryCellCount = 4;
     else if(vbat < 211)batteryCellCount = 5;
     else if(vbat < 255)batteryCellCount = 6;

     Serial.print("setting cellcount: ");
     Serial.println(batteryCellCount);
}

void send_msp_to_airunit()
{
    //MSP_ANALOG
    analog.vbat = vbat;
    msp.send(MSP_ANALOG, &analog, sizeof(analog));


    //MSP_BATTERY_STATE
    battery_state.batteryVoltage =  vbat * 10;
    battery_state.batteryCellCount = batteryCellCount;
    battery_state.batteryState = batteryState;
    battery_state.legacyBatteryVoltage = vbat;

    msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));

    //MSP_OSD_CONFIG
    send_osd_config();
    
} __attribute__ ((packed));



void send_osd_config()
{
  
    msp_osd_config.units = 1;

    msp_osd_config.osd_item_count = 56;
    msp_osd_config.osd_stat_count = 24;
    msp_osd_config.osd_timer_count = 2;
    msp_osd_config.osd_warning_count = 16;              // 16
    msp_osd_config.osd_profile_count = 1;              // 1
    msp_osd_config.osdprofileindex = 1;                // 1
    msp_osd_config.overlay_radio_mode = 0;             //  0

    msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
    msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
    msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
    msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
    msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
    msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
    msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
    msp_osd_config.osd_flymode_pos = osd_flymode_pos;
    msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
    msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
    msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
    msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
    msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
    msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
    msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
    msp_osd_config.osd_altitude_pos = osd_altitude_pos;
    msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
    msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
    msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
    msp_osd_config.osd_power_pos = osd_power_pos;
    msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
    msp_osd_config.osd_warnings_pos = osd_warnings_pos;
    msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
    msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
    msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
    msp_osd_config.osd_debug_pos = osd_debug_pos;
    msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
    msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
    msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
    msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
    msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
    msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
    msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
    msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
    msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
    msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
    msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
    msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
    msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
    msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
    msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
    msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
    msp_osd_config.osd_g_force_pos = osd_g_force_pos;
    msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
    msp_osd_config.osd_log_status_pos = osd_log_status_pos;
    msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
    msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
    msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
    msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
    msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
    msp_osd_config.osd_display_name_pos = osd_display_name_pos;
    msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
    msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
    msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
    msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
    msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
    msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;

    msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}
