/*
  AES Controls, CPAP Ventilator Controller program
  Written : 20200328
  Updated : 20201020 Broke up Main file into modules (Built to Teensy)
  NHD 230128WG-BTFH-VZ Monochrome LCD Display (U8g2lib Drivers)
  Teensy 3.2 (www.pjrc.com/teensy)
*/
// Arduino Standard
#include <Arduino.h>
#include <EEPROM.h>
//#include <Wire.h>

// Additional Installed
#include <Bounce.h>
#include "ClickButton.h"
#include <Encoder.h>
#include <U8g2lib.h>
#include <FastPID.h>
#include <movingAvg.h>
#include <CircularBuffer.h>

// Function Libraries
#include "alarms.h"
#include "breathing.h"
#include "display.h"
#include "init.h"
#include "readSensors.h"
#include "update.h"


#define ENCODER_OPTIMIZE_INTERRUPTS
#define BUILD "2.1"


// Setup PID Variables for Object
float Kp = 155.0, Ki = 25.0, Kd = 0.0, Hz = 15000.0, peep = 5.0;  // Original
int output_bits = 12, inhale_detect = 750;
bool output_signed = false, inhaling = true, breathstarted = false, waitingforinhale = false;
int16_t pidout;
uint32_t delt_t = 0, count = 0;


// SETUP OBJECTS
U8G2_RA8835_320X240_F_8080 display(U8G2_R1, A7, A6, A3, 0, 1, 2, 3, 4, A8, 5, A9, 6);
Encoder encoder(11, 12);
Bounce btn_encoder = Bounce(10, 5);
Bounce btn_menu = Bounce(8, 5);
ClickButton btn_mode(7, LOW, CLICKBTN_PULLUP);
movingAvg avgPressure(100);
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
CircularBuffer<float, 226> pressure_graph;
CircularBuffer<float, 226> flow_graph;
CircularBuffer<float, 226> volume_graph;
CircularBuffer<float, 160> current_graph;


// Constants (Static. No Reset)
const float brth_inc = 0.5, brth_high = 50.0, brth_low = 1.0;
const float pres_inc = 1.0, pres_high = 35.0, pres_low = 5.0;
const float peep_inc = 1.0, peep_high = 20.0, peep_low = 1.0;
const float bmin_inc = 1.0, bmin_high = 40.0, bmin_low = 5.0;
const float ie_inc = 0.5, ie_high = 5.0, ie_low = 0.5;
const float alarm_inc = 25.0, currentdraw_limit = 2.0;
const float Kp_inc = 5.0, Ki_inc = 5.0, Kd_inc = 5.0;


// Mode select init
String s_mode = "O";
int menuArrow = 0, menuCount = 1, inhale = 0, exhale = 0;
int mode = 0, prog = 0,faulted = 0, avg_pressure = 0, flow_smooth = 0;
float tf_result, tf_inc, cmh2o, cmh2o_set, flow, inh_perct, exh_perct, curr_reading, prev_reading;
float iqval, currentdraw, rate_change, target_inhale_rate, target_exhale_rate;
float last_cmh2o = 0.0, brth_thresh = 15.0, last_currentdraw = 0.0;
float liters = 0.0, last_liters = 0.0, max_liters = 6.0; // VOLUME
long last_encoder_pos = 0, run_timer = 0, liter_timer = 0;
double lpm = 0.0, last_lpm = 0.0; // FLOW

// false == mask, true == et;
bool mask_et_select = false;
String mask_select = "MASK";
// false == Controlled, true == Spontaneous;
bool bool_mode_select = false;
String mode_select = "Controlled";

// Initialize Alarming
int alarm1 = 600, alarm2 = 600, alarm3 = 400;
int alarm4 = 400, alarm5 = 500, alarm6 = 500;
bool alarm1_trip = false, alarm2_trip = false;
bool alarm3_trip = false, alarm4_trip = false;
bool alarm5_trip = false, alarm6_trip = false;
String alarm1_message = "Pressure Sensor Reading High";
String alarm2_message = "Pressure Sensor Reading Low";
String alarm3_message = "Peak Pressure Not Reached";
String alarm4_message = "Pressure Sensor error";
String alarm5_message = "Motor Overcurrent Fault";
String alarm6_message = "Max Volume Exceeded";
unsigned long current_time = millis();
unsigned long alarm1_trip_time, alarm2_trip_time, alarm3_trip_time;
unsigned long alarm4_trip_time, alarm5_trip_time, alarm6_trip_time;


// Target values initialize
int target_pres = 20, target_bmin = 16, target_peep = 5;
float target_ie = 2.0, flow_graph_min = 0.0, flow_graph_max = 0.0;
float volume_graph_min = 0.0, volume_graph_max = 0.0;

//Encoder Tacking 
int btn_encoder_val, btn_menu_val, btn_mode_val;


void setup() {
  //comms_init();
  avgPressure.begin();
  io_init();
  display_init();
}
// "Main" Loop
void loop() {
  
  //Get Sensor Values
  readCurrent();
  readPressure();
  readFlow();
  readVolume();

  // Check for user input
  updateEncoder();
  // Press Encoder Button then Menu to get PID tuning.
  if (btn_menu_val == 0 && btn_encoder_val == 0) {prog  = 2;} 
  else if (btn_mode_val == -1) {toggle_mode();} 
  else if (btn_menu_val == 0) {toggle_prog();}  
  else if (btn_encoder_val == 0) {toggle_menu();}

  // Set display (Program Button Active)
  if (prog  == 0) {display_main();}
  if (prog  == 1) {display_setup();}
  if (prog  == 2) {display_pid_config();}
  //  if (prog  == 3) {display_alarm_config();}

  // Check mode and Update Rates
  breathMode();
  breathingData();

  // Fault detection check
  fault_detect();

} // void (MAIN) loop end
