/*
  AES Controls, PPAP Ventilator Upgrade program
  Written : 20200328
  Updated : 20201230(Built to Teensy)
  Update Notes :
    20201101 - Added Tidal Vol and current breathing rate to Main Screen
    20201029 - Removed new spontaneous mode and Interface from Setup Menu
    20201028 - Renamed Modes and added Logic for breathing monitoring
    20201108 - Added backup rate and changed UI
    20201222 - Finalizing PS Mode with backup rates
  Hardware:
    NHD 230128WG-BTFH-VZ Monochrome LCD Display (U8g2lib Drivers)
    Teensy 3.2 (www.pjrc.com/teensy)
*/

// Arduino Standard
#include <Arduino.h>
#include <EEPROM.h>
//#include <Wire.h>

// Additional Installed
#include <Bounce.h>
#include <ClickButton.h>
#include <Encoder.h>
#include <U8g2lib.h>
#include "aes_logo.h"
#include <FastPID.h>
#include <movingAvg.h>
#include <CircularBuffer.h>

// ? Project Version
#define BUILD "2.1"

#define ENCODER_OPTIMIZE_INTERRUPTS

// Alarm Setup
#define GRACE_PERIOD 20
#define LED 9
#define BUZZER_ON_LOW 13
#define BUZZER_MED 15
#define BUZZER_HIGH 16
// Motor Controls
#define TI_ENABLE 19 // Cuts power to the motor
#define MOTOR_MIN 150 // 50 for Delta motor, 150 for Phillips motor

//Pre-defined Calucation Variables
#define Tube_ID 0.02            // in m
#define Air_Viscosity 0.0001825 // Pa*s @ 20C)
#define Tube_len 1.5            // in m
#define PA_to_MMHG 0.00750062   // Possibly unecessary

// LCD/u8g2 text alignment helper functions
#define LCDWidth 240 //display. getDisplayWidth()
#define ALIGN_CENTER(t) ((LCDWidth - (display.getUTF8Width(t))) / 2)
#define ALIGN_RIGHT(t) (LCDWidth - display.getUTF8Width(t))

// sgn function for use later (GCC Compiler Fault)
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// Setup PID Variables for Object
float Kp = 155.0, Ki = 25.0, Kd = 0.0, Hz = 15000.0, peep = 5.0; // Original
int output_bits = 12, inhale_detect = 750;
bool output_signed = false, inhaling = true, exhaling = false; 
bool breathstarted = false, exhalestarted = false;
int16_t pidout;
uint32_t delt_t = 0, count = 0;

// SETUP OBJECTS
U8G2_RA8835_320X240_F_8080 display(U8G2_R1, A7, A6, A3, 0, 1, 2, 3, 4, A8, 5, A9, 6);
Encoder encoder(11, 12);
Bounce btn_encoder = Bounce(10, 5);
Bounce btn_menu = Bounce(8, 5);
ClickButton btn_mode(7, LOW, CLICKBTN_PULLUP);
movingAvg avgTidalVolume(100);
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
CircularBuffer<float, 226> pressure_graph;
CircularBuffer<float, 226> flow_graph;
CircularBuffer<float, 226> volume_graph;
CircularBuffer<float, 160> current_graph;

// Constants (Static. No Reset)
const float brth_inc = 1.0, brth_high = 50.0, brth_low = 1.0;
const float pres_inc = 1.0, pres_high = 35.0, pres_low = 5.0;
const float peep_inc = 1.0, peep_high = 20.0, peep_low = 1.0;
const float bmin_inc = 1.0, bmin_high = 30.0, bmin_low = 5.0;
const long BackupRate_inc = 1000, BackupRate_high = 60000, BackupRate_low = 1000;
const float ie_inc = 0.5, ie_high = 5.0, ie_low = 0.5;
const float alarm_inc = 25.0, currentdraw_limit = 2.0;
const float Kp_inc = 5.0, Ki_inc = 5.0, Kd_inc = 5.0;

// Mode select init
String s_mode = "O";
int menuArrow = 0, menuCount = 1, inhale = 0, exhale = 0;
int mode = 0, prog = 0, faulted = 0, avg_pressure = 0, flow_smooth = 0;
float tf_result, tf_inc, cmh2o, cmh2o_set, flow, inh_perct, exh_perct;
float iqval, currentdraw;
float last_cmh2o = 0.0, brth_thresh = 15.0, last_currentdraw = 0.0;
float liters = 0.0, last_liters = 0.0, max_liters = 6.0, tidalVolume = 0.0, t_liters = 0.0;
long last_encoder_pos = 0, run_timer = 0, liter_timer = 0;
double lpm = 0.0, last_lpm = 0.0; // FLOW

// false == mask, true == et;
bool mask_et_select = false;
String mask_select = "MASK";
// false == Press Support, true == Press Assist;
bool bool_mode_select = false;
String mode_select = "Press Support";

// Initialize Alarming
bool alarm1_trip = false, alarm2_trip = false;
bool alarm3_trip = false, alarm4_trip = false;
bool alarm5_trip = false, alarm6_trip = false;
bool backUpTrip = false, belowSettings = false;
String alarm1_message = "Pressure Sensor Reading High";
String alarm2_message = "Pressure Sensor Reading Low";
String alarm3_message = "Peak Pressure Not Reached";
String alarm4_message = "Pressure Sensor error";
String alarm5_message = "Motor Overcurrent Fault";
String alarm6_message = "Circuit Disconnected";
String backup1_message = "Below Rate Timeout";
String backup2_message = "No Inhale Timeout";
unsigned long alarm1 = 500, alarm2 = 500, alarm3 = 500;
unsigned long alarm4 = 500, alarm5 = 500, alarm6 = 1000;
unsigned long current_time = 0, backUpTripTime, belowSettingsTime;
unsigned long alarm1_trip_time, alarm2_trip_time, alarm3_trip_time;
unsigned long alarm4_trip_time, alarm5_trip_time, alarm6_trip_time;
unsigned long lastBreathTime, sinceLastBreath, belowSetTime;
unsigned long backUpTime = 60000, belowRateTime = 30000; // in milliseconds

// Current Breathing Rate Variables
float current_hi_pres = 0.0, current_low_pres = 0.0, current_bmin;
float current_ie, curr_cmh2o, curr_period, curr_reading, prev_reading;
float curr_inhale, curr_exhale, curr_inh_perct, curr_exh_perct;
bool inhexh_start = false;
uint32_t inhexh_start_time = 0;

// Target values initialize
int target_pres = 20, target_bmin = 12, target_peep = 5;
float target_ie = 2.0, flow_graph_min = 0.0, flow_graph_max = 0.0;
float volume_graph_min = 0.0, volume_graph_max = 0.0;

//Encoder Tacking
int btn_encoder_val, btn_menu_val, btn_mode_val;

// Prototype Delaration
void buzzer_on();
void buzzer_off();
//void comms_init();
void display_backup();
void display_init();
void display_fault();
void display_alarm_config();
void display_main();
void display_mode();
void display_pid_config();
void display_setup();
void fault_detect();
void io_init();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
void recievEvent();
void toggle_menu();
void toggle_mode();
void toggle_prog();
void update_pid();
void update_target();
void volume_update();

void setup()
{
  //comms_init();
  avgTidalVolume.begin();
  io_init();
  display_init();
}

// "Main" Loop
void loop()
{

  //Current Draw calc
  float iqval = analogRead(A10) * (3.3 / 4095.0);
  currentdraw = mapfloat(iqval, 0.0, 3.3, 0.0, 5.78);
  current_graph.push(last_currentdraw);
  current_graph.push((last_currentdraw + currentdraw) / 2);
  current_graph.push((last_currentdraw + currentdraw + currentdraw) / 3);
  current_graph.push(currentdraw);
  last_currentdraw = currentdraw;

  //Pressure (cmH20) calc
  iqval = analogRead(A0) * (5.0 / 4095.0);
  cmh2o = 35.56 * (iqval - 0.25) / 3.75;
  pressure_graph.push(last_cmh2o);
  pressure_graph.push((last_cmh2o + cmh2o) / 2);
  pressure_graph.push(cmh2o);
  last_cmh2o = cmh2o;

  //Flow LPM calc
  iqval = analogRead(A11) * (5.0 / 4095.0);
  float diff_pa = sgn((iqval / 5.0) - 0.5) * sq((iqval / (5.0 * 0.4)) - 1.25) * 525.0;
  // Poiseuille's Law (pressure difference and volume flow)
  lpm = ((PI * pow(Tube_ID, 4) * diff_pa) / (8 * Air_Viscosity * Tube_len)) * 60000; // Convert from M^3/s to L/Min
  flow_graph.push(last_lpm);
  flow_graph.push((last_lpm + lpm) / 2);
  flow_graph.push(lpm);
  last_lpm = lpm;

  //TOTAL VOLUME CALCULATION
  //Stop Volume from continuing to grow in off mode
  int ti_liters = 0;

  if (mode == 0)
  {
    liters = 0.01;
    tidalVolume = 0.0;
    t_liters = 0.0;
  }
  else if (mode > 0)
  {
    liters += (lpm * ((millis() - liter_timer) / 60000.0));
    if (lpm < 0)
    {
      t_liters += abs((lpm * ((millis() - liter_timer) / 60000.0)));
      ti_liters = int(t_liters * 1000);  //Step up to save decimal places. (int truncates)
      avgTidalVolume.reading(ti_liters); // stored in mL
    }
  }

  //Update Volme Graphing
  volume_graph.push(last_liters);
  volume_graph.push((last_liters + liters) / 2);
  volume_graph.push(liters);
  last_liters = liters;
  liter_timer = millis();

  /*  ********************** ATTEMPT TO MONITOR PATIENT RATES ********************* */
  // ! Monitor paitient rates
  if (mode > 0)
  {
    //______Get Current Pressure_____
    curr_reading = (prev_reading * 0.9) + (cmh2o * 0.1);
    // _____Get High and Low pressures_____
    if (current_low_pres == 0)
    {
      current_low_pres = cmh2o;
    }
    if (curr_cmh2o < current_low_pres)
    {
      current_low_pres = cmh2o;
    }
    if (curr_cmh2o > current_hi_pres)
    {
      current_hi_pres = cmh2o;
    }
    float rate_of_change = curr_reading - prev_reading;
    if (!inhexh_start && rate_of_change > 0.0)
    {
      inhexh_start = true;
      curr_exhale = (millis() - inhexh_start_time);
      inhexh_start_time = millis();
      current_hi_pres = 0.0;
    }
    else if (inhexh_start && rate_of_change <= 0.0)
    {
      inhexh_start = false;
      curr_inhale = (millis() - inhexh_start_time);
      inhexh_start_time = millis();
      current_low_pres = 0.0;
    }
    prev_reading = curr_reading;
    curr_period = (curr_inhale + curr_exhale); // 3750 @ 16 bmin
    current_bmin = (60000 / curr_period);      // 16 bmin == 3750ms total_period
    if (current_bmin > 200)
    {
      current_bmin = 200;
    }

    // if they havent breathed in 5 seconds, reset RR
    if (sinceLastBreath > 5000)
    {
      //inhexh_start_time = 0;
      current_bmin = 0;
    }
  }
  else
  {
    current_bmin = 0;
  } // Dont monitor OFF Mode

  // ! Detect Patient Breath
  if (!inhaling && lpm > brth_thresh)
  {
    breathstarted = true;
    lastBreathTime = millis();
  } else if (inhaling && lpm < -20.0)
  {
    exhalestarted = true;
  }


  // ! Backup /Apneic Rate 
  if (mode == 1)
  {

    // TODO: CHECK TO SEE IF PATIENT IS BLEOW SETTINGS *********************************
    if ((current_bmin < target_bmin) && (!belowSettings))
    {
      belowSettings = true;
      belowSettingsTime = millis();
    }
    else if ((current_bmin >= target_bmin))
    {
      belowSettings = false;
      belowSettingsTime = millis();
    }

    // * Calculate Times
    if (!breathstarted)
    {
      sinceLastBreath = millis() - lastBreathTime;
      belowSetTime = millis() - belowSettingsTime;
    }
    else if (breathstarted)
    {
      lastBreathTime = millis();
      belowSettingsTime = millis();
      backUpTrip = false;
    }

    // TODO: ATTEMPT TO TRIGGER BACKUP RATE *********************************
    if ( (belowSetTime > belowRateTime) && (run_timer > GRACE_PERIOD) )
    {
      backUpTrip = true;
    }
    else if ( (belowSetTime < belowRateTime) || (run_timer <= GRACE_PERIOD) )
    {
      backUpTrip = false;
    }
  }
  else if (mode != 1)
  {
    sinceLastBreath = 0;
    belowSetTime = 0;
    lastBreathTime = millis();
    belowSettingsTime = millis();
    backUpTrip = false;
  }

  // ! **** SYSTEM VERIFICATION (UNCOMMENT TO VIEW ON SERIAL MONITOR)******
//  Serial.print(" inhaling: ");
//  Serial.print(inhaling);
//  Serial.print(" exhaling: ");
//  Serial.print(exhaling);
//  Serial.print(" breathstarted: ");
//  Serial.print(breathstarted);
//  Serial.print(" exhalestarted: ");
//  Serial.print(exhalestarted);
//  Serial.print(" belowSetTime: ");
//  Serial.print(belowSetTime);
//  Serial.print(" sinceLastBreath: ");
//  Serial.print(sinceLastBreath);
//  Serial.print(" backUpTrip: ");
//  Serial.print(backUpTrip);
//  Serial.print(" belowSettings: ");
//  Serial.print(belowSettings);
//  Serial.print(" current_bmin: ");
//  Serial.print(current_bmin);
//  Serial.println("");

  /* 
    ! Beginning of PID Control 

    ? PRESSURE SUPPORT MODE
  */
  delt_t = millis() - count;
  if (mode == 1)
  {
    // Breathing Control
    if (inhaling)
    {
      if (delt_t >= inhale || (exhalestarted))
      { // ! START EXHALE PERIOD
        breathstarted = false; //end triggered inhale
        cmh2o_set = target_peep;
        if (lpm <= brth_thresh)
        {
          inhaling = false;
          exhaling = true;
        }
        count = millis();
      }
    }
    else
    {
      if ((delt_t >= exhale && backUpTrip) || (breathstarted))
      { // ! START ASSISTED INHALE PERIOD
        exhalestarted = false; // end triggered exhale
        volume_update();
        cmh2o_set = target_pres;
        inhaling = true;
        exhaling = false;
        count = millis();
      }
    }
  }

  // ? PRESSURE ASSIST MODE
  else if (mode == 2)
  {
    if (inhaling)
    {
      if (delt_t >= inhale)
      {
        breathstarted = false;
        cmh2o_set = target_peep;
        if (lpm <= brth_thresh)
        {
          inhaling = false;
          exhaling = true;
        }
        count = millis();
      }
    }
    else
    {
      if (delt_t >= exhale || breathstarted)
      {
        volume_update();
        cmh2o_set = target_pres;
        inhaling = true;
        exhaling = false;
        count = millis();
      }
    }
  }

  pidout = myPID.step(cmh2o_set, cmh2o);

  if (mode > 0 && !faulted)
  {
    run_timer++;
    analogWrite(A14, pidout);
  }
  else if (mode == 0)
  {
    run_timer = 0;
    analogWrite(A14, MOTOR_MIN);
    myPID.clear();
    delt_t = 0;
  }

  //Cut Enable to motor if faulted
  if (faulted)
  {
    digitalWrite(TI_ENABLE, LOW);
  }
  else
  {
    digitalWrite(TI_ENABLE, HIGH);
  }

  /* Update breathing rate is 1:2.0 -> % = parts/whole ->  1/3 == .333, 2/3 == .666
     inhale 1250 @ 16bmin 1:2.0ie, exhale 2500 (inhale * 2.0)                        */
  int cycle_time = int(60000 / target_bmin); // 3750ms @ 16 b/min 1.20 ie
  inh_perct = 1 / (1 + target_ie);           // 0.29333 (~30%)
  exh_perct = target_ie / (1 + target_ie);   // 0.704 (~70%)
  inhale = int(cycle_time * inh_perct);      // 1250ms @ 16 b/min 1.20 ie
  exhale = int(cycle_time * exh_perct);      // 2500ms @ 16 b/min 1.20 ie

  // Fault detection check
  fault_detect();

  // Set display (Program Button Active)
  if (prog == 0)
  {
    display_main();
  }
  if (prog == 1)
  {
    display_setup();
  }
  if (prog == 2)
  {
    display_pid_config();
  }
  //  if (prog  == 3) {
  //    display_alarm_config();
  //  }

  long new_encoder_pos = encoder.read();
  if (new_encoder_pos >= last_encoder_pos + 4)
  {
    update_values(1);
    last_encoder_pos = new_encoder_pos;
  }
  else if (new_encoder_pos <= last_encoder_pos - 4)
  {
    update_values(-1);
    last_encoder_pos = new_encoder_pos;
  }

  btn_encoder.update();
  btn_menu.update();
  btn_mode.Update();

  int btn_encoder_val = btn_encoder.read();
  int btn_menu_val = btn_menu.read();
  int btn_mode_val = btn_mode.clicks;

  // Press Encoder Button then Menu to get PID tuning.
  if (btn_menu_val == 0 && btn_encoder_val == 0)
  {
    prog = 2;
  }
  else if (btn_mode_val == -1)
  {
    toggle_mode();
  }
  else if (btn_menu_val == 0)
  {
    toggle_prog();
  }
  else if (btn_encoder_val == 0)
  {
    toggle_menu();
  }

} // void (MAIN) loop end

void buzzer_off()
{
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER_ON_LOW, LOW);
  digitalWrite(BUZZER_MED, LOW);
  digitalWrite(BUZZER_HIGH, LOW);
}

void buzzer_on()
{
  digitalWrite(LED, HIGH);
  // MUST USE ON_LOW for ALL LEVELS
  digitalWrite(BUZZER_ON_LOW, HIGH);
  delay(10);
  digitalWrite(BUZZER_HIGH, HIGH);
}

//void comms_init() {
//  // Join I2C bus as slave with address 8
//  Wire.begin(0x8);
//  // Call receiveEvent when data received
//  Wire.onReceive(receiveEvent);
//}

void display_backup()
{
  String s_backup = "Backup Mode: ";

  if (backUpTrip && belowSettings)
  {
    s_backup += backup1_message;
  }
  if (backUpTrip && !belowSettings)
  {
    s_backup += backup2_message;
  }

  // Setup inverse
  display.setFontMode(1);                /* activate transparent font mode */
  display.setDrawColor(1);               /* color 1 for the box */
  display.drawBox(0, 30, 240, 25);       /* (X of upper left edge, Y of upper left edge, Width, Height) */
  display.setFontMode(0);                /* activate transparent font mode */
  display.setDrawColor(0);               /* color 1 for the box */
  display.setFont(u8g2_font_helvB08_tf); //Alarm Font
  display.setCursor(2, 45);
  display.print(s_backup);
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
}

void display_init()
{
  display.begin();
  display.enableUTF8Print();

  // Draw Splash Screen (AES Logo)
  display.clearBuffer();
  display.drawXBMP(0, 0, logo_width, logo_height, logo);
  display.sendBuffer();
  delay(2000);
  display.clearBuffer();
  display.drawXBMP(0, 0, logo_width, logo_height, splash);
  display.sendBuffer();
  delay(2000);
  buzzer_off();
}

void display_fault()
{
  String s_fault = "FAULT:  ";
  switch (faulted)
  {
  case 1:
    s_fault += alarm1_message;
    break;
  case 2:
    s_fault += alarm2_message;
    break;
  case 3:
    s_fault += alarm3_message;
    break;
  case 4:
    s_fault += alarm4_message;
    break;
  case 5:
    s_fault += alarm5_message;
    break;
  case 6:
    s_fault += alarm6_message;
    break;
  default:
    s_fault += "UNDEFINED FAULT OCCURED";
    break;
  }
  // Setup inverse
  display.setFontMode(1);                /* activate transparent font mode */
  display.setDrawColor(1);               /* color 1 for the box */
  display.drawBox(0, 30, 240, 25);       /* (X of upper left edge, Y of upper left edge, Width, Height) */
  display.setFontMode(0);                /* activate transparent font mode */
  display.setDrawColor(0);               /* color 1 for the box */
  display.setFont(u8g2_font_helvB08_tf); //Alarm Font
  display.setCursor(2, 45);
  display.print(s_fault);
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
}

void display_alarm_config()
{
  menuArrow = (menuCount * 16) + 50;
  display.clearBuffer();                   // clear the internal memory
  display.setFont(u8g2_font_profont17_tf); // choose a suitable font
  display.setCursor(ALIGN_CENTER(string2char("Alarm Settings")), 16);
  display.print("Alarm Settings");
  // Row 30 - 65 are off limits for Alarms/ Backup Rate
  if (backUpTrip)
  {
    display_backup();
  }
  if (faulted > 0)
  {
    display_fault();
  }
  // Menu Creation Row 1 - 5
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(10, 68);
  display.print("Alarm 1 Delay");
  display.setCursor(110, 68);
  display.print(":  " + String(alarm1));
  display.setCursor(180, 68);
  display.print("(ms)");
  display.setCursor(10, 84);
  display.print("Alarm 2 Delay");
  display.setCursor(110, 84);
  display.print(":  " + String(alarm2));
  display.setCursor(180, 84);
  display.print("(ms)");
  display.setCursor(10, 100);
  display.print("Alarm 3 Delay");
  display.setCursor(110, 100);
  display.print(":  " + String(alarm3));
  display.setCursor(180, 100);
  display.print("(ms)");
  display.setCursor(10, 116);
  display.print("Alarm 4 Delay");
  display.setCursor(110, 116);
  display.print(":  " + String(alarm4));
  display.setCursor(180, 116);
  display.print("(ms)");
  // Pointer Creation logic
  display.setCursor(2, menuArrow);
  display.print(">");
  display.sendBuffer(); // transfer internal memory to the display
}

void display_mode()
{
  if (s_mode == "O")
  {
    display.setCursor(1, 15);
    display.setFont(u8g2_font_helvB10_tr);
    display.print("OFF");
  }
  else if (s_mode == "AC")
  {
    display.setCursor(1, 23);
    display.setFont(u8g2_font_helvB18_tf);
    display.print("AC");
  }
  else if (s_mode == "PS")
  {
    display.setCursor(1, 23);
    display.setFont(u8g2_font_helvB18_tf);
    display.print("PS");
  }
}

void display_pid_config()
{
  menuArrow = (menuCount * 16) + 50;
  display.clearBuffer();                   // clear the internal memory
  display.setFont(u8g2_font_profont17_tf); // choose a suitable font
  display.setCursor(ALIGN_CENTER(string2char("PID Tuning")), 16);
  display.print("PID Tuning");

  // Row 30 - 65 are off limits for Alarms/ Backup Rate
  if (backUpTrip)
  {
    display_backup();
  }
  if (faulted > 0)
  {
    display_fault();
  }

  // Menu Creation Row 1 - 5
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(10, 68);
  display.print("Proportional (Kp) ");
  display.setCursor(140, 68);
  display.print(":  " + String(Kp));
  display.setCursor(10, 84);
  display.print("Integral (Ki) ");
  display.setCursor(140, 84);
  display.print(":  " + String(Ki));
  display.setCursor(10, 100);
  display.print("Derviative (Kd) ");
  display.setCursor(140, 100);
  display.print(":  " + String(Kd));
  // Pointer Creation logic
  display.setCursor(2, menuArrow);
  display.print(">");
  //  Serial.println(sizeof(current_graph));
  for (int x = 0; x < current_graph.size() - 1; x++)
  {
    display.drawLine(x, mapfloat(current_graph[x], 0.0, 5.78, 128, 80), x + 1, mapfloat(current_graph[x + 1], 0.0, 5.78, 128, 80));
  }

  //  display.print(":  " + String(int(cmh2o)));
  display.setFont(u8g2_font_8x13B_tr);
  display.setCursor(ALIGN_RIGHT(string2char(String(currentdraw, 3))) - 36, 126);
  //  Serial.println(string2char(cmh2o));
  display.print(String(currentdraw, 3));
  display.setCursor(208, 126);
  display.print("Amps");

  display.sendBuffer(); // transfer internal memory to the display
}

void display_setup()
{
  menuArrow = (menuCount * 16) + 47;
  display.clearBuffer();                   // clear the internal memory
  display.setFont(u8g2_font_profont17_tf); // choose a suitable font
  display.setCursor(ALIGN_CENTER(string2char("Ventilation Settings")), 16);
  display.print("  Ventilation Settings");

  // Mode Top Left
  display_mode();

  // Row 30 - 65 are off limits for Alarms/ Backup Rate
  if (backUpTrip)
  {
    display_backup();
  }
  if (faulted > 0)
  {
    display_fault();
  }

  // Menu Creation Row 1 - 5
  display.setFont(u8g2_font_helvB10_tr);
  //Row 1
  display.setCursor(10, 65);
  display.print("Insp. Pressure");
  display.setCursor(115, 65);
  display.print(":  " + String(target_pres));
  display.setCursor(180, 65);
  display.print("cmH2O");
  //Row 2
  display.setCursor(10, 81);
  display.print("PEEP");
  display.setCursor(115, 81);
  display.print(":  " + String(int(target_peep)));
  display.setCursor(180, 84);
  display.print("cmH2O");
  //Row 3
  display.setCursor(10, 97);
  display.print("Respiration Rt");
  display.setCursor(115, 97);
  display.print(":  " + String(target_bmin));
  display.setCursor(180, 100);
  display.print("b/min");
  //Row 4
  display.setCursor(10, 113);
  display.print("I:E");
  display.setCursor(115, 113);
  display.print(":  1: " + String(target_ie, 1));
  //Row 5
  //int t_brth_thresh = int(brth_thresh);
  display.setCursor(10, 128);
  display.print("Flow Trigger");
  display.setCursor(115, 128);
  display.print(":  " + String(int(brth_thresh)));
  display.setCursor(180, 128);
  display.print("L/Min");

  //Row 6
  int t_belowRateTime = belowRateTime / 1000;
  display.setCursor(10, 143);
  display.print("Below RR Trig");
  display.setCursor(115, 143);
  display.print(":  " + String(t_belowRateTime));
  display.setCursor(180, 143);
  display.print("sec");

  //Row 7
  display.setCursor(10, 159);
  display.print("Mode Select");
  display.setCursor(115, 159);
  display.print(":  " + mode_select);


  /********** Selection Display removed 20201029 */
  //  //Row 8
  //  display.setCursor(10, 174);
  //  display.print("Interface");
  //  display.setCursor(115, 174);
  //  display.print(":  " + mask_select);
  /********** Selection Display removed 20201029 */

  // NO ROW, JUST and Indicator
  display.setCursor(10, 250);
  display.print("Insp. Time");
  display.setCursor(115, 250);
  display.print(":  " + String(inhale));
  display.setCursor(180, 250);
  display.print("ms");

  // Pointer Creation logic
  display.setCursor(2, menuArrow);
  display.print(">");
  display.sendBuffer(); // transfer internal memory to the display
}

void display_main()
{
  // Main Screen Recreate (U8G2)
  display.clearBuffer(); // clear the internal memory

  // Mode Top Left
  display_mode();

  display.setCursor(41, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("P");
  display.setCursor(48, 13);
  display.setFont(u8g2_font_6x13B_tr);
  display.print("IP");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(ALIGN_CENTER(string2char(target_pres)) - 68, 27);
  display.print(String(target_pres));

  display.setCursor(78, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("PEEP");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(ALIGN_CENTER(string2char(target_peep)) - 26, 27);
  display.print(target_peep);

  display.setCursor(126, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("RR");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(ALIGN_CENTER(string2char(int(current_bmin))) + 15, 27);
  display.print(int(current_bmin));

  display.setCursor(163, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("V");
  display.setCursor(170, 12);
  display.setFont(u8g2_font_6x13B_tr);
  display.print("TE");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(ALIGN_CENTER(string2char(tidalVolume)) + 56, 27);
  display.print(String(tidalVolume, 2));

  display.setCursor(207, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("I:E");
  display.setCursor(201, 27);
  display.setFont(u8g2_font_helvB10_tr);
  display.print("1: " + String(target_ie, 1));

  display.setCursor(207, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("I:E");
  display.setCursor(201, 27);
  display.setFont(u8g2_font_helvB10_tr);
  display.print("1: " + String(target_ie, 1));

  display.drawLine(13, 114, 14, 114);
  display.setCursor(4, 107);
  display.setFont(u8g2_font_6x12_tr);
  display.print("5");
  display.drawLine(13, 103, 14, 103);

  display.drawLine(13, 91, 14, 91);
  display.setCursor(0, 84);
  display.setFont(u8g2_font_6x12_tr);
  display.print("15");
  display.drawLine(13, 80, 14, 80);

  display.drawLine(13, 68, 14, 68);
  display.setCursor(0, 61);
  display.setFont(u8g2_font_6x12_tr);
  display.print("25");
  display.drawLine(13, 57, 14, 57);

  display.drawLine(13, 45, 14, 45);
  display.setCursor(0, 38);
  display.setFont(u8g2_font_6x12_tr);
  display.print("35");
  display.drawLine(13, 34, 14, 34);

  display.setCursor(236, 125);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t");
  display.drawLine(239, 114, 239, 115);

  display.setCursor(197, 125);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-1");
  display.drawLine(203, 114, 203, 115);

  display.setCursor(161, 125);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-2");
  display.drawLine(167, 114, 167, 115);

  display.setCursor(124, 125);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-3");
  display.drawLine(130, 114, 130, 115);

  display.setCursor(87, 125);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-4");
  display.drawLine(93, 114, 93, 115);

  display.setCursor(51, 125);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-5");
  display.drawLine(57, 114, 57, 115);
  display.drawLine(20, 114, 20, 115);
  display.drawLine(14, 114, 14, 115);

  display.setFont(u8g2_font_6x12_tr);
  display.setCursor(ALIGN_RIGHT(string2char(String(cmh2o, 0))) - 229, 128);
  display.print(String(cmh2o, 0));
  display.setCursor(13, 128);
  display.print("cmH");
  display.setFont(u8g2_font_5x7_tr);
  display.setCursor(31, 128);
  display.print("2");
  display.setFont(u8g2_font_6x12_tr);
  display.setCursor(36, 128);
  display.print("O");

  for (int x = 0; x < pressure_graph.size() - 1; x++)
  {
    display.drawLine(x + 14, mapfloat(pressure_graph[x], 0.0, 35.0, 114, 34), x + 15, mapfloat(pressure_graph[x + 1], 0.0, 35.0, 114, 34));
  }

  display.setCursor(236, 225);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t");
  display.drawLine(239, 214, 239, 215);

  display.setCursor(197, 225);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-1");
  display.drawLine(203, 214, 203, 215);

  display.setCursor(161, 225);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-2");
  display.drawLine(167, 214, 167, 215);

  display.setCursor(124, 225);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-3");
  display.drawLine(130, 214, 130, 215);

  display.setCursor(87, 225);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-4");
  display.drawLine(93, 214, 93, 215);

  display.setCursor(51, 225);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-5");
  display.drawLine(57, 214, 57, 215);
  display.drawLine(20, 214, 20, 215);
  display.drawLine(14, 214, 14, 215);

  display.setFont(u8g2_font_6x12_tr);
  display.setCursor(ALIGN_RIGHT(string2char(String(cmh2o, 0))) - 229, 231);
  display.print(String(lpm, 2));
  display.setCursor(33, 231);
  display.print("L/min");

  flow_graph_max = 0.0;
  flow_graph_min = 0.0;

  for (int x = 0; x < flow_graph.size() - 1; x++)
  {
    if (flow_graph[x] > flow_graph_max)
    {
      flow_graph_max = flow_graph[x];
    }
    if (flow_graph[x] < flow_graph_min)
    {
      flow_graph_min = flow_graph[x];
    }
  }

  for (int x = 0; x < flow_graph.size() - 1; x++)
  {
    display.drawLine(x + 14, mapfloat(flow_graph[x], flow_graph_min, flow_graph_max, 214, 134), x + 15, mapfloat(flow_graph[x + 1], flow_graph_min, flow_graph_max, 214, 134));
  }

  //  for(int x=0;x<flow_graph.size()-1;x++) {
  //    display.drawLine(x+14,mapfloat(flow_graph[x],-12.5,12.5,214,134),x+15,mapfloat(flow_graph[x+1],-12.5,12.5,214,134));
  //  }

  display.setCursor(236, 317);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t");
  display.drawLine(239, 306, 239, 307);

  display.setCursor(197, 317);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-1");
  display.drawLine(203, 306, 203, 307);

  display.setCursor(161, 317);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-2");
  display.drawLine(167, 306, 167, 307);

  display.setCursor(124, 317);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-3");
  display.drawLine(130, 306, 130, 307);

  display.setCursor(87, 317);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-4");
  display.drawLine(93, 306, 93, 307);

  display.setCursor(51, 317);
  display.setFont(u8g2_font_5x7_tr);
  display.print("t-5");
  display.drawLine(57, 306, 57, 307);
  display.drawLine(20, 306, 20, 307);
  display.drawLine(14, 306, 14, 307);

  display.setFont(u8g2_font_6x12_tr);
  display.setCursor(ALIGN_RIGHT(string2char(String(cmh2o, 0))) - 229, 320);
  display.print(String(liters, 2));
  display.setCursor(33, 320);
  display.print("L");

  volume_graph_max = 0.0;
  volume_graph_min = 0.0;

  for (int x = 0; x < volume_graph.size() - 1; x++)
  {
    if (volume_graph[x] > volume_graph_max)
    {
      volume_graph_max = volume_graph[x];
    }
    if (volume_graph[x] < volume_graph_min)
    {
      volume_graph_min = volume_graph[x];
    }
  }

  for (int x = 0; x < volume_graph.size() - 1; x++)
  {
    display.drawLine(x + 14, mapfloat(volume_graph[x], volume_graph_min, volume_graph_max, 314, 234), x + 15, mapfloat(volume_graph[x + 1], volume_graph_min, volume_graph_max, 314, 234));
  }

  /* Troubleshooting line error */
  //  for (int x = 0; x < volume_graph.size() - 1; x++) {
  //    display.drawLine(x + 14, mapfloat(volume_graph[x], volume_graph_min, volume_graph_max, 314, 234), x + 15, mapfloat(volume_graph[x + 1], volume_graph_min, volume_graph_max, 314, 234));
  //  }

  // Row 30 - 65 are off limits for Alarms/ Backup Rate
  if (backUpTrip)
  {
    display_backup();
  }
  if (faulted > 0)
  {
    display_fault();
  }

  display.sendBuffer(); // transfer internal memory to the display
}

void fault_detect()
{
  // *********   Fault Detect (Set faulted status) ************* //
  if (mask_et_select)
  { // "ET" selected
    // Alarm 1 - "PRESSURE HIGH" (ET Setting)
    if ((cmh2o > (target_pres * 1.1)) && (mode != 0) && run_timer > GRACE_PERIOD)
    {
      alarm1_trip = true;
      if (alarm1_trip_time == 0)
      {
        alarm1_trip_time = millis();
      }
    }
    else if ((cmh2o <= (target_pres * 1.1)) || (mode == 0) || run_timer <= GRACE_PERIOD)
    {
      alarm1_trip = false;
      alarm1_trip_time = 0;
    }
    //  Alarm 2 - "PRESSURE LOW" (ET Setting)
    if (target_peep <= 10)
    { // False trip if peep at 5 and Peak less than 10
      if (((cmh2o_set == target_peep) && (cmh2o < (target_peep - 2))) && (mode > 0) && run_timer > GRACE_PERIOD)
      {
        alarm2_trip = true;
        if (alarm2_trip_time == 0)
        {
          alarm2_trip_time = millis();
        }
      }
      else if ((cmh2o >= (target_peep - 2)) || (mode == 0) || run_timer <= GRACE_PERIOD)
      {
        alarm2_trip = false;
        alarm2_trip_time = 0;
      }
    }
    else
    { // All other Conditions
      if (((cmh2o_set == target_peep) && (cmh2o < (target_peep * .9))) && (mode > 0) && run_timer > GRACE_PERIOD)
      {
        alarm2_trip = true;
        if (alarm2_trip_time == 0)
        {
          alarm2_trip_time = millis();
        }
      }
      else if ((cmh2o >= (target_peep * .9)) || (mode == 0) || run_timer <= GRACE_PERIOD)
      {
        alarm2_trip = false;
        alarm2_trip_time = 0;
      }
    }
  }

  else
  { // "MASK" selected
    // Alarm 3 - "PEAK PRESSURE NOT REACHED" (MASK Setting)
    if (((cmh2o_set == target_pres) && (cmh2o < (cmh2o_set * .40))) && (mode > 0) && run_timer > GRACE_PERIOD)
    {
      // alarm3_trip = true; // ! Redundant with Alarm 6
      if (alarm3_trip_time == 0)
      {
        alarm3_trip_time = millis();
      }
    }
    else if (((cmh2o_set == target_pres) && (cmh2o > (cmh2o_set * 0.41))) || (mode == 0) || run_timer <= GRACE_PERIOD)
    {
      alarm3_trip = false;
      alarm3_trip_time = 0;
    }
    // Alarm 4 - "PRESSURE SENSOR ERROR" (MASK Setting)
    if ((cmh2o > 41 || cmh2o < -1.0) && (mode > 0) && run_timer > GRACE_PERIOD)
    {
      alarm4_trip = true;
      if (alarm4_trip_time == 0)
      {
        alarm4_trip_time = millis();
      }
    }
    else if ((cmh2o < 40 && cmh2o > -1.0) || (mode == 0) || run_timer <= GRACE_PERIOD)
    {
      alarm4_trip = false;
      alarm4_trip_time = 0;
    }
  }

  //  Alarm 5 - "MOTOR Overcurrent" 
  //  P peak must be ast least 15cmh2o for more than 1.5 Amps
  if (currentdraw > currentdraw_limit && (mode > 0) && run_timer > GRACE_PERIOD)
  {
    alarm5_trip = true;
    if (alarm5_trip_time == 0)
    {
      alarm5_trip_time = millis();
    }
  } else if (currentdraw <= currentdraw_limit || run_timer <= GRACE_PERIOD)
  {
    alarm5_trip = false;
    alarm5_trip_time = 0;
  }

  //  Alarm 6 - "Circuit Disconnected" 

  if ( (cmh2o_set == target_peep) && (lpm > 200) && (run_timer > GRACE_PERIOD) ) {
    alarm6_trip = true;
    if (alarm6_trip_time == 0) {
      alarm6_trip_time = millis();
    }
  } else if ( (cmh2o_set == target_peep) && (lpm < 200) || (run_timer <= GRACE_PERIOD) ) 
  {
    //alarm6_trip = false; // conditions drop when alarm goes off
    alarm6_trip_time = 0;
  }

  // Debounce each alarm against septoint to trigger Display/LED/Buzzer
  current_time = millis();
  if (alarm1_trip && (current_time - alarm1_trip_time > alarm1) && (alarm1_trip_time != 0))
  {
    faulted = 1;
    buzzer_on();
  }
  if (alarm2_trip && (current_time - alarm2_trip_time > alarm2) && (alarm2_trip_time != 0))
  {
    faulted = 2;
    buzzer_on();
  }
  if (alarm3_trip && (current_time - alarm3_trip_time > alarm3) && (alarm3_trip_time != 0))
  {
    faulted = 3;
    buzzer_on();
  }
  if (alarm4_trip && (current_time - alarm4_trip_time > alarm4) && (alarm4_trip_time != 0))
  {
    faulted = 4;
    buzzer_on();
  }
  if (alarm5_trip && (current_time - alarm5_trip_time > alarm5) && (alarm5_trip_time != 0))
  {
    faulted = 5;
    buzzer_on();
  }
  if (alarm6_trip && (current_time - alarm6_trip_time > alarm6) && (alarm6_trip_time != 0))
  {
    faulted = 6;
    buzzer_on();
  }

  if ((!alarm1_trip && !alarm2_trip && !alarm3_trip && !alarm4_trip && !alarm5_trip && !alarm6_trip) || mode == 0)
  {
    alarm6_trip = false; // Needs to stay on to tell user
    faulted = 0;
    buzzer_off();
  }
}

void io_init()
{
  pinMode(10, INPUT);
  pinMode(8, INPUT);
  pinMode(7, INPUT);
  btn_mode.debounceTime = 50;
  btn_mode.multiclickTime = 100;
  btn_mode.longClickTime = 500;
  pinMode(LED, OUTPUT);
  pinMode(BUZZER_ON_LOW, OUTPUT);
  pinMode(BUZZER_MED, OUTPUT);
  pinMode(BUZZER_HIGH, OUTPUT);
  pinMode(TI_ENABLE, OUTPUT);
  digitalWrite(TI_ENABLE, HIGH);
  analogReadRes(12);
  analogWriteRes(12);
  analogWrite(A14, MOTOR_MIN);

  Serial.begin(115200);
  delay(100);
  Serial.println("START");
  digitalWrite(BUZZER_ON_LOW, HIGH);
  delay(10);
  digitalWrite(BUZZER_MED, HIGH);
  digitalWrite(LED, HIGH);
}

// Function that executes whenever data is received from I2C master
//void receiveEvent(int howMany) {
//  while (Wire.available()) { // loop through all but the last
//    char c = Wire.read(); // receive byte as a character
//    Serial.println(c);
//  }
//}

void toggle_menu()
{
  static unsigned long last_menu_time = 0;
  unsigned long menu_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (menu_time - last_menu_time > 200)
  {
    // Change Values menu
    if (prog == 1)
    {
      if (menuCount < 7)
      {
        menuCount++;
      }
      else if (menuCount >= 7)
      {
        menuCount = 1;
      }
    }
    // PID config menu
    if (prog == 2)
    {
      if (menuCount < 3)
      {
        menuCount++;
      }
      else if (menuCount >= 3)
      {
        menuCount = 1;
      }
    }
    // Alarm congfig menu
    if (prog == 3)
    {
      if (menuCount < 4)
      {
        menuCount++;
      }
      else if (menuCount >= 4)
      {
        menuCount = 1;
      }
    }
  }
  last_menu_time = menu_time;
}

void toggle_mode()
{
  static unsigned long last_mode_time = 0;
  unsigned long mode_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (mode_time - last_mode_time > 200)
  {
    if (mode == 0 && bool_mode_select == false)
    {
      mode = 1;
      s_mode = "PS";
      avgTidalVolume.reset();
    }
    else if (mode == 0 && bool_mode_select == true)
    {
      mode = 2;
      s_mode = "AC";
      avgTidalVolume.reset();
    }
    else
    {
      mode = 0;
      s_mode = "O";
    }
  }
  last_mode_time = mode_time;
}

void toggle_prog()
{
  static unsigned long last_prog_time = 0;
  unsigned long prog_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (prog_time - last_prog_time > 200)
  {
    if (prog == 0)
    {
      prog = 1;
    }
    else
    { // PID and Alarm Timer screens removed for V2
      prog = 0;
    }
    // Reset Pointer Position (may have different Menu #'s)
    menuCount = 1;
  }
  last_prog_time = prog_time;
}

void update_pid(int num_ticks)
{
  // Rotary Button Press ends variable editing
  switch (menuCount)
  {
  case 1: // Line 1 Proportinal Value (Kp)
    tf_result = Kp;
    tf_inc = Kp_inc;
    break;
  case 2: // Line 2 Integral Value (Ki)
    tf_result = Ki;
    tf_inc = Ki_inc;
    break;
  case 3: // Line 3 Derivative Value (Kd)
    tf_result = Kd;
    tf_inc = Kd_inc;
    break;
  default:
    break;
  }

  tf_result += (num_ticks * tf_inc);

  switch (menuCount)
  {
  case 1: // Line 1 Proportinal Value (Kp)
    Kp = tf_result;
    break;
  case 2: // Line 2 Integral Value (Ki)
    Ki = tf_result;
    break;
  case 3: // Line 3 Derivative Value (Kd)
    Kd = tf_result;
    break;
  default:
    break;
  }

  // If config Error - DO SOMETHING
  if (myPID.setCoefficients(Kp, Ki, Kd, Hz))
  {
    Serial.println("There is a configuration error!");
  }
  else
  {
    Serial.print("PID Updated successfully !");
  }

  // Test updated PID attributs (Error will stop PID)
  int16_t pidout_test = myPID.step(cmh2o_set, cmh2o);
  pidout_test = 0;
}

void update_target(int num_ticks)
{
  // Rotary Button Press ends variable editing
  switch (menuCount)
  {
  case 1: // Line 1 Pressure (cm/H2O)(INT)
    tf_result = float(target_pres);
    tf_inc = pres_inc;
    break;
  case 2: // Line 2 PEEP (cm/H2O)(FLOAT)
    tf_result = float(target_peep);
    tf_inc = peep_inc;
    break;
  case 3: // Line 3 Breaths Per Minute(INT)
    tf_result = float(target_bmin);
    tf_inc = bmin_inc;
    break;
  case 4: // Line 4 IE
    tf_result = target_ie;
    tf_inc = ie_inc;
    break;
  case 5: // Line 5 Breath Detect Threshold
    tf_result = brth_thresh;
    tf_inc = brth_inc;
    break;
  case 6: // Line 6 Backup Rate Adjust
    tf_result = float(belowRateTime);
    tf_inc = BackupRate_inc;
    break;
  case 7: // Line 7 Mode select
    tf_result = 0.0;
    tf_inc = 0.0;
    break;
  case 8: // Line 8 Mask or ET select
    tf_result = 0.0;
    tf_inc = 0.0;
    break;

  default:
    break;
  }

  tf_result += (num_ticks * tf_inc);

  switch (menuCount)
  {
  case 1: // Line 1 Pressure (cm/H2O)(INT)
    if (tf_result >= pres_low && tf_result <= pres_high)
    {
      target_pres = int(tf_result);
      run_timer = 0;
    }
    break;
  case 2: // Line 2 PEEP (cm/H2O)(FLOAT)
    if (tf_result >= peep_low && tf_result <= peep_high)
    {
      target_peep = int(tf_result);
      run_timer = 0;
    }
    break;
  case 3: // Line 3 Breaths Per Minute(INT)
    if (tf_result >= bmin_low && tf_result <= bmin_high)
    {
      target_bmin = int(tf_result);
      run_timer = 0;
    }
    break;
  case 4: // Line 4 IE
    if (tf_result >= ie_low && tf_result <= ie_high)
    {
      target_ie = tf_result;
      run_timer = 0;
    }
    break;
  case 5: // Line 5 Breathing Threshold
    if (tf_result >= brth_low && tf_result <= brth_high)
    {
      brth_thresh = tf_result;
      run_timer = 0;
    }
    break;
  case 6: // Line 6 Backup Rate Adjust
    if (tf_result >= BackupRate_low && tf_result <= BackupRate_high)
    {
      belowRateTime = long(tf_result);
      run_timer = 0;
    }
    break;
  case 7: // Line 7 Mode Select
    if (mode == 0)
    { // Must be "OFF" to change modes
      if (bool_mode_select)
      {
        bool_mode_select = false;
        mode_select = "Press Support";
      }
      else
      {
        bool_mode_select = true;
        mode_select = "Assist Control";
      }
    }
    break;
  case 8: // ! Line 8 MASK / ET Select *** REMOVED ******
    if (mask_et_select)
    {
      mask_et_select = false;
      mask_select = "MASK";
    }
    else
    {
      mask_et_select = true;
      mask_select = "ET";
    }
    break;
  default:
    break;
  }
}

void update_values(int num_ticks)
{
  if (prog == 0)
  {
    // Do nothing
  }
  else if (prog == 1)
  {
    // Rotary Button Press ends variable editing
    update_target(num_ticks);
  }
  else if (prog == 2)
  {
    // Comment here
    update_pid(num_ticks);
  }
}

void volume_update()
{
  // Retrieve MAX from Liter Array
  tidalVolume = avgTidalVolume.getMax();
  tidalVolume /= 1000;
  avgTidalVolume.reset();
  liters = 0.0;
  t_liters = 0.0;
}

char *string2char(String command)
{
  if (command.length() != 0)
  {
    char *p = const_cast<char *>(command.c_str());
    return p;
  }
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
