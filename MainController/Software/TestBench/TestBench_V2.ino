/*
  PPAP Ventilator Upgrade program
  USED TO VALIDATE A COMPLETED UNIT (Run through provided range a and test all functions)
  Written : 20200328
  Updated : 20200406 (Built to Teensy)
  Hardware:
  NHD 230128WG-BTFH-VZ Monochrome LCD Display (U8g2lib Drivers)
  Teensy 3.2 (www.pjrc.com/teensy)
*/

// Arduino Standard
#include <Arduino.h>
#include <Wire.h>

//Additional Written Libraries
#include "sensirion3200.h"
#include "logo.h"

// Additional Installed
#include <SimpleRotary.h>
#include <U8g2lib.h>
#include <movingAvg.h>


//__________PCB Defined Pins_________//
U8G2_T6963_240X128_F_8080 display(U8G2_R0, A7, A6, A3, 0, 1, 2, 3, 4, /*enable/wr=*/ A8, /*cs/ce=*/ 5, /*dc=*/ A9, /*reset=*/ 6); // Connect RD with +5V, FS0 and FS1 with GND
SimpleRotary rotary(12, 11, 10);
#define LED 9
#define BUZZER_ON_LOW 13
#define BUZZER_MED 15
#define BUZZER_HIGH 16
#define MOTOR_MIN 120
#define btn_mode 7
#define btn_prog 8
#define encd_a 11
#define encd_b 12
#define btn_rotary 10
//#define FLOW_CONVERSION 115  // J's old number
#define FLOW_CONVERSION 120  // From SFM3200 Manual

movingAvg avgFlow(100);

// Mode select init
String s_mode = "MODE: WAIT";
int inhale, exhale;
int mode = 0;
int prog = 0;
float tf_result, tf_inc, cmh2o, cmh2o_set;

// ******* Testing values initialize *******
int flow = 0;
int flow_smooth = 0;
int flow_smooth_prev = 0;
int disp_flow = 0;
int disp_flow_smooth = 0;
float current_hi_pres = 0.0; // ~5.0 cm/H2O == 287-289, 21 cm/H2O == 580, 31 cm/h20 == 620(0-4095)
float current_low_pres = 0.0; // ~5.0 cm/H2O == 287-289, 21 cm/H2O == 650, (0-4095)
float current_bmin = 0.0; // 3750ms total period (== 60000 / total period)
float current_ie = 0.0; // == exhale / inhale
bool test_run = false;
bool inhexh_start = false;
float curr_cmh2o, prev_reading, total_period, cur_inhale, cur_exhale, inh_perct, exh_perct, curr_reading;
uint32_t inhexh_start_time = 0;
// ******* Testing values initialize *******


// Prototype Delaration
void comms_init();
void display_init();
void display_test();
void io_init();
void reset_values();
void toggle_mode();
void toggle_prog();

void setup() {
  comms_init();
  io_init();
  display_init();
}

// "Main" Loop
void loop() {
  // ************  INPUT VALUE TESTING START ***************
  if (mode == 1) {
    
    // _____Flow Meter Calculations_____
    // flow table - open air, aprox conversion  115
    // 0=0L/m, 3771=34L/m(*derived), 7100=64L/m, 10600=90L/m
    // 13800=116L/m, 16800=141L/m, 19600=165L/m
    
    // TODO Flow given in SLM??? Check against L/Min
    flow = read_flow();
    Serial.print("flow: ");
    Serial.print(flow);
    Serial.println(" SLM(SL/Min)");
        
     flow_smooth = (0.9*flow_smooth) + (0.1*flow);
     flow_smooth_prev = flow_smooth;
     disp_flow = flow;
     disp_flow_smooth = flow_smooth;

    
    //______Get Current Pressure_____ 
    float iqval = analogRead(A11) * (5.0 / 4095.0);
    curr_cmh2o = 35.56 * (iqval - 0.25) / 3.75;
    curr_reading = (prev_reading * 0.9) + (curr_cmh2o * 0.1);


    // _____Get High and Low pressures_____
    if(current_low_pres == 0){
      current_low_pres = curr_cmh2o;
    }
    if(curr_cmh2o < current_low_pres){
      current_low_pres = curr_cmh2o;
    }
    if(curr_cmh2o > current_hi_pres){
      current_hi_pres = curr_cmh2o;
    }

    float rate_of_change = curr_reading - prev_reading;
    if (!inhexh_start && rate_of_change > 0.0) {
      inhexh_start = true;
      exhale = (millis() - inhexh_start_time);
      inhexh_start_time = millis();
      current_hi_pres = 0.0;
    } else if(inhexh_start && rate_of_change <= 0.0) {
      inhexh_start = false;
      inhale = (millis() - inhexh_start_time);
      inhexh_start_time = millis();
      current_low_pres = 0.0;
    }

    prev_reading = curr_reading;

    // Update breathing rate
    // 1:2.0 -> % = parts/whole ->  1/3 == .333, 2/3 == .666
    // inhale 1250 @ 16bmin 1:2.0ie
    // exhale 2500 (inhale * 2.0)
    // Calcs
    total_period = (inhale + exhale); // 3750 @
    inh_perct = inhale / total_period; // 0.33333 (~30%)
    exh_perct = (1 - inh_perct); // 0.6666 (~70%)

    // Final value of each
    current_ie = (exh_perct / inh_perct); // 2.0 ie (0.6666 / 0.3333)
    current_bmin = (60000 / total_period); // 16 bmin == 3750ms total_period
  // ************  INPUT VALUE TESTING END ***************
  
  } else{
    reset_values();
  }

  // Call Update Main Display
  display_test();
}


void comms_init() {
  Wire.begin();
  //disable internal pullups to support 3.3v I2C (Flow Meter)
//  pinMode(A4, INPUT);   // A4/D18/SDA
//  pinMode(A5, INPUT);   // A5/D19/SCL 
  delay(50);
  
  init_flow_sensor();

  Serial.begin(11520);
  delay(100);
  Serial.println("START");
}

void display_init() {
  // Initialize Display Settings
  display.begin();
  display.enableUTF8Print();
  
  // Draw Splash Screen (AES Logo)
  display.clearBuffer();
  display.drawXBMP( 0, 0, logo_width, logo_height, logo);
  display.sendBuffer();
  delay(2000);
}

void display_test() {
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_tenfatguys_tu); // choose a suitable font
  display.setCursor(2, 12);
  display.print(s_mode);
  display.setFont(u8g2_font_profont17_tf); // choose a suitable font
  display.setCursor(125, 13);
  display.print("TEST PARAMS");
  
  // Menu Creation Row 1 - 5
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(10, 52);
  display.print("High Pressure");
  display.setCursor(110, 52);
  display.print(":  " + String(current_hi_pres, 2));
  display.setCursor(180, 52);
  display.print("cmH2O");
  display.setCursor(10, 68);
  display.print("Low Pressure");
  display.setCursor(110, 68);
  display.print(":  " + String(current_low_pres, 2));
  display.setCursor(180, 68);
  display.print("cmH2O");
  display.setCursor(10, 84);
  display.print("b/min");
  display.setCursor(110, 84);
  display.print(":  " + String(int(current_bmin)));
  display.setCursor(10, 100);
  display.print("Raw Flow");
  display.setCursor(110, 100);
  display.print(":  " + String(disp_flow));
  display.setCursor(180, 100);
  display.print("L/Min");
  display.setCursor(10, 116);
  display.print("Smooth Flow");
  display.setCursor(110, 116);
  display.print(":  " + String(disp_flow_smooth));
  display.setCursor(180, 116);
  display.print("L/Min");


  display.sendBuffer();          // transfer internal memory to the display
}

void io_init() {
  pinMode(LED, OUTPUT);
  pinMode(BUZZER_ON_LOW, OUTPUT);
  pinMode(BUZZER_MED, OUTPUT);
  pinMode(BUZZER_HIGH, OUTPUT);
  analogReadRes(12);
  analogWriteRes(12);
  analogWrite(A14, MOTOR_MIN);

  //Interrupts for push_buttons
  attachInterrupt(digitalPinToInterrupt(btn_mode), toggle_mode, RISING);
  attachInterrupt(digitalPinToInterrupt(btn_rotary), reset_values, RISING);

}

void reset_values() {
  static unsigned long last_reset_time = 0;
  unsigned long reset_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (reset_time - last_reset_time > 150) {
    // Reset Tester Values to run new test
    current_hi_pres = 0.0; 
    current_low_pres = 0.0;
    current_bmin = 0.0;
    current_ie = 0.0;
    disp_flow = 0;
    disp_flow_smooth = 0;
  }    
  last_reset_time = reset_time;
}

void toggle_mode() {
  static unsigned long last_mode_time = 0;
  unsigned long mode_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (mode_time - last_mode_time > 150) {
    if (mode == 0) {
      mode = 1;
      s_mode = "MODE: TEST";
    } else if (mode == 1) {
      mode = 0;
      s_mode = "MODE: WAIT";
    }
  }
  last_mode_time = mode_time;
}
