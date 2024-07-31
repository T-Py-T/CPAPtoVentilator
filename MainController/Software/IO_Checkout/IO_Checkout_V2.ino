/*
  AES Controls, PPAP Ventilator Upgrade program
  Written : 20200328
  Updated : 20200406 (Built to Teensy)
  Hardware:
  NHD 230128WG-BTFH-VZ Monochrome LCD Display (U8g2lib Drivers)
  Teensy 3.2 (www.pjrc.com/teensy)
*/
// Arduino Standard
#include <Arduino.h>
#include <Wire.h>

// Additional Installed
#include <SimpleRotary.h>
#include <U8g2lib.h>
#include <FastPID.h>


/* UNCOMMENT FOR EACH DEV AREA. */

//__________PCB Defined Pins________//
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


// ******* Testing values initialize *******
bool testing = false;
String s_mode = "PCB TEST", encoder_up_result = "", encoder_dn_result = "", mode_result = "", prog_result = "", encoder_btn_result = "", led_result = "", buzzer_result = "", display_result = "";
bool test_complete = false;
bool inputs_complete = false;
bool outputs_complete = false;
byte input_check = 0; 
byte output_check = 0;
float cmh2o, cmh2o_2, iqval, iqval_2;

// Prototype Delaration
void display_init();
void display_input_test();
void display_output_test();
void display_test_complete();
void io_init();
void mode_pass();
void output_test();
void prog_pass();
void rotary_btn_pass();
void rotary_pass();


void setup() {
  //comms_init();
  io_init();
  display_init();
}


// "Main" Loop
void loop() {
//  Serial.print("Output Status:  ");
//  Serial.println(inputs_complete);
//  Serial.print("led results:  ");
//  Serial.println(led_result);
//  Serial.print("buzzer results:  ");
//  Serial.println(buzzer_result);

  //Read from Sensor 1
  iqval = analogRead(A0) * (5.0 /4095.0);
  cmh2o = 35.56 * (iqval - 0.25)/3.75;
  //Read from Sensor 1
  iqval_2 = analogRead(A11) * (5.0 /4095.0);
  cmh2o_2 = 35.56 * (iqval_2 - 0.25)/3.75;

  
  if (test_complete == false){
    if (input_check  != 31) {
        display_input_test();
    } else if (input_check  == 31) {
        inputs_complete = true;
    }
    if (inputs_complete && output_check != 3) {
        display_output_test();
        output_test();
    }
    if (inputs_complete && output_check == 7) {
      outputs_complete = true;
      test_complete = true;
      buzzer_off();
    }
  }
  if(test_complete == true){
    display_test_complete();
  }
}


void buzzer_off() {
    digitalWrite(LED, LOW);
    digitalWrite(BUZZER_ON_LOW, LOW);
    digitalWrite(BUZZER_MED, LOW);
    digitalWrite(BUZZER_HIGH, LOW);
}

void buzzer_low() {
    digitalWrite(BUZZER_ON_LOW, HIGH);
}
void buzzer_med() {
    digitalWrite(BUZZER_ON_LOW, HIGH);
    digitalWrite(BUZZER_MED, HIGH);
}
void buzzer_high() {
    digitalWrite(BUZZER_ON_LOW, HIGH);
    digitalWrite(BUZZER_HIGH, HIGH);
}


void display_init() {
  // Initialize Display Settings
  display.begin();
  display.enableUTF8Print();

  // BLACK SCREEN TEST
  display.clearBuffer(); // clear the internal memory
  // Setup inverse
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
  display.drawBox(0, 0, 240, 128); /* (X of upper left edge, Y of upper left edge, Width, Height) */
  display.setFontMode(0);  /* activate transparent font mode */
  display.setDrawColor(0); /* color 1 for the box */
  display.setFont(u8g2_font_tenfatguys_tu); //Alarm Font
  display.setCursor(100, 68);
  display.print("LCD TEST");
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
  display.sendBuffer(); // transfer internal memory to the display
  delay(1000);


  // WHITE SCREEN TEST
  display.clearBuffer(); // clear the internal memory
  // Setup inverse
  display.setFontMode(0);  /* activate transparent font mode */
  display.setDrawColor(0); /* color 1 for the box */
  display.drawBox(0, 0, 240, 128); /* (X of upper left edge, Y of upper left edge, Width, Height) */
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
  display.setFont(u8g2_font_tenfatguys_tu); //Alarm Font
  display.setCursor(100, 68);
  display.print("LCD TEST");
  display.setFontMode(0);  /* activate transparent font mode */
  display.setDrawColor(0); /* color 1 for the box */
  display.sendBuffer(); // transfer internal memory to the display
  delay(1000);

  // BLACK SCREEN TEST
  display.clearBuffer(); // clear the internal memory
  // Setup inverse
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
  display.drawBox(0, 0, 240, 128); /* (X of upper left edge, Y of upper left edge, Width, Height) */
  display.setFontMode(0);  /* activate transparent font mode */
  display.setDrawColor(0); /* color 1 for the box */
  display.setFont(u8g2_font_tenfatguys_tu); //Alarm Font
  display.setCursor(100, 68);
  display.print("LCD TEST");
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
  display.sendBuffer(); // transfer internal memory to the display
  delay(1000);

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
  attachInterrupt(digitalPinToInterrupt(btn_mode), mode_pass, RISING);
  attachInterrupt(digitalPinToInterrupt(btn_prog), prog_pass, RISING);
  attachInterrupt(digitalPinToInterrupt(btn_rotary), rotary_btn_pass, RISING);

  //Interrupts for encoder
  attachInterrupt(digitalPinToInterrupt(encd_a), rotary_pass, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encd_b), rotary_pass, CHANGE);


  Serial.begin(115200);
  delay(100);
  Serial.println("START");
}

void display_input_test() {
  display.clearBuffer(); // clear the internal memory
  display.setFont(u8g2_font_tenfatguys_tu);
  display.setCursor(2, 12);
  display.print(s_mode);
  display.setFont(u8g2_font_profont17_tf);
  display.setCursor(130, 13);
  display.print("CHECK INPUTS");


  // CALCULATE AND DISPLAY READINGS
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(178, 60);
  display.print("Sensor 1");
  display.setCursor(178, 75);
  display.print(String(cmh2o,0));
  display.setCursor(178, 100);
  display.print("Sensor 2");
  display.setCursor(178, 115);
  display.print(String(cmh2o_2,0));
  
  // Banner and sensor Strings
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(2, 30);
  display.print("Check Pressure Readings FIRST!");
  
  
  // BUTTON PRESS TESTING
  display.setCursor(2, 52);
  display.print("Mode Button");
  display.setCursor(115, 52);
  display.print(":" + mode_result);
  display.setCursor(2, 68);
  display.print("Program Button");
  display.setCursor(115, 68);
  display.print(":" + prog_result);
  display.setCursor(2, 84);
  display.print("Rotary Button");
  display.setCursor(115, 84);
  display.print(":" + encoder_btn_result);
  display.setCursor(2, 100);
  display.print("Encoder Up" );
  display.setCursor(115, 100);
  display.print(":" + encoder_up_result);
  display.setCursor(2, 116);
  display.print("Encoder Down" );
  display.setCursor(115, 116);
  display.print(":" + encoder_dn_result);
  display.sendBuffer(); // transfer internal memory to the display
}

void display_output_test() {
  display.clearBuffer(); // clear the internal memory
  display.setFont(u8g2_font_tenfatguys_tu); // choose a suitable font
  display.setCursor(2, 12);
  display.print(s_mode);
  display.setFont(u8g2_font_profont17_tf); // choose a suitable font
  display.setCursor(130, 13);
  display.print("CHECK OUTPUTS");
  
  // Menu Creation Row 1 - 5
  display.setFont(u8g2_font_helvB08_tr);
  display.setCursor(10, 36);
  display.print("PRESS MODE BUTTON TO CONFIRM");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(10, 52);
  display.print("LED Test");
  display.setCursor(110, 52);
  display.print(":  " + led_result);
  display.setCursor(10, 68);
  display.print("Buzzer Test");
  display.setCursor(110, 68);
  display.print(":  " + buzzer_result);
  display.setFont(u8g2_font_helvB08_tr);
  display.setCursor(10, 84);
  display.print("PRESS PROG BUTTON TO CONFIRM");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(10, 100);
  display.print("Display Test");
  display.setCursor(110, 100);
  display.print(":  " + display_result);
  display.sendBuffer(); // transfer internal memory to the display
}

void display_test_complete() {
  display.clearBuffer(); // clear the internal memory
  display.setCursor(10, 68);
  display.println("TEST IS COMPLETE!");
  display.setCursor(10, 100);
  display.println("ALL IO VALID!");
  display.sendBuffer(); // transfer internal memory to the display
}


void output_test() {
  digitalWrite(LED, HIGH);
  buzzer_low();
  delay(500);
  buzzer_med();
  delay(500);
  buzzer_high();
  delay(500);
}


void mode_pass() {
  static unsigned long last_mode_time = 0;
  unsigned long mode_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (mode_time - last_mode_time > 150) {
    if(!inputs_complete){
      mode_result = " PASS ";
      bitWrite(input_check, 0, 1); // bit 0   (BYTE, BIT, VALUE)
    }
    if(inputs_complete && !outputs_complete){
      led_result = "PASS ";
      buzzer_result = "PASS ";
      bitWrite(output_check, 0, 1); // bit 0   (BYTE, BIT, VALUE)
      bitWrite(output_check, 1, 1); // bit 1   (BYTE, BIT, VALUE)
    }
  }
  last_mode_time = mode_time;
}


void prog_pass() {
  static unsigned long last_prog_time = 0;
  unsigned long prog_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (prog_time - last_prog_time > 150) {
    if(!inputs_complete){
      prog_result = " PASS ";
      bitWrite(input_check, 1, 1); // bit 1   (BYTE, BIT, VALUE)
    }
    if(inputs_complete && !outputs_complete){
      display_result = " PASS ";
      bitWrite(output_check, 2, 1); // bit 2   (BYTE, BIT, VALUE)
    }
  }
  last_prog_time = prog_time;
}

void rotary_btn_pass() {
  static unsigned long last_rotary_btn_time = 0;
  unsigned long rotary_btn_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (rotary_btn_time - last_rotary_btn_time > 150) {
    encoder_btn_result = " PASS ";
    bitWrite(input_check, 2, 1); // bit 2   (BYTE, BIT, VALUE)
  }
  last_rotary_btn_time = rotary_btn_time;
}


void rotary_pass() {
    // Encoder Dial Reading
    byte t_rotate;
    // 0 = not turning, 1 = CW, 2 = CCW
    t_rotate = rotary.rotate();
    if (t_rotate == 1) {
    encoder_up_result = " PASS ";
    bitWrite(input_check, 3, 1); // bit 3   (BYTE, BIT, VALUE)
    }
    if (t_rotate == 2) {
    encoder_dn_result = " PASS ";
    bitWrite(input_check, 4, 1); // bit 4   (BYTE, BIT, VALUE)
    }
}
