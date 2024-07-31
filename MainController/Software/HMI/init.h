#include <Arduino.h>
#include <U8g2lib.h>
#include "ClickButton.h"
#include "aesLogo.h"

// File contains Reading Functions
#define ENCODER_OPTIMIZE_INTERRUPTS
#define TI_ENABLE 19 // Cuts power to the motor
#define LED 9
#define BUZZER_ON_LOW 13
#define BUZZER_MED 15
#define BUZZER_HIGH 16
#define MOTOR_MIN 150 //50 doesnt work for Phillips motor


// Set up reference to Global Variables

/* USE THIS FOR EXTERN OBJECT REFERENCE 
https://stackoverflow.com/questions/8910047/issue-declaring-extern-class-object
*/

// Display
extern U8G2_RA8835_320X240_F_8080 display;
// Inputs
extern ClickButton btn_mode;


void io_init() {
  pinMode(10, INPUT);
  pinMode(8, INPUT);
  pinMode(7, INPUT);
  btn_mode.debounceTime   = 50;
  btn_mode.multiclickTime = 100;
  btn_mode.longClickTime  = 500;
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
}


void display_init() {
  display.begin();
  display.enableUTF8Print();
  // Draw Splash Screen (AES Logo)
  display.clearBuffer();
  display.drawXBMP( 0, 0, logo_width, logo_height, logo);
  display.sendBuffer();
  delay(2000);
  display.clearBuffer();
  display.drawXBMP( 0, 0, logo_width, logo_height, splash);
  display.sendBuffer();
  delay(2000);
  // Reset Lights and Buzzer
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER_ON_LOW, LOW);
  digitalWrite(BUZZER_MED, LOW);
  digitalWrite(BUZZER_HIGH, LOW);
}
