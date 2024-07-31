#include <Arduino.h>
#include <U8g2lib.h>
#include <movingAvg.h>
#include <CircularBuffer.h>
#include "helper.h"

// LCD/u8g2 text alignment helper functions
#define LCDWidth                        240//display. getDisplayWidth()
#define ALIGN_CENTER(t)                 ((LCDWidth - (display.getUTF8Width(t))) / 2)
#define ALIGN_RIGHT(t)                  (LCDWidth -  display.getUTF8Width(t))


/* USE THIS FOR EXTERN OBJECT REFERENCE 
https://stackoverflow.com/questions/8910047/issue-declaring-extern-class-object
*/

// Display Global Objects
extern U8G2_RA8835_320X240_F_8080 display;
extern movingAvg avgPressure;
extern CircularBuffer<float, 226> pressure_graph;
extern CircularBuffer<float, 226> flow_graph;
extern CircularBuffer<float, 226> volume_graph;
extern CircularBuffer<float, 160> current_graph;

// Set up reference to Global Variables
extern double lpm, last_lpm;
extern String alarm1_message, alarm2_message, alarm3_message;
extern String alarm4_message, alarm5_message, alarm6_message;
extern String  mask_select, mode_select, s_mode;
extern int alarm1, alarm2, alarm3, alarm4, alarm5, alarm6;
extern int faulted, menuArrow, menuCount, target_bmin;
extern int avg_pressure, target_pres, target_peep;
extern float cmh2o, liters, currentdraw, Kp, Ki, Kd, Hz;
extern float target_ie, brth_thresh;
extern float volume_graph_min, volume_graph_max; 
extern float pressure_graph_min, pressure_graph_max;
extern float current_graph_min, current_graph_max;
extern float flow_graph_min, flow_graph_max;
extern bool mask_et_select, bool_mode_select;


void display_fault() {
  String s_fault = "FAULT:  ";
  switch (faulted) {
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
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
  display.drawBox(0, 30, 240, 25); /* (X of upper left edge, Y of upper left edge, Width, Height) */
  display.setFontMode(0);  /* activate transparent font mode */
  display.setDrawColor(0); /* color 1 for the box */
  display.setFont(u8g2_font_helvB08_tf); //Alarm Font
  display.setCursor(2, 45);
  display.print(s_fault);
  display.setFontMode(1);  /* activate transparent font mode */
  display.setDrawColor(1); /* color 1 for the box */
}


void display_alarm_config() {
  menuArrow = (menuCount * 16) + 50;
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_profont17_tf); // choose a suitable font
  display.setCursor(ALIGN_CENTER(string2char("Alarm Settings")), 16);
  display.print("Alarm Settings");
  // Row 30 - 65 are off limits for alarms
  if (faulted > 0) {
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
  display.sendBuffer();          // transfer internal memory to the display
}


void display_pid_config() {
  menuArrow = (menuCount * 16) + 50;
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_profont17_tf); // choose a suitable font
  display.setCursor(ALIGN_CENTER(string2char("PID Tuning")), 16);
  display.print("PID Tuning");
  // Row 30 - 65 are off limits for alarms
  if (faulted > 0) {
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
  for (int x = 0; x < current_graph.size() - 1; x++) {
    display.drawLine(x, mapfloat(current_graph[x], 0.0, 5.78, 128, 80), x + 1, mapfloat(current_graph[x + 1], 0.0, 5.78, 128, 80));
  }

  //  display.print(":  " + String(int(cmh2o)));
  display.setFont(u8g2_font_8x13B_tr);
  display.setCursor(ALIGN_RIGHT(string2char(String(currentdraw, 3))) - 36, 126);
  //  Serial.println(string2char(cmh2o));
  display.print(String(currentdraw, 3));
  display.setCursor(208, 126);
  display.print("Amps");

  display.sendBuffer();          // transfer internal memory to the display
}


void display_setup() {
  menuArrow = (menuCount * 16) + 47;
  display.clearBuffer();          // clear the internal memory
  display.setFont(u8g2_font_profont17_tf); // choose a suitable font
  display.setCursor(ALIGN_CENTER(string2char("Ventilation Settings")), 16);
  display.print("Ventilation Settings");

    if (s_mode == "O") {
    display.setCursor(1, 20);
    display.setFont(u8g2_font_helvB10_tr );
    display.print("OFF");
  } else if (s_mode == "C") {
    display.setCursor(1, 27);
    display.setFont(u8g2_font_helvB24_tf);
    display.print("C");
  } else if (s_mode == "S") {
    display.setCursor(1, 27);
    display.setFont(u8g2_font_helvB24_tf);
    display.print("S");
  }
  
  // Row 30 - 65 are off limits for alarms
  if (faulted > 0) {
    display_fault();
  }
  // Menu Creation Row 1 - 5
  display.setFont(u8g2_font_helvB10_tr);
  //Row 1
  display.setCursor(10, 65);
  display.print("Ppeak");
  display.setCursor(110, 65);
  display.print(":  " + String(target_pres));
  display.setCursor(180, 65);
  display.print("cmH2O");
  //Row 2
  display.setCursor(10, 81);
  display.print("PEEP");
  display.setCursor(110, 81);
  display.print(":  " + String(int(target_peep)));
  display.setCursor(180, 84);
  display.print("cmH2O");
  //Row 3
  display.setCursor(10, 97);
  display.print("f Total");
  display.setCursor(110, 97);
  display.print(":  " + String(target_bmin));
  display.setCursor(180, 100);
  display.print("b/min");
  //Row 4
  display.setCursor(10, 113);
  display.print("I:E");
  display.setCursor(110, 113);
  display.print(":  1: " + String(target_ie, 1));
  //Row 5
  display.setCursor(10, 128);
  display.print("Interface");
  display.setCursor(110, 128);
  display.print(":  " + mask_select);
  //Row 6 
  display.setCursor(10, 143);
  display.print("Mode Select");
  display.setCursor(110, 143);
  display.print(":  " + mode_select);
  //Row 7  
  display.setCursor(10, 159);
  display.print("Inhale Trigger");
  display.setCursor(110, 159);
  display.print(":  " + String(brth_thresh));
  display.setCursor(180, 159);
  display.print("L/Min");

  // Pointer Creation logic
  display.setCursor(2, menuArrow);
  display.print(">");
  display.sendBuffer();   // transfer internal memory to the display
}


void display_main() {
  // Main Screen Recreate (U8G2)
  display.clearBuffer();  // clear the internal memory
  if (s_mode == "O") {
    display.setCursor(1, 20);
    display.setFont(u8g2_font_helvB10_tr );
    display.print("OFF");
  } else if (s_mode == "C") {
    display.setCursor(1, 27);
    display.setFont(u8g2_font_helvB24_tf);
    display.print("C");
  } else if (s_mode == "S") {
    display.setCursor(1, 27);
    display.setFont(u8g2_font_helvB24_tf);
    display.print("S");
  }

  display.setCursor(35, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("P");
  display.setCursor(43, 12);
  display.setFont(u8g2_font_6x13B_tr);
  display.print("peak");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(ALIGN_CENTER(string2char(target_pres)) - 68, 27);
  display.print(String(target_pres));

  display.setCursor(81, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("P");
  display.setCursor(88, 12);
  display.setFont(u8g2_font_6x13B_tr);
  display.print("avg");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(ALIGN_CENTER(string2char(avg_pressure)) - 26, 27);
  display.print(avg_pressure);

  display.setCursor(119, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("PEEP");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(ALIGN_CENTER(string2char(target_peep)) + 15, 27);
  display.print(target_peep);

  display.setCursor(163, 10);
  display.setFont(u8g2_font_8x13B_tr);
  display.print("f");
  display.setCursor(170, 12);
  display.setFont(u8g2_font_6x13B_tr);
  display.print("tot");
  display.setFont(u8g2_font_helvB10_tr);
  display.setCursor(ALIGN_CENTER(string2char(target_bmin)) + 56, 27);
  display.print(String(target_bmin));

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

  for (int x = 0; x < pressure_graph.size() - 1; x++) {
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
  display.setCursor(ALIGN_RIGHT(string2char(String(cmh2o, 0))) - 229, 228);
  display.print(String(lpm, 2));
  display.setCursor(33, 228);
  display.print("L/min");

  flow_graph_max = 0.0;
  flow_graph_min = 0.0;

  for (int x = 0; x < flow_graph.size() - 1; x++) {
    if (flow_graph[x] > flow_graph_max) {
      flow_graph_max = flow_graph[x];
    }
    if (flow_graph[x] < flow_graph_min) {
      flow_graph_min = flow_graph[x];
    }
  }

  for (int x = 0; x < flow_graph.size() - 1; x++) {
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

  for (int x = 0; x < volume_graph.size() - 1; x++) {
    if (volume_graph[x] > volume_graph_max) {
      volume_graph_max = volume_graph[x];
    }
    if (volume_graph[x] < volume_graph_min) {
      volume_graph_min = volume_graph[x];
    }
  }

  for (int x = 0; x < volume_graph.size() - 1; x++) {
    display.drawLine(x + 14, mapfloat(volume_graph[x], volume_graph_min, volume_graph_max, 314, 234), x + 15, mapfloat(volume_graph[x + 1], volume_graph_min, volume_graph_max, 314, 234));
  }

  //  for(int x=0;x<volume_graph.size()-1;x++) {
  //    display.drawLine(x+14,mapfloat(volume_graph[x],-0.1,0.1,314,234),x+15,mapfloat(volume_graph[x+1],-0.1,0.1,314,234));
  //  }

  // Row 30 - 65 are off limits for alarms
  if (faulted > 0) {
    display_fault();
  }

  display.sendBuffer();          // transfer internal memory to the display
}
