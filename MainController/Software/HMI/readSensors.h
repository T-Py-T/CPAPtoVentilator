#include <Arduino.h>


#define ENCODER_OPTIMIZE_INTERRUPTS
#define TI_ENABLE 19 // Cuts power to the motor
#define LED 9
#define MOTOR_MIN 150 //50 doesnt work for Phillips motor
#define Tube_ID 0.02// in m
#define Air_Viscosity 0.001  // Pa*s @ 20C)
#define Tube_len 1.5// in m
#define PA_to_MMHG 0.00750062   // Possibly unecessary
#define Tube_ID 0.02// in m
#define Air_Viscosity 0.001  // Pa*s @ 20C)
#define Tube_len 1.5// in m


// Set up reference to Global Variables
extern double lpm, last_lpm;
extern long liter_timer;
extern float currentdraw, last_currentdraw, iqval, cmh2o, last_cmh2o, diff_pa, t_lpm;
extern float liters, last_liters;

/* USE THIS FOR EXTERN OBJECT REFERENCE 
https://stackoverflow.com/questions/8910047/issue-declaring-extern-class-object
*/

// Display
extern CircularBuffer<float, 226> pressure_graph;
extern CircularBuffer<float, 226> flow_graph;
extern CircularBuffer<float, 226> volume_graph;
extern CircularBuffer<float, 160> current_graph;


template <typename T> int sgn(T val) {
 return (T(0) < val) - (val < T(0));
}


void readCurrent(){
  //Current Draw calc
  iqval = analogRead(A10) * (3.3 / 4095.0);
  currentdraw = mapfloat(iqval, 0.0, 3.3, 0.0, 5.78);
  current_graph.push(last_currentdraw);
  current_graph.push((last_currentdraw + currentdraw) / 2);
  current_graph.push((last_currentdraw + currentdraw + currentdraw) / 3);
  current_graph.push(currentdraw);
  last_currentdraw = currentdraw;
}


void readPressure(){
  //Pressure (cmH20) calc
  iqval = analogRead(A0) * (5.0 / 4095.0);
  cmh2o = 35.56 * (iqval - 0.25) / 3.75;
  pressure_graph.push(last_cmh2o);
  pressure_graph.push((last_cmh2o + cmh2o) / 2);
  pressure_graph.push(cmh2o);
  last_cmh2o = cmh2o;
}


void readFlow(){
  //Liters Per Minute calc
  iqval = analogRead(A11) * (5.0 / 4095.0);
  float diff_pa = sgn((iqval / 5.0) - 0.5) * sq((iqval / (5.0 * 0.4)) - 1.25) * 525.0;
  // Poiseuille's Law (pressure difference and volume flow)
  float t_lpm = (PI * pow(Tube_ID,4) * diff_pa) / (8 * Air_Viscosity * Tube_len);
  lpm = t_lpm * 60000; // Convert from M^3/s to L/Min
  flow_graph.push(last_lpm);
  flow_graph.push((last_lpm + lpm) / 2);
  flow_graph.push(lpm);
  last_lpm = lpm;
}

void readVolume(){
  //Liters calc

  //Stop Volume from continuing to grow in off position
  if (mode == 0) {liters = 0.0;} else {liters += (lpm * ((millis() - liter_timer) / 60000.0));}
  
  //Update Graphing
  volume_graph.push(last_liters);
  volume_graph.push((last_liters + liters) / 2);
  volume_graph.push(liters);
  last_liters = liters;
  liter_timer = millis();
}
