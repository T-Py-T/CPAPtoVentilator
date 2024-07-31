#include <movingAvg.h>
#include <FastPID.h>

#define TI_ENABLE 19 // Cuts power to the motor
#define MOTOR_MIN 150 //50 doesnt work for Phillips motor


/* USE THIS FOR EXTERN OBJECT REFERENCE 
https://stackoverflow.com/questions/8910047/issue-declaring-extern-class-object
*/
extern movingAvg avgPressure;
extern FastPID myPID;

// Set up reference to Global Variables
extern int faulted, menuArrow, menuCount, target_pres, target_bmin;
extern int target_peep, avg_pressure, output_bits, inhale_detect;
extern int inhale, exhale;
extern float cmh2o, liters, currentdraw, Kp, Ki, Kd, Hz, peep; 
extern float target_ie, brth_thresh,inh_perct, exh_perct, curr_reading, prev_reading;
extern bool output_signed, inhaling, breathstarted, waitingforinhale;
extern double lpm;
extern int16_t pidout;
extern uint32_t delt_t, count;


void breathMode(){

  //Detect Patient Breath 
  if (!inhaling && lpm > brth_thresh) {
    breathstarted = true;
  }

  //Timing breathing cycle
  delt_t = millis() - count;

  // New Spontaneous Mode
  if (mode == 1) {
    if (inhaling) {
      if (delt_t >= inhale) {
        // Dwell Cycle (Maintain PEEP)
        cmh2o_set = target_peep;
        inhaling = false;
        count = millis();
      }
    } else {
      if (delt_t >= exhale) {
        // Inhale cycle start
        liters = 0.0;
        cmh2o_set = target_pres;
        inhaling = true;
        count = millis();
      }
    }
  }

  // Spontaneous Mode
  else if (mode == 2) {
    if (inhaling) {
      if (delt_t >= inhale) {
        breathstarted = false;
        cmh2o_set = target_peep;
        if (lpm <= brth_thresh) {
          inhaling = false;
        }
        count = millis();
      }
    } else {
      if (delt_t >= exhale || breathstarted) {
        liters = 0.0;
        cmh2o_set = target_pres;
        inhaling = true;
        count = millis();
      }
    }
  }

  pidout = myPID.step(cmh2o_set, cmh2o);

  if (mode > 0 && !faulted) {
    run_timer++;
    analogWrite(A14, pidout);
  } else if (mode == 0) {
    run_timer = 0;
    analogWrite(A14, MOTOR_MIN);
    myPID.clear();
    delt_t = 0;
  }

  //Cut Enable to motor if faulted
  if (faulted) {
    digitalWrite(TI_ENABLE, LOW);
  } else {
    digitalWrite(TI_ENABLE, HIGH);
  }
}

void breathingData(){
  /* 
  Update breathing rate is 1:2.0 -> % = parts/whole ->  1/3 == .333, 2/3 == .666
  inhale 1250 @ 16bmin 1:2.0ie, exhale 2500 (inhale * 2.0)                       
  */

  int cycle_time = int(60000 / target_bmin); // 3750ms @ 16 b/min 1.20 ie
  inh_perct = 1 / (1 + target_ie); // 0.29333 (~30%)
  exh_perct = target_ie / (1 + target_ie); // 0.704 (~70%)
  inhale = int(cycle_time * inh_perct); // 1250ms @ 16 b/min 1.20 ie
  exhale = int(cycle_time * exh_perct); // 2500ms @ 16 b/min 1.20 ie

  // get average pressure instance (FIFO)
  avg_pressure = avgPressure.reading(cmh2o);
}
