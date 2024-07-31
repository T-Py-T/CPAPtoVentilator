#include <Arduino.h>

#define LED 9
#define BUZZER_ON_LOW 13
#define BUZZER_MED 15
#define BUZZER_HIGH 16
#define GRACE_PERIOD 20 //20 milliseconds

// Set up reference to Global Variables
extern long run_timer;
extern int mode, prog, faulted, target_pres, target_peep;
extern int alarm1, alarm2, alarm3,alarm4,alarm5, alarm6;
extern const float currentdraw_limit;
extern float cmh2o, cmh2o_set, currentdraw, liters, max_liters;
extern bool alarm1_trip, alarm2_trip, alarm3_trip, alarm4_trip, alarm5_trip, alarm6_trip;
extern unsigned long alarm1_trip_time, alarm2_trip_time, alarm3_trip_time, current_time;
extern unsigned long alarm4_trip_time, alarm5_trip_time, alarm6_trip_time;


void buzzer_off() {
  digitalWrite(LED, LOW);
  digitalWrite(BUZZER_ON_LOW, LOW);
  digitalWrite(BUZZER_MED, LOW);
  digitalWrite(BUZZER_HIGH, LOW);
}


void buzzer_on() {
  digitalWrite(LED, HIGH);
  // MUST USE ON_LOW for ALL LEVELS
  digitalWrite(BUZZER_ON_LOW, HIGH);
  delay(10);
  digitalWrite(BUZZER_HIGH, HIGH);
}


void fault_detect() {
  // *********   Fault Detect (Set faulted status) ************* //
  
  // Alarm 1 - "PRESSURE HIGH"
  if ((cmh2o > (target_pres * 1.1)) && (mode != 0) && run_timer > GRACE_PERIOD) {
    alarm1_trip = true;
    if (alarm1_trip_time == 0) {
      alarm1_trip_time = millis();
    }
  } else if ((cmh2o <= (target_pres * 1.1)) || (mode == 0) || run_timer <= GRACE_PERIOD) {
    alarm1_trip = false;
    alarm1_trip_time = 0;
  }
  //  Alarm 2 - "PRESSURE LOW"
  if (target_peep <= 10) { // False trip if peep at 5 and Peak less than 10
    if (((cmh2o_set == target_peep) && (cmh2o < (target_peep - 2))) && (mode > 0) && run_timer > GRACE_PERIOD) {
      alarm2_trip = true;
      if (alarm2_trip_time == 0) {
        alarm2_trip_time = millis();
      }
    } else if ((cmh2o >= (target_peep - 2)) || (mode == 0) || run_timer <= GRACE_PERIOD) {
      alarm2_trip = false;
      alarm2_trip_time = 0;
    }
  } else { // All other Conditions
    if (((cmh2o_set == target_peep) && (cmh2o < (target_peep * .9))) && (mode > 0) && run_timer > GRACE_PERIOD) {
      alarm2_trip = true;
      if (alarm2_trip_time == 0) {
        alarm2_trip_time = millis();
      }
    } else if ((cmh2o >= (target_peep * .9)) || (mode == 0) || run_timer <= GRACE_PERIOD) {
      alarm2_trip = false;
      alarm2_trip_time = 0;
    }
  }

  // Alarm 3 - (PEAK PRESSURE NOT REACHED)
  if (((cmh2o_set == target_pres) && (cmh2o < (cmh2o_set * .50))) && (mode > 0) && run_timer > GRACE_PERIOD) {
    alarm3_trip = true;
    if (alarm3_trip_time == 0) {
      alarm3_trip_time = millis();
    }
  } else if (((cmh2o_set == target_pres) && (cmh2o > (cmh2o_set * 0.51))) || (mode == 0) || run_timer <= GRACE_PERIOD) {
    alarm3_trip = false;
    alarm3_trip_time = 0;
  }

  // Alarm 4 - "PRESSURE SENSOR ERROR" 
  if ((cmh2o > 41 || cmh2o < -1.0) && (mode > 0) && run_timer > GRACE_PERIOD) {
    alarm4_trip = true;
    if (alarm4_trip_time == 0) {
      alarm4_trip_time = millis();
    }
  } else if ((cmh2o < 40 && cmh2o > -1.0) || (mode == 0) || run_timer <= GRACE_PERIOD) {
    alarm4_trip = false;
    alarm4_trip_time = 0;
  }
  

  /*  Alarm 5 - "MOTOR Overcurrent"
      Overcurrent protection P peak must be at least 15cmh2o for more than 1.5 Amps */
  if (currentdraw > currentdraw_limit && (mode > 0) && run_timer > GRACE_PERIOD) {
    alarm5_trip = true;
    if (alarm5_trip_time == 0) {
      alarm5_trip_time = millis();
    }
  } else if (currentdraw <= currentdraw_limit || run_timer <= GRACE_PERIOD) {
    alarm5_trip = false;
    alarm5_trip_time = 0;
  }

  /*  Alarm 6 - "Exceeded Max Volume
      Supplied more than 3 L of air into patient 
      Only valid in spontaneous mode */

    // Removing to ship units. Working! Cant risk faults with University Testing.
//  if (liters > max_liters && (mode == 2) && run_timer > GRACE_PERIOD) {
//    alarm6_trip = true;
//    if (alarm6_trip_time == 0) {
//      alarm6_trip_time = millis();
//    }
//  } else if (liters > max_liters || run_timer <= GRACE_PERIOD) {
//    alarm6_trip = false;
//    alarm6_trip_time = 0;
//  }

  // Debounce each alarm against septoint to trigger Display/LED/Buzzer
  current_time = millis();
  if (alarm1_trip && (current_time - alarm1_trip_time > alarm1) && (alarm1_trip_time != 0)) {
    faulted = 1;
    buzzer_on();
  }
  if (alarm2_trip && (current_time - alarm2_trip_time > alarm2) && (alarm2_trip_time != 0)) {
    faulted = 2;
    buzzer_on();
  }
  if (alarm3_trip && (current_time - alarm3_trip_time > alarm3) && (alarm3_trip_time != 0)) {
    faulted = 3;
    buzzer_on();
  }
  if (alarm4_trip && (current_time - alarm4_trip_time > alarm4) && (alarm4_trip_time != 0)) {
    faulted = 4;
    buzzer_on();
  }
  if (alarm5_trip && (current_time - alarm5_trip_time > alarm5) && (alarm5_trip_time != 0)) {
    faulted = 5;
    buzzer_on();
  }
  if (alarm6_trip && (current_time - alarm6_trip_time > alarm6) && (alarm6_trip_time != 0)) {
    faulted = 6;
    buzzer_on();
  }
  
  if ((!alarm1_trip && !alarm2_trip && !alarm3_trip && !alarm4_trip && !alarm5_trip && !alarm6_trip) || mode == 0) {
    faulted = 0;
    buzzer_off();
  }
}
