#include <Arduino.h>

#define ENCODER_OPTIMIZE_INTERRUPTS

// Set up reference to Global Variables
extern String  mask_select, mode_select, s_mode;
extern int alarm1, alarm2, alarm3, alarm4, alarm5, alarm6;
extern int faulted, menuArrow, menuCount, target_pres, target_bmin;
extern int target_peep, avg_pressure, prog, mode, menuCount;
extern int btn_encoder_val, btn_menu_val, btn_mode_val;
extern float cmh2o, cmh2o_set, liters, currentdraw, Kp, Ki, Kd, Hz;
extern float target_ie, brth_thresh, tf_result, tf_inc;
extern long run_timer, last_encoder_pos;
extern bool mask_et_select, bool_mode_select;

extern const float brth_inc, brth_high, brth_low, pres_inc, pres_high, pres_low;
extern const float peep_inc, peep_high, peep_low, bmin_inc, bmin_high, bmin_low;
extern const float ie_inc, ie_high, ie_low, alarm_inc, currentdraw_limit;
extern const float Kp_inc, Ki_inc, Kd_inc;

/* USE THIS FOR EXTERN OBJECT REFERENCE 
https://stackoverflow.com/questions/8910047/issue-declaring-extern-class-object
*/
//Display
extern movingAvg avgPressure;
// Inputs
extern Encoder encoder;
extern Bounce btn_encoder;
extern Bounce btn_menu;
extern ClickButton btn_mode;


void toggle_menu() {
  static unsigned long last_menu_time = 0;
  unsigned long menu_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (menu_time - last_menu_time > 200) {
    // Change Values menu
    if (prog == 1) {
      if (menuCount < 7) {
        menuCount++;
      } else if (menuCount >= 7) {
        menuCount = 1;
      }
    }
    // PID config menu
    if (prog == 2) {
      if (menuCount < 3) {
        menuCount++;
      } else if (menuCount >= 3) {
        menuCount = 1;
      }
    }
    // Alarm congfig menu
    if (prog == 3) {
      if (menuCount < 4) {
        menuCount++;
      }
      else if (menuCount >= 4) {
        menuCount = 1;
      }
    }
  }
  last_menu_time = menu_time;
}


void toggle_mode() {
  static unsigned long last_mode_time = 0;
  unsigned long mode_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (mode_time - last_mode_time > 200) {
    if (mode == 0 && bool_mode_select == false) {
      mode = 1;
      s_mode = "C";
      avgPressure.reset();
    } else if (mode == 0 && bool_mode_select == true) {
      mode = 2;
      s_mode = "S";
      avgPressure.reset();
    } else {
      mode = 0;
      s_mode = "O";
    }
  }
  last_mode_time = mode_time;
}


void toggle_prog() {
  static unsigned long last_prog_time = 0;
  unsigned long prog_time = millis();
  // If interrupts come faster than 200ms, assume it's a bounce and ignore
  if (prog_time - last_prog_time > 200) {
    if (prog == 0) {
      prog = 1;
    }
    else {// PID and Alarm Timer screens removed for V2
      prog = 0;
    }
    // Reset Pointer Position (may have different Menu #'s)
    menuCount = 1;
  }
  last_prog_time = prog_time;
}


void update_alarms(int num_ticks) {
  // Rotary Button Press ends variable editing
  switch (menuCount) {
    case 1: // Line 1
      tf_result = float(alarm1);
      tf_inc = alarm_inc;
      break;
    case 2: // Line 2
      tf_result = float(alarm2);
      tf_inc = alarm_inc;
      break;
    case 3: // Line 3
      tf_result = float(alarm3);
      tf_inc = alarm_inc;
      break;
    case 4: // Line 4
      tf_result = float(alarm4);
      tf_inc = alarm_inc;
      break;
    default:
      break;
  }

  tf_result += (num_ticks * tf_inc);

  switch (menuCount) {
    case 1: // Line 1
      alarm1 = int(tf_result);
      break;
    case 2: // Line 2
      alarm2 = int(tf_result);
      break;
    case 3: // Line 3
      alarm3 = int(tf_result);
      break;
    case 4: // Line 4
      alarm4 = int(tf_result);
      break;
    default:
      break;
  }
}


void update_pid(int num_ticks) {
  // Rotary Button Press ends variable editing
  switch (menuCount) {
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

  switch (menuCount) {
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
  if (myPID.setCoefficients(Kp, Ki, Kd, Hz)) {
    Serial.println("There is a configuration error!");
  } else {
    Serial.print("PID Updated successfully !");
  }

  // Test updated PID attributs (Error will stop PID)
  int16_t pidout_test = myPID.step(cmh2o_set, cmh2o);
  pidout_test = 0;
}


void update_target(int num_ticks) {
  // Rotary Button Press ends variable editing
  switch (menuCount) {
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
    case 5: // Line 5 Mask or ET select
      tf_result = 0.0;
      tf_inc = 0.0;
      break;
    case 6: // Line 6 Mode select
      tf_result = 0.0;
      tf_inc = 0.0;
      break;
    case 7: // Line 7 Breath Detect Threshold
      tf_result = brth_thresh;
      tf_inc = brth_inc;
      break;

    default:
      break;
  }

  tf_result += (num_ticks * tf_inc);

  switch (menuCount) {
    case 1: // Line 1 Pressure (cm/H2O)(INT)
      if (tf_result >= pres_low && tf_result <= pres_high) {
        target_pres = int(tf_result);
        run_timer = 0;
      }
      break;
    case 2: // Line 2 PEEP (cm/H2O)(FLOAT)
      if (tf_result >= peep_low && tf_result <= peep_high) {
        target_peep = int(tf_result);
        run_timer = 0;
      }
      break;
    case 3: // Line 3 Breaths Per Minute(INT)
      if (tf_result >= bmin_low && tf_result <= bmin_high) {
        target_bmin = int(tf_result);
        run_timer = 0;
      }
      break;
    case 4: // Line 4 IE
      if (tf_result >= ie_low && tf_result <= ie_high) {
        target_ie = tf_result;
        run_timer = 0;
      }
      break;
    case 5: // Line 5 Mask or ET select
      if (mask_et_select) {
        mask_et_select = false;
        mask_select = "MASK";
      } else {
        mask_et_select = true;
        mask_select = "ET";
      }
      break;
    case 6: // Line 6 Mode Select
      if (bool_mode_select) {
        bool_mode_select = false;
        mode_select = "Controlled";
      } else {
        bool_mode_select = true;
        mode_select = "Spontaneous";
      }
      break;
    case 7: // Line 7 Mask or ET select
      if (tf_result >= brth_low && tf_result <= brth_high) {
        brth_thresh = tf_result;
        run_timer = 0;
      }
      break;
    default:
      break;
  }
}


void update_values(int num_ticks) {
  if (prog  == 0) {
    // Do nothing
  } else if (prog == 1) {
    // Rotary Button Press ends variable editing
    update_target(num_ticks);
  } else if (prog == 2) {
    // Comment here
    update_pid(num_ticks);
  } else if (prog == 3) {
    update_alarms(num_ticks);
  }
}


void updateEncoder(){
  long new_encoder_pos = encoder.read();
  if (new_encoder_pos >= last_encoder_pos + 4) {
    update_values(1);
    last_encoder_pos = new_encoder_pos;
  } else if (new_encoder_pos <= last_encoder_pos - 4) {
    update_values(-1);
    last_encoder_pos = new_encoder_pos;
  }

  btn_encoder.update();
  btn_menu.update();
  btn_mode.Update();

  btn_encoder_val = btn_encoder.read();
  btn_menu_val = btn_menu.read();
  btn_mode_val = btn_mode.clicks;  
}
