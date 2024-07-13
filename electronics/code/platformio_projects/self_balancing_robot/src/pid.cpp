// Obtained from Leonard Cross https://www.instructables.com/Upside-Up-Robot-Balancing-Revisited/
#include <Arduino.h>
#include "pid.h"

//----------------------------------------------------------------------------------------------------
// PID Controller
// Class contructor - initialize "memory" variables, set PID limits
PropIntDiff::PropIntDiff(double MINlimit, double MAXlimit, double Threshold) {
  ITerm = 0.0;
  lastInput = 0.0;
  inAuto = true;              // Default to auto mode
  MIN = MINlimit;
  MAX = MAXlimit;
  ITermThreshold = Threshold;
}

// Initialize PID "memory" variables
void PropIntDiff::initialize() {
  ITerm = 0.0;
  lastInput = 0.0;
}

// Calculate PID update
void PropIntDiff::calculate(double Setpoint, double Input) {
   // Check whether PID is in auto or manual
   if(!inAuto) return;
   
   // Compute all the working error variables
   error = Setpoint - Input;

   // If error is above threshold, accumulate normally, else bleed off ITerm
   if (abs(error) >= ITermThreshold) ITerm += (ki * error);        // Accumulate
   else ITerm = 0.75*ITerm;                                        // Bleed   

   if (ITerm < MIN) ITerm = MIN;
   if (ITerm > MAX) ITerm = MAX;
   dInput = (Input - lastInput);
  
   // Compute PID Output
   Output = kp * error + ITerm - kd * dInput;
   
   if (Output < MIN) Output = MIN;
   if (Output > MAX) Output = MAX;

    // Remember some variables for next time
   lastInput = Input;
}

// Calculate PID update, with D as an input
void PropIntDiff::calculate(double Setpoint, double Input, double dInput) {
   // Check whether PID is in auto or manual
   if(!inAuto) return;
   
   // Compute all the working error variables
   error = Setpoint - Input;

   // If error is above threshold, accumulate normally, else bleed off ITerm
   if (abs(error) >= ITermThreshold) ITerm += (ki * error);        // Accumulate
   else ITerm = 0.75*ITerm;                                        // Bleed   

   if (ITerm < MIN) ITerm = MIN;
   if (ITerm > MAX) ITerm = MAX;
  
   // Compute PID Output
   Output = kp * error + ITerm - kd * dInput;
   
   if (Output < MIN) Output = MIN;
   if (Output > MAX) Output = MAX;

    // Remember some variables for next time
   lastInput = Input;
}