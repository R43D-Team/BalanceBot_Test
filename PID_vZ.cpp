/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1 *** Proposed V 1.1.2 Modifed by ZHomeSlice***
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_vZ.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
         double Kp, double Ki, double Kd, int ControllerDirection) {

  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  inAuto = false;

  PID::SetOutputLimits(0, 255);  //default output limit corresponds to
                                 //the arduino pwm limits

  SampleTime = 100;  //default Controller Sample Time is 0.1 seconds

  PID::SetControllerDirection(ControllerDirection);
  PID::SetTunings(Kp, Ki, Kd);

  lastTime = millis() - SampleTime;
}


/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
/*
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      //Compute all the working error variables
	  
	  double input = *myInput;
      double error = *mySetpoint - input; 		// double error = mySetpoint - input;
      ITerm+= (ki * error); 					// double ITerm = ki * (ITerm + error);
      if(ITerm > outMax) ITerm= outMax;			// if(iTerm > 90) iTerm = 90;
      else if(ITerm < outMin) ITerm= outMin;    // else if(iTerm < -90) iTerm = -90;
      double dInput = (input - lastInput);      // double dTerm = Kd * (error - lastError);  
 
      //Compute PID Output
	       //double pTerm = Kp * error;
      double output = kp * error + ITerm - kd * dInput;
      
	  if(output > outMax) output = outMax;      // if(PIDValue > 799) PIDValue = 799;
      else if(output < outMin) output = outMin; // else if(PIDValue < -799) PIDValue = -799;
	  *myOutput = output;
	  
      //Remember some variables for next time
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}
*/
/*
bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long timeChange = (now - lastTime);
   if(timeChange>=SampleTime)
   {
      //Compute all the working error variables
	  double input = *myInput;
      double error = *mySetpoint - input;
      ITerm+= (ki * error);
      if(ITerm > outMax) ITerm= outMax;
      else if(ITerm < outMin) ITerm= outMin;
      double dInput = (input - lastInput);
 
      //Compute PID Output
      double output = kp * error + ITerm- kd * dInput;
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;
	  *myOutput = output;
	  
      //Remember some variables for next time
      lastInput = input;
      lastTime = now;
	  return true;
   }
   else return false;
}
*/

bool PID::Compute() {
  static int SkipCtr;
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (!inAuto) {
    SkipCtr = 21;  // Soft Start Skipps ki and kd for 20 cycles
    integral = 0;
    pre_error = 0;
    return false;
  }
  if (timeChange >= SampleTime)  // Sample time is now Minimum Sample Time
  {
    double DTerm = 0;
    double derivative = 0;
    // kp -  proportional gain
    // ki -  Integral gain
    // kd -  derivative gain
    // dt -  loop interval time
    // outMax - maximum value of manipulated variable
    // outMin - minimum value of manipulated variable

    //Compute all the working error variables
    double input = *myInput;

    // Calculate error
    double error = *mySetpoint - input;

    // Proportional term
    double PTerm = kp * error;
    SkipCtr--;
    if (SkipCtr <= 1) {
      SkipCtr = 1;
      // Integral term
      integral += error * (double)(timeChange * .001);  // uses real delta T not a fixed delta T
      ITerm = ki * integral;
      if ((ITerm > outMax) || (ITerm < outMin)) integral -= error * (double)(timeChange * .001);  // Prevemts Windup


      /*
///////////////////////////////////////////////////////////////////////////////////////////////////
We were not talking about that but it's something clever:
....Jerky robot, that's because the Kd * de(t)/dt term goes to infinity when you change the set-point. Try instead of using the angle error, e(t), use the negative of the actual value change, dangle(t)/dt. they are equivalent but this way you avoid infinity errors. 

Kd*de(t)/dt = -Kd*dangle(t)/dt
///////////////////////////////////////////////////////////////////////////////////////////////////
*/



      // Derivative term using error change
      //derivative = (error - pre_error)  / (double)(timeChange*.001); // uses real delta T not a fixed delta T
      //DTerm = kd * derivative;
      // Derivative term using angle change
      derivative = (input - lastInput) / (double)(timeChange * .001);  // uses real delta T not a fixed delta T
      DTerm = -kd * derivative;
    }
    //Compute PID Output
    double output = PTerm + ITerm + DTerm;

    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;
    *myOutput = output;

    //Remember some variables for next time
    pre_error = error;
    lastInput = input;
    lastTime = now;
    /*
// Debugging 
	for (static long QTimer = millis(); (long)( millis() - QTimer ) >= 100; QTimer = millis() ) {  // one line Spam Delay at 100 miliseconds
		  Serial.print(F("\tInput ")); Serial.print(input);
		  Serial.print(F("\mySetpoint ")); Serial.print(mySetpoint);
		  Serial.print(F("\tDelta T ")); Serial.print(timeChange);
		  Serial.print(F("\tPTerm ")); Serial.print(PTerm);
		  Serial.print(F("\tITerm ")); Serial.print(ITerm);
		  Serial.print(F("\tKd ")); Serial.print(kd);
		  Serial.print(F("\tderivative ")); Serial.print(derivative);
		  Serial.print(F("\tDTerm ")); Serial.print(DTerm);
		  Serial.print(F("\tOutput ")); Serial.print(output);
		  Serial.println();
	}	  
*/
    return true;

  } else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted. 
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::SetTunings(double Kp, double Ki, double Kd) {
  if (Kp < 0 || Ki < 0 || Kd < 0) return;

  dispKp = Kp;
  dispKi = Ki;
  dispKd = Kd;

  double SampleTimeInSec = ((double)SampleTime) / 1000;
  kp = Kp;
  ki = Ki;  // ki = Ki  * SampleTimeInSec;
  kd = Kd;  // kd = Kd/ SampleTimeInSec;

  if (controllerDirection == REVERSE) {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::SetSampleTime(int NewSampleTime) {
  if (NewSampleTime > 0) {
    double ratio = (double)NewSampleTime
                   / (double)SampleTime;
    //    ki *= ratio;
    //    kd /= ratio;
    SampleTime = (unsigned long)NewSampleTime;
  }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID::SetOutputLimits(double Min, double Max) {
  if (Min >= Max) return;
  outMin = Min;
  outMax = Max;

  if (inAuto) {
    if (*myOutput > outMax) *myOutput = outMax;
    else if (*myOutput < outMin) *myOutput = outMin;

    if (ITerm > outMax) ITerm = outMax;
    else if (ITerm < outMin) ITerm = outMin;
  }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID::SetMode(int Mode) {
  bool newAuto = (Mode == AUTOMATIC);
  if (newAuto == !inAuto) { /*we just went from manual to auto*/
    PID::Initialize();
  }
  inAuto = newAuto;
}

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
void PID::Initialize() {
  ITerm = *myOutput;
  lastInput = *myInput;
  if (ITerm > outMax) ITerm = outMax;
  else if (ITerm < outMin) ITerm = outMin;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads 
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID::SetControllerDirection(int Direction) {
  if (inAuto && Direction != controllerDirection) {
    kp = (0 - kp);
    ki = (0 - ki);
    kd = (0 - kd);
  }
  controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp() {
  return dispKp;
}
double PID::GetKi() {
  return dispKi;
}
double PID::GetKd() {
  return dispKd;
}
int PID::GetMode() {
  return inAuto ? AUTOMATIC : MANUAL;
}
int PID::GetDirection() {
  return controllerDirection;
}
