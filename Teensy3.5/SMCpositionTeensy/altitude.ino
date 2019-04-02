void altitude_control(void) {
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*  Main Code written by Joop Brokking
 *   Edited by Sital Khatiwada for Experimental Platform Compatibility
 *   
 *   Variables that need to be added:
 *  unsigned int takeoff_detected, manual_altitude_change; 
 *  int takeoff_throttle, error, manual_throttle,  manual_takeoff_throttle = 1500, motor_idle_speed = 1100;
 *  float pid_altitude_setpoint, pid_altitude_input, sch_kpz
 *  int pid_max_altitude = 400;                //Maximum output of the PID-controller (+/-).*/
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Flight Mode Setup
//=====================================================================================================================================================================
/* Flight mode needs to be set such that we can implement the right controller to engage
*  Make sure to set up flgiht mode code!
 *  For example: if R5 < 1300, auto level code only. If R5 >= 1300 altitude hold 
 *  Auto Level = Flight Mode 1, Altitude Hold = Flight Mode 2*/

if (R5 >= 1200 && R5 < 1600){
  flight_mode = 2;                       //If channel 6 is between 1200us and 1600us the flight mode is 2 (Altitude Hold)
}
else{
  flight_mode = 1;                        // Quadcopter is set to auto-level
}

//=====================================================================================================================================================================

// Initialization Prior to Takeoff (Should be integrated into our own initizalization code)
//=====================================================================================================================================================================  
  if (R3 < 1050 && R4 < 1050)start = 1;                              //For starting the motors: throttle low and yaw left (step 1).
  if (start == 1 && channel_3 < 1050 && channel_4 > 1450) {                        //When yaw stick is back in the center position start the motors (step 2).
    throttle = motor_idle_speed;                                                   //Set the base throttle to the motor_idle_speed variable.
    angle_pitch = accPitch;                                                 //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    angle_roll = accRoll;                                                   //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

    //ground_pressure = actual_pressure;   (Need to set z at ground as 0, I think this is being done in Hedgehog code)                                          //Register the pressure at ground level for altitude calculations.
    
    course_lock_heading = yaw;                                         //Set the current compass heading as the course lock heading. #### MIGHT NEED TO WAIT FOR YAW TO SETTLE OUT!###
    acc_total_vector_at_start = accA;                                  //Register the acceleration when the quadcopter is started.
    start = 2;
    //Set the start variable to 2 to indicate that the quadcopter is started.
    
    acc_alt_integrated = 0;
  if (R3 > 1400 && R3 < 1600) {                             //If the manual hover throttle is used and valid (between 1400us and 1600us pulse).
    takeoff_throttle = R3 - 1500;                           //Use the manual hover throttle.
    takeoff_detected = 1;
    // Might want to reset PID controller here for smooth takeoff. This is included in our code and should be integrated there
        /*pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;*/
  }
  else if (R3) {                                            //If the manual hover throttle value is invalid.
    error = 5;                                              // Error status = 5
    takeoff_throttle = 0;                                   //No hover throttle compensation.
    start = 0;                                              //Set the start variable to 0 to stop the motors.
  }
  }

  //Stopping the motors: throttle low and yaw right.
  if (start == 2 && R3 < 1050 && R4 > 1950) {
    start = 0;                                                                     //Set the start variable to 0 to disable the motors.
    takeoff_detected = 0;                                                          //Reset the auto take-off detection.
  }
//=====================================================================================================================================================================

// Takeoff Detection
//=====================================================================================================================================================================
  if (takeoff_detected == 0 && start == 2) {                                       //When the quadcopter is started and no take-off is detected.
    if (R3 > 1480 && throttle < 1750) throttle++;                                  //When the throttle is half way or higher, increase the throttle.
    if (throttle == 1750)error = 6;                                                //If take-off is not detected when the throttle has reached 1700: error = 6.
    if (R3 <= 1480) {                                                              //When the throttle is below the center stick position.
      if (throttle > motor_idle_speed)throttle--;                                  //Lower the throttle to the motor_idle_speed variable.
      //Reset the PID controllers for a smooth take-off.
      else {                                                                       //When the throttle is back at idle speed reset the PID controllers.
        /*pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_output_roll = 0;            // THIS ELSE STATEMENT SHOULD ALSO BE PART OF OUR INITIALIZATION PROCESS
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_output_pitch = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
        pid_output_yaw = 0;*/
      }
    }
    // Need Altitude acceleration (Take it from onboard IMU accelerometer)
    // accZ > 3, because accZ > 0 in order to counteract gravity and takeoff as well. We can play around with this value.
    if (accZ > 3) {                                                                //A take-off is detected when the quadcopter is accelerating.
      takeoff_detected = 1;                                                        //Set the take-off detected variable to 1 to indicate a take-off.
      pid_altitude_setpoint = 1;                                                    //Set the altitude setpoint at 1 meters above ground.
      if (throttle > 1400 && throttle < 1700) takeoff_throttle = throttle - 1530;  //If the automated throttle is between 1400 and 1600us during take-off, calculate take-off throttle.
      else {                                                                       //If the automated throttle is not between 1400 and 1600us during take-off.
        takeoff_throttle = 0;                                                      //No take-off throttle is calculated.
        error = 7;                                                                 //Show error 7 on the red LED.
      }*/
    }
  }
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////// PID ALTITUDE CONTROL CODE /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
if (flight_mode >= 2 && takeoff_detected == 1) {                                     //If the quadcopter is in altitude mode and flying.
      if (pid_altitude_setpoint == 0)pid_altitude_setpoint = 1;                      //If not yet set, set the PID altitude setpoint at 1 meters.
      //When the throttle stick position is increased or decreased the altitude hold function is partially disabled. The manual_altitude_change variable
      //will indicate if the altitude of the quadcopter is changed by the pilot.
      manual_altitude_change = 0;                                                    //Preset the manual_altitude_change variable to 0.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0.
      if (R3 > 1600) {                                                               //If the throtttle is increased above 1600us (60%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = 1;                                                   //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1600) / 3;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }
      if (R3 < 1400) {                                                               //If the throtttle is lowered below 1400us (40%).
        manual_altitude_change = 1;                                                  //Set the manual_altitude_change variable to 1 to indicate that the altitude is adjusted.
        pid_altitude_setpoint = 1;                                                   //Adjust the setpoint to the actual pressure value so the output of the P- and I-controller are 0.
        manual_throttle = (channel_3 - 1400) / 5;                                    //To prevent very fast changes in hight limit the function of the throttle.
      }

      //Calculate the PID output of the altitude hold.
      pid_altitude_input = z;                                                        //Set the setpoint (pid_altitude_input) of the PID-controller.
      ez = pid_altitude_input - pid_altitude_setpoint;                               //Calculate the error between the setpoint and the actual pressure value.

      //To get better results the P-gain is increased when the error between the setpoint and the actual pressure value increases.
      //The variable sch_kpz will be used to adjust the P-gain of the PID-controller.
      
      sch_kpz = 0;    //This is the "gain scheduled" P gain                         //Set the scheduled P gain to 0.
      if (ez > 10 || ez < -10) {                                                    //If the error between the setpoint and the actual pressure is larger than 10 or smaller then -10.
        sch_kpz = (abs(ez) - 10) / 20.0;                                            //The positive sch_kpz variable is calculated based based on the error.
        if (sch_kpz > 3)sch_kpz = 3;                                                //To prevent extreme P-gains it must be limited to 3.
      }

      //In the following section the I-output is calculated. It's an accumulation of errors over time.
      //The time factor is removed as the program loop runs at 250Hz.
      kiz += (pid_i_gain_altitude / 100.0) * ez;
      if (kiz > pid_max_altitude)kiz = pid_max_altitude;
      else if (kiz < pid_max_altitude * -1)kiz = pid_max_altitude * -1;

      // PID CONTROL ALGORITHM
      u1 = (kpz + sch_kpz) * ez + kiz + kdz * ezD1;                                 // Note here that kpz+sch_kpz makes up our total P gain
      
      //To prevent extreme PID-output the output must be limited.
      if (u1 > pid_max_altitude)u1 = pid_max_altitude;
      else if (u1 < pid_max_altitude * -1)u1 = pid_max_altitude * -1;
    }

    //If the altitude hold function is disabled some variables need to be reset to ensure a bumpless start when the altitude hold function is activated again.
    else if (flight_mode < 2 && pid_altitude_setpoint != 0) {                        //If the altitude hold mode is not set and the PID altitude setpoint is still set.
      pid_altitude_setpoint = 0;                                                     //Reset the PID altitude setpoint.
      u1 = 0;                                                       //Reset the output of the PID controller.
      kiz = 0;                                                        //Reset the I-controller.
      manual_throttle = 0;                                                           //Set the manual_throttle variable to 0 .
      manual_altitude_change = 1;                                                    //Set the manual_altitude_change to 1.
    }
  }
}
  
