///////////////////////////////////////////////////////////////////////////////////////
//Sliding Mode Quadcopter Flight Controller V1 (SMCattitude.ino)
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>

#define ST_LSM303DLHC_L3GD20        (0)
#define ST_LSM9DS1                  (1)
#define NXP_FXOS8700_FXAS21002      (2)

// Define your target sensor(s) here based on the list above!
// #define AHRS_VARIANT    ST_LSM303DLHC_L3GD20
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002

#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#elif AHRS_VARIANT == ST_LSM9DS1
// ToDo!
#elif AHRS_VARIANT == NXP_FXOS8700_FXAS21002
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#else
#error "AHRS_VARIANT undefined! Please select a target sensor combination!"
#endif

// Create sensor instances.
#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
Adafruit_L3GD20_Unified       gyro(20);
Adafruit_LSM303_Accel_Unified accel(30301);
Adafruit_LSM303_Mag_Unified   mag(30302);
#elif AHRS_VARIANT == ST_LSM9DS1
// ToDo!
#elif AHRS_VARIANT == NXP_FXOS8700_FXAS21002
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);
#endif

//Unit Conversions//
float deg2rad = PI/180;       // [rad/deg]

///////////////////////////////////////////////////////////////////////////////////////
//Marvelmind Hedgehog Setup
///////////////////////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#define HWSERIAL Serial1

float hedgehog_x, hedgehog_y, hedgehog_z;// coordinates of hedgehog (X,Y,Z), mm
int hedgehog_pos_updated = 0;// flag of new data from hedgehog received
int cnt = 1; //Loop counter
bool high_resolution_mode;
#define HEDGEHOG_BUF_SIZE 40 
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs = 0;
#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
unsigned int hedgehog_data_id;
typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;

///////////////////////////////////////////////////////////////////////////////////////
//Magnetometer Calibration
///////////////////////////////////////////////////////////////////////////////////////

// Offsets applied to raw x/y/z mag values
float mag_offsets[3] = { 21.79F, -35.36F, 40.98F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 0.969, -0.024,  0.015 },
                                    { -0.024,  0.983, -0.035 },
                                    { 0.015, -0.035,  1.052 } };

float mag_field_strength = 37.45F;

//Filter Selection//
//Mahony filter; //Weak
Madgwick filter; //Strong (Computationally Expensive)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//User Initialized Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Misc//
int satBound = 1;     //Saturation Function Bound  
float yawCal =0; //[rad] Yaw Angle Calibration

//Equivalent Control Gains//
float cRoll = 1.5;                    
float cPitch = cRoll;                   
float cYaw = 3;                     

//Discontinuous Control Gains//
float kRoll = 20;      
float kPitch = kRoll;         
float kYaw = 8.5;

//Proportional Gains//
float kpx = 50;
float kpy = kpx;
float kpz = 30;

//Integral Gains//
float kix = 0;
float kiy = 0;
float kiz = 0;

//Derivative Gains//
float kdx = 50;
float kdy = kdx;
float kdz = 6;

//Max Receiver Inputs//
float maxRoll = 164.0*deg2rad;
float maxPitch = 164.0*deg2rad;
float maxYaw = 70.0*deg2rad;

//Hardware Paramaters//
float vMax = 555;                             // [rad/s] Maximum Motor Velocity

///Quadcopter Physical Parameters//
float Kf = 0.0000158;                         // [N-s2] Aerodynamic Force Constant
float Km = 0.00000029;                        // [N-m-s2] Aerodynamic Moment Constant
float Ixx = 0.016;                            // [kg/m2] Moment of Inertia Around x Axis
float Iyy = 0.016;                            // [kg/m2] Moment of Inertia Around y Axis
float Izz = 0.031;                            // [kg/m2] Moment of Inertia Around z Axis
float Jr = 0.00006;                           // [kg/m2] Rotor Inertia
float len = 0.225;                            // [m] Quadcopter Arm Length
float g = 9.81;

// PWM Settings
int pwmFreq = 250;
int pwmRes = 8;
float dt = 0.004;

// Total pulse length for esc
int escPulseTime = 4000;

// ESC Pins
int escOut1 = 5;
int escOut2 = 6;
int escOut3 = 10;
int escOut4 = 20;

int led = 13;

//Rx Pins
int ch1 = 7;
int ch2 = 8;
int ch3 = 14;
int ch4 = 35;
int ch5 = 33;
int ch6 = 34;

float p0 = -3319;
float p1 = 52.77;
float p2 = -0.245;
float p3 = 0.000549;
float p4 = -0.0000005792;
float p5 = 0.0000000002309;

// Timing Variables for Pulse Width
unsigned long prev1 = 0, prev2 = 0, prev3 = 0, prev4 = 0, prev5 = 0, prev6 = 0;
volatile unsigned long R1 = 1500, R2 = 1500, R3 = 0, R4 = 1500, R5 = 1500, R6 = 1500; // Receiver Pulses {Pitch, Roll, Heave, Yaw}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//System Initialized Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Inertia Terms//
float a1=(Iyy-Izz)/Ixx;
float a2=Jr/Ixx;
float a3=(Izz-Ixx)/Iyy;
float a4=Jr/Iyy;
float a5=(Ixx-Iyy)/Izz;
float b1=len/Ixx;
float b2=len/Iyy;
float b3=1/Izz;

//Electronic Speed Controllers//
float u1, u2, u3, u4, ux, uy, uz;                           // Thrust Forces {Heave, Pitch, Roll, Yaw}
float w1, w2, w3, w4;                           // Rotor Velocities {Front (ccw), Right (cw), Back (ccw), Left(cw)}
int esc1, esc2, esc3, esc4;                     // ESC Pulse Widths {Front (ccw), Right (cw), Back (ccw), Left(cw)}

//Receiver//            
float pitchDes, rollDes, yawDes;                        // Desired Euler Angles
float pitchDesD1, rollDesD1, yawDesD1;                  // Desired Euler Angular Rates
float pitchDesD1old, rollDesD1old, yawDesD1old;         // Last Loop Rate
float pitchDesD2, rollDesD2, yawDesD2;                  // Rate Derivative
float throttle;

//Gyroscope//
float gyroPitch, gyroRoll, gyroYaw;   //Euler Angles
int calcInt;

//Accelerometer//
float accRoll, accPitch;        //Euler Angles
float accX, accY, accZ, accA;    //Accelerations {x, y, z, Total Vector}
float gx, gy, gz;

//Sliding Mode Controller//
float pitch, roll, yaw, pitch2, roll2;       //Euler Angles

float pitchD1, rollD1, yawD1;       //Euler Angular Rates
float ePitch, eRoll, eYaw;          //Error
float ePitchD1, eRollD1, eYawD1;    //Error Derivative
float sPitch, sRoll, sYaw;          //Sliding Surfaces

//Misc//
int start = 0;              //Motor start

// Get maximum value for selected PWM resolution (100% Duty)
int pwmMax = 256;
// Initializing pulse for ESCs, 25% duty
int escInit = pwmMax/4;
unsigned long loopTimer;
unsigned long timer1;

int esc1PWM, esc2PWM, esc3PWM, esc4PWM;

float w1a, w2a, w3a, w4a, wr;   // [rad/s] Rotor Velocities
float w1Des, w2Des, w3Des, w4Des;   // [rad/s] Rotor Velocities
float F1, F2, F3, F4;     // Fault Magnitude
float F1p, F2p, F3p, F4p;     // Fault Magnitude

float xMag, yMag, zMag, mx, my, mz;

float sinPitch, sinRoll, sinYaw, cosPitch, cosRoll, cosYaw;

//Position PID Controller//
float X, Y, Z;           // [mm] Global Coordinates of hedgehog (X,Y,Z) 
float x, y, z;           // [mm] Local Coordinates of hedgehog (X,Y,Z) 
float exOld, eyOld, ezOld;   // [mm] Old Coordinates of hedgehog (X,Y,Z)
float xDes, yDes, zDes;  // Desired Coordinates
float ex, ey, ez;        // Position Error
float eix, eiy, eiz;     // Integrated Error
float exD1, eyD1, ezD1;  // Position Error Derivative
float gpsCal[4];

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){

  Serial.begin(500000);
  HWSERIAL.begin(500000);

  //Warning LED//
  pinMode(led,OUTPUT); 
  digitalWrite(led,HIGH); //Setup has begun LED ON
                     
  // Initialize the sensors.
  if(!gyro.begin())
  {
    /* There was a problem detecting the gyro ... check your connections */
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }

#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
#else
  if (!accel.begin())
  {
    /* There was a problem detecting the accel ... check your connections */
    Serial.println("Ooops, no accel detected ... Check your wiring!");
    while (1);
  }

  if (!mag.begin())
  {
    /* There was a problem detecting the mag ... check your connections */
    Serial.println("Ooops, no mag detected ... Check your wiring!");
    while (1);
  }
#endif

  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(250);

  //Indoor GPS Calibration//
  calcInt = 0;
  for (calcInt = 0; calcInt < 2000 ; calcInt ++){                           // Take 2000 readings for calibration.
    loop_hedgehog();                                                        // Read gps coordinates
    gpsCal[1] += hedgehog_x;
    gpsCal[2] += hedgehog_y;
    gpsCal[3] += hedgehog_z;                                                              // Wait                                          
    delay(3);
  }

  //Average Position Offset//
  gpsCal[1] /= 2000.0F;  // x
  gpsCal[2] /= 2000.0F;  // y
  gpsCal[3] /= 2000.0F;  // z 

  // Setup ESC Pins
  pinMode(escOut1, OUTPUT);
  pinMode(escOut2, OUTPUT);
  pinMode(escOut3, OUTPUT);
  pinMode(escOut4, OUTPUT);
  
  // All on Timer FTM0 -> pwmFreq
  analogWriteFrequency(escOut1, pwmFreq);

  // Set PWM resolution
  analogWriteResolution(pwmRes);

  // Initialize ESCs
  analogWrite(escOut1, escInit);
  analogWrite(escOut2, escInit);
  analogWrite(escOut3, escInit);
  analogWrite(escOut4, escInit);
  delay(5000);

  //Enable Pin Change Interrupts//
  attachInterrupt(ch1,ch1Int,CHANGE);
  attachInterrupt(ch2,ch2Int,CHANGE);
  attachInterrupt(ch3,ch3Int,CHANGE);
  attachInterrupt(ch4,ch4Int,CHANGE);
  attachInterrupt(ch5,ch5Int,CHANGE);
  attachInterrupt(ch6,ch6Int,CHANGE);                                                        

  digitalWrite(led,LOW); //Setup Complete LED
  loopTimer = micros();                                                    // Set the timer for the next loop.

}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  
  Serial.print(pitch/deg2rad);
  Serial.print("  ");
  Serial.print(roll/deg2rad);
  Serial.print("  ");
  Serial.print(pitch2);
  Serial.print("  ");
  Serial.print(roll2);
  Serial.print("  ");
  Serial.print(yaw/deg2rad);
  Serial.println();
 
/*
  Serial.print("  ");
  Serial.print(pitch/deg2rad);
  Serial.print("  ");
  Serial.print(roll/deg2rad);
  Serial.print(" ");
  Serial.print(yaw/deg2rad);
  Serial.print("  ");
  Serial.print(gyroPitch);
  Serial.print("  ");
  Serial.print(gyroRoll);
  Serial.print("  ");
  Serial.print(gyroYaw);
  Serial.print("  ");

  
  Serial.print(x*1000.0);
  Serial.print("  ");
  Serial.print(y*1000.0);
  Serial.print("  ");
  Serial.print(y*1000.0);
  Serial.print("  ");
  Serial.println(ez*1000);

  Serial.print(kpz*ez);
  Serial.print("  ");
  Serial.print(kiz*eiz);
  Serial.print("  ");
  Serial.println(kdz*ezD1);*/

  throttle = R3;                                      //We need the throttle signal as a base signal.
  gyro_signalen();
  loop_hedgehog();  
  accA = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));       //Calculate the total accelerometer vector.
  if(abs(accX) < accA){                                        //Prevent the asin function to produce a NaN
    accPitch = -asin((float)accX/accA);          //Calculate the pitch angle.
  }
  if(abs(accY) < accA){                                        //Prevent the asin function to produce a NaN
    accRoll = asin((float)accY/accA);          //Calculate the roll angle.
  }


  //Angular Rates [rad/s]//
  gyroRoll = 4.5*gx;
  gyroPitch = 4.5*gy;
  gyroYaw = -4.5*gz;

  //Filtered Angular Rates [rad/s]//
  pitchD1 = 0.7*pitchD1 + 0.3*gyroPitch;
  rollD1 = 0.7*rollD1 + 0.3*gyroRoll;
  yawD1 = 0.7*yawD1 + 0.3*gyroYaw;

  //Angles [rad]//
  pitch += gyroPitch*dt;
  roll += gyroRoll*dt;

  //Yawing Angle Adjustment//
  pitch += roll * sin(gyroYaw * dt);  //If the IMU has yawed transfer the roll angle to the pitch angle.                       
  roll -= pitch * sin(gyroYaw * dt);  //If the IMU has yawed transfer the pitch angle to the roll angle.
                                   
  //Complementary Filter//
  pitch = pitch * 0.99 + accPitch * 0.01;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  roll = roll * 0.99 + accRoll * 0.01;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.


  //Start Motors//
  if(R3 < 1050 && R4 < 1050)start = 1; //For starting the motors: throttle low and yaw left (step 1).
  if(start == 1 && R3 < 1050 && R4 > 1450){ //When yaw stick is back in the center position start the motors (step 2).
    start = 2;

    //Reset the SMC controller for a bumpless start.
    pitchDes = 0;
    rollDes = 0;
    yawDes = 0;
    pitch = 0;
    roll = 0;
    yaw = 0;
    pitchDesD1old = 0;
    rollDesD1old = 0;
    yawDesD1old = 0;    
  }
  
  //Stop Motors//
  if(start == 2 && R3 < 1050 && R4 > 1950)start = 0;

  //Desired Angular Acceleration [rad/s2]//
  pitchDesD2 = (pitchDesD1 - pitchDesD1old)/dt;
  rollDesD2 = (rollDesD1 - rollDesD1old)/dt;
  yawDesD2 = (yawDesD1 - yawDesD1old)/dt;

  //Desired Angular Velocity [rad/s]//
  pitchDesD1 = (R2-1500.0)*maxPitch/500.0;
  rollDesD1 = (R1-1500.0)*maxRoll/500.0;
  yawDesD1 = 0;

  //Transmitter Dead Band//
  pitchDesD1 = (abs(pitchDesD1) < 10.0*deg2rad) ? 0 : pitchDesD1;
  rollDesD1 = (abs(rollDesD1) < 10.0*deg2rad) ? 0 : rollDesD1;
 
  //Error Calculations [rad]//
  ePitch = pitchDes - pitch;
  eRoll = rollDes - roll;
  eYaw = yawDes - yaw;
 
  //Error Time Derivatives [rad/s]//
  ePitchD1 = pitchDesD1 - pitchD1;
  eRollD1 = rollDesD1 - rollD1;  
  eYawD1 = yawDesD1 - yawD1;;

  //Sliding Surfaces//
  sPitch = cPitch*ePitch + ePitchD1;
  sRoll = cRoll*eRoll + eRollD1;
  sYaw = cYaw*eYaw + eYawD1;
  
  //Sliding Surface Saturation//
  sPitch = (sPitch > satBound) ? satBound : (sPitch < -satBound) ? -satBound : sPitch;
  sRoll = (sRoll > satBound) ? satBound : (sRoll < -satBound) ? -satBound : sRoll;
  sYaw = (sYaw > satBound) ? satBound : (sYaw < -satBound) ? -satBound : sYaw;

  //Thrust Inputs//                              
  //throttle = (throttle > 1800.0) ? 1800.0 : (throttle < 1100.0) ? 1100.0 : throttle;                // Need room to control
  u1 = uz; // (throttle-1000.0)*30.0/1000.0; // [N] Heave
  u2 = ((kPitch*sPitch) + (cPitch*ePitchD1) + pitchDesD2 /*+(a2*rollD1*wr)*/ - (a1*rollD1*yawD1))/b1;     // [Nm] Pitch Torque
  u3 = ((kRoll*sRoll) + (cRoll*eRollD1) + rollDesD2 /*-(a4*pitchD1*wr)*/ - (a3*pitchD1*yawD1))/b2;         // [Nm] Roll Torque
  u4 = ((kYaw*sYaw) + (cYaw*eYawD1) + yawDesD2 - (a5*pitchD1*rollD1))/b3;             // [Nm] Yaw Torque

  //Rotor Velocities [rad/s]//
  w1 = sqrt(((u1-2*u2)/Kf + u4/Km)/4); // Front (CCW)
  w2 = sqrt(((u1-2*u3)/Kf - u4/Km)/4); // Right (CW)
  w3 = sqrt(((u1+2*u2)/Kf + u4/Km)/4); // Back (CCW)
  w4 = sqrt(((u1+2*u3)/Kf - u4/Km)/4); // Left (CW)

  //Nan Check//
  w1 = isnan(w1) ? 1 : w1;
  w2 = isnan(w2) ? 1 : w2;
  w3 = isnan(w3) ? 1 : w3;
  w4 = isnan(w4) ? 1 : w4;

  //ESC Pulse Widths [us]//
  esc1 = p5*w1*w1*w1*w1*w1 + p4*w1*w1*w1*w1 + p3*w1*w1*w1 + p2*w1*w1 + p1*w1 + p0;
  esc2 = p5*w2*w2*w2*w2*w2 + p4*w2*w2*w2*w2 + p3*w2*w2*w2 + p2*w2*w2 + p1*w2 + p0;
  esc3 = p5*w3*w3*w3*w3*w3 + p4*w3*w3*w3*w3 + p3*w3*w3*w3 + p2*w3*w3 + p1*w3 + p0; 
  esc4 = p5*w4*w4*w4*w4*w4 + p4*w4*w4*w4*w4 + p3*w4*w4*w4 + p2*w4*w4 + p1*w4 + p0;
  
  //Save Last Loop [rad/s]//
  rollDesD1old=rollDesD1;
  pitchDesD1old=pitchDesD1;
  yawDesD1old=yawDesD1;


  if (start == 2){                                                          //The motors are started.
    //ESC Limits//
    esc1 = (esc1 < 1100) ? 1100 : (esc1 > 2000) ? 2000 : esc1;
    esc2 = (esc2 < 1100) ? 1100 : (esc2 > 2000) ? 2000 : esc2;
    esc3 = (esc3 < 1100) ? 1100 : (esc3 > 2000) ? 2000 : esc3;
    esc4 = (esc4 < 1100) ? 1100 : (esc4 > 2000) ? 2000 : esc4;
  }

  //Motors Off
  else{
    esc1 = 1000;                                                 
    esc2 = 1000;                                                      
    esc3 = 1000;                                                   
    esc4 = 1000;                                       
  }

  // Calculate ESC pulse for PWM resolution
  esc1PWM = esc1*(pwmMax)/escPulseTime;
  esc2PWM = esc2*(pwmMax)/escPulseTime;
  esc3PWM = esc3*(pwmMax)/escPulseTime;
  esc4PWM = esc4*(pwmMax)/escPulseTime;

  //Create PWM for ESCs
  analogWrite(escOut1,esc1PWM);
  analogWrite(escOut2,esc2PWM);
  analogWrite(escOut3,esc3PWM);
  analogWrite(escOut4,esc4PWM);

  while(timer1 < dt*1000000) timer1 = micros() - loopTimer;          
  loopTimer = micros(); 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reading Receiver Signals 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

 void ch5Int(){
  if (digitalReadFast(ch5)){
    prev5 = micros();
  }
  else{
    R5 = micros() - prev5; 
  }
 }

  void ch6Int(){
  if (digitalReadFast(ch6)){
    prev6 = micros();
  }
  else{
    R6 = micros() - prev6;
   
  }
 }
 
 void ch1Int(){
  if (digitalReadFast(ch1)){
    prev1 = micros();
  }
  else{
    R1 = micros() - prev1;
  }
 }

 void ch2Int(){
  if (digitalReadFast(ch2)){
    prev2 = micros();
  }
  else{
    R2 = micros() - prev2;
  }
 }
 
  void ch3Int(){
   if (digitalReadFast(ch3)){
    prev3 = micros();
   }
   else{
    R3 = micros() - prev3;
   }  
  }
 
  void ch4Int(){
   if (digitalReadFast(ch4)){
    prev4 = micros();
   }
   else{
    R4 = micros() - prev4;
   }
 }
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reading Gyroscope and Accelerometer Angles
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void gyro_signalen(){

  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  xMag = mag_event.magnetic.x - mag_offsets[0];
  yMag = mag_event.magnetic.y - mag_offsets[1];
  zMag = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  mx = xMag * mag_softiron_matrix[0][0] + yMag * mag_softiron_matrix[0][1] + zMag * mag_softiron_matrix[0][2];
  my = xMag * mag_softiron_matrix[1][0] + yMag * mag_softiron_matrix[1][1] + zMag * mag_softiron_matrix[1][2];
  mz = xMag * mag_softiron_matrix[2][0] + yMag * mag_softiron_matrix[2][1] + zMag * mag_softiron_matrix[2][2];

    // Apply gyro zero-rate error compensation
  gx = gyro_event.gyro.x - 0.003429340;
  gy = gyro_event.gyro.y - 0.000649837;
  gz = gyro_event.gyro.z - 0.000710419;

  //Accelerometer Values//
  accX = accel_event.acceleration.x;
  accY = accel_event.acceleration.y;
  accZ = accel_event.acceleration.z;
  
  //Angle Filter//
  filter.update(gx/deg2rad, gy/deg2rad, gz/deg2rad,
                accX, accY, accZ,
                mx, my, mz);

  //Angles [rad]//
  yaw = -asin(sin(filter.getYaw()*deg2rad)) - yawCal;
  pitch2 = filter.getPitch();
  roll2 = filter.getRoll();
}

/*
  The circuit: * HEDGEHOG serial data (TX2 pin) to digital pin 0 (RXD)
 * Hedgehog ground to Arduino ground
 * IMPORTANT: When uploading code to Arduino, unplug the pin 0. This is a serial pin for the Arduino, and code will not upload if plugged in!
 * Beacons need to be setup as in the YouTube Marvelmind Indoor GPS unboxing video
 * Hedgehog (mobile beacon) will be audibly ticking when this is working.
 * Submap must be frozen
 * 
 * Returned data in [mm]
 * Baud rate: 500000 (Marvelmind)m
 * 
 * 
  */
void loop_hedgehog()
{int incoming_byte;
 int total_received_in_loop;
 int packet_received;
 bool good_byte;
 byte packet_size;
 uni_8x2_16 un16;
 uni_8x4_32 un32;

  total_received_in_loop= 0;
  packet_received= 0;
  
  while(HWSERIAL.available() > 0)
    {
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) 
      {
        hedgehog_serial_buf_ofs= 0;// restart bufer fill
        break;// buffer overflow
      }
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header

      incoming_byte= HWSERIAL.read();
      good_byte= false;
      switch(hedgehog_serial_buf_ofs)
      {
        case 0:
        {
          good_byte= (incoming_byte = 0xff);
          break;
        }
        case 1:
        {
          good_byte= (incoming_byte = 0x47);
          break;
        }
        case 2:
        {
          good_byte= true;
          break;
        }
        case 3:
        {
          hedgehog_data_id= (((unsigned int) incoming_byte)<<8) + hedgehog_serial_buf[2];
          good_byte=   (hedgehog_data_id == POSITION_DATAGRAM_ID) ||
                       (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID);
          break;
        }
        case 4:
        {
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_CM_DATA_SIZE);
              break;
            }
            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_MM_DATA_SIZE);
              break;
            }
          }
          break;
        }
        default:
        {
          good_byte= true;
          break;
        }
      }
      
      if (!good_byte)
        {
          hedgehog_serial_buf_ofs= 0;// restart bufer fill         
          continue;
        }     
      hedgehog_serial_buf[hedgehog_serial_buf_ofs++]= incoming_byte; 
      if (hedgehog_serial_buf_ofs>5)
        {
          packet_size=  7 + hedgehog_serial_buf[4];
          if (hedgehog_serial_buf_ofs == packet_size)
            {// received packet with required header
              packet_received= 1;
              hedgehog_serial_buf_ofs= 0;// restart bufer fill
              break; 
            }
        }
    }

  if (packet_received)  
    {
      hedgehog_set_crc16(&hedgehog_serial_buf[0], packet_size);// calculate CRC checksum of packet
      if ((hedgehog_serial_buf[packet_size] == 0)&&(hedgehog_serial_buf[packet_size+1] == 0))
        {// checksum success
          switch(hedgehog_data_id)
          {
            case POSITION_DATAGRAM_ID:
            {
              // coordinates of hedgehog (X,Y), cm ==> mm
              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              hedgehog_x= 10*long(un16.wi);

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              hedgehog_y= 10*long(un16.wi);
              
              // height of hedgehog, cm==>mm (FW V3.97+)
              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              hedgehog_z= 10*long(un16.wi);


              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= false;
              pid(); 
              break;
            }

            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              // coordinates of hedgehog (X,Y), mm
              un32.b[0]= hedgehog_serial_buf[9];
              un32.b[1]= hedgehog_serial_buf[10];
              un32.b[2]= hedgehog_serial_buf[11];
              un32.b[3]= hedgehog_serial_buf[12];
              hedgehog_x= un32.vi32;


              un32.b[0]= hedgehog_serial_buf[13];
              un32.b[1]= hedgehog_serial_buf[14];
              un32.b[2]= hedgehog_serial_buf[15];
              un32.b[3]= hedgehog_serial_buf[16];
              hedgehog_y= un32.vi32;

              
              // height of hedgehog, mm 
              un32.b[0]= hedgehog_serial_buf[17];
              un32.b[1]= hedgehog_serial_buf[18];
              un32.b[2]= hedgehog_serial_buf[19];
              un32.b[3]= hedgehog_serial_buf[20];
              hedgehog_z= un32.vi32;
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= true;
              pid(); 
              break;
            }        
          }
        } 
    }
}

// Calculate CRC-16 of hedgehog packet
void hedgehog_set_crc16(byte *buf, byte size)
{uni_8x2_16 sum;
 byte shift_cnt;
 byte byte_cnt;

  sum.w=0xffffU;

  for(byte_cnt=size; byte_cnt>0; byte_cnt--)
   {
   sum.w=(unsigned int) ((sum.w/256U)*256U + ((sum.w%256U)^(buf[size-byte_cnt])));

     for(shift_cnt=0; shift_cnt<8; shift_cnt++)
       {
         if((sum.w&0x1)==1) sum.w=(unsigned int)((sum.w>>1)^0xa001U);
                       else sum.w>>=1;
       }
   }

  buf[size]=sum.b[0];
  buf[size+1]=sum.b[1];// little endian
}

void pid(){

  //Global Coordinates//
  X = (hedgehog_x - gpsCal[1])/1000.0;
  Y = (hedgehog_y - gpsCal[2])/1000.0;
  Z = (hedgehog_z - gpsCal[3])/1000.0;
    
  //Local Coordinates//
  sinYaw = sin(yaw);
  cosYaw = cos(yaw);
  x=cosYaw*X + sinYaw*Y;
  y=-sinYaw*X + cosYaw*Y;
  z=Z;

  //Desired Height [mm]//
  xDes = 0;
  yDes = 0;
  zDes = (throttle-1000.0)/1000.0;         
  zDes = (zDes < 0) ? 0 : zDes; 
  
  //Error Calculations//
  ex = xDes - x;        //Error
  ey = yDes - y;        //Error
  ez = zDes - z;        //Error

  //Error Derivative Calculations//
  exD1 = (ex - exOld)*16.0;  
  eyD1 = (ey - eyOld)*16.0; 
  ezD1 = (ez - ezOld)*16.0; 

  //Error Integral Calculations//
  if(start == 2){
  eix += ex;       
  eiy += ey; 
  eiz += ez;
  eix = (eix > 0.5) ? 0.5 : eix;
  eiy = (eiy > 0.5) ? 0.5 : eiy;
  eiz = (eiz > 0.5) ? 0.5 : eiz;
  }
          
  //Control Outputs [N]//
  ux = kpx*ex + kdx*exD1; //+ kix*eix 
  uy = kpy*ey + kdy*eyD1; //+ kiy*eiy
  uz = kpz*ez + kdz*ezD1; //+ kiz*eiz

  //Desired Angles [rad]//
  pitchDes = 0; //-atan(-uy/sqrt(ux*ux+(uz+g)*(uz+g)));
  rollDes = 0; // -atan(ux/(uz+g));
  yawDes = 0;
  
  //Save Last Loop//
  exOld = ex;
  eyOld = ey;
  ezOld = ez;
}
