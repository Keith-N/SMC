///////////////////////////////////////////////////////////////////////////////////////
//Sliding Mode Quadcopter Flight Controller V1 (SMCattitude.ino)
///////////////////////////////////////////////////////////////////////////////////////
//Controller designed by Sital Khatiwada. Controller programmed and implemented by 
//Aaron Cantara and Michael Goulet at the University of New Hampshire 2018.
// Edited for Teensy compatibility by Sital Khatiwada (3/1/2019)
///////////////////////////////////////////////////////////////////////////////////////
//Description: 
///////////////////////////////////////////////////////////////////////////////////////
//Non linear sliding mode quadcopter controller. Version 1 (SMCattitude.ino) is 
//designed to control the attitude only using the pitch, roll, and yaw readings from
//the gyroscope and accelerometer.
///////////////////////////////////////////////////////////////////////////////////////
//Extensively modified from YMFC-AL open source code by Joop Brooking.
//http://www.brokking.net/ymfc-al_main.html
///////////////////////////////////////////////////////////////////////////////////////
//TERMS OF USE
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you 
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////
//Preliminary Commands
///////////////////////////////////////////////////////////////////////////////////////

//Additional Libraries//
#include <Wire.h>             // Gyroscope Communication
#include <Adafruit_Sensor.h>
#include <Adafruit_FXOS8700.h>
#include <Mahony.h>
#include <Madgwick.h>

//Gyro Stuff//
#define ST_LSM303DLHC_L3GD20        (0)
#define ST_LSM9DS1                  (1)
#define NXP_FXOS8700_FXAS21002      (2)
#define AHRS_VARIANT   NXP_FXOS8700_FXAS21002
#if AHRS_VARIANT == ST_LSM303DLHC_L3GD20
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#elif AHRS_VARIANT == ST_LSM9DS1
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


// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { 6.39F, -38.13F, 32.84F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.976,  -0.035,  -0.016 },
                                    {  -0.035,  1.004, -0.032 },
                                    {  -0.016, -0.032,  1.023 } };

float mag_field_strength        = 38.33F;

// Mahony is lighter weight as a filter and should be used
// on slower systems
//Mahony filter;
Madgwick filter;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//User Initialized Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Saturation Function Bound//
int satBound = 2;   

//Equivalent Control Gains//
float cRoll = 3;                    
float cPitch = cRoll;                   
float cYaw = 3.50;                     

//Discontinuous Control Gains//
float kRoll = 6;      
float kPitch = kRoll;         
float kYaw = 4.7466;

 /**
//Equivalent Control Gains//
float cRoll = 3.06;                    
float cPitch = 2.8;                   
float cYaw = 1.88;                     

//Discontinuous Control Gains//
float kRoll = 45.44;      
float kPitch = 41.75;         
float kYaw = 43.61;
**/

//Max Receiver Inputs//
float maxRoll = 164.0*deg2rad;
float maxPitch = 164.0*deg2rad;
float maxYaw = 70.0*deg2rad;

//Hardware Paramaters//
float gyroSense = 65.5/deg2rad;               // [LSB/(rad/s)] Gyro Sensitivity
float IRsense = 13.75*PI/30;                  // [rad/Vs] IR Sensor Sensitivity
float vMax = 555;                             // [rad/s] Maximum Motor Velocity
float dt = 0.004;                             // [s] Main Loop Period

///Quadcopter Physical Parameters//
float Kf = 0.0000158;                         // [N-s2] Aerodynamic Force Constant
float Km = 0.000000283;                          // [N-m-s2] Aerodynamic Moment Constant
float Ixx = 0.01548;                          // [kg/m2] Moment of Inertia Around x Axis
float Iyy = 0.01565;                          // [kg/m2] Moment of Inertia Around y Axis
float Izz = 0.03024;                          // [kg/m2] Moment of Inertia Around z Axis
float Jr = 0.00006;                           // [kg/m2] Rotor Inertia
float len = 0.17;                             // [m] Quadcopter Arm Length
float g = 9.81;                               // [m/s] Gravitational Acceleration
float mass = 0.98;                            // [kg] Quadcopter Total Mass

// PWM Settings
int pwmFreq = 250;
int pwmRes = 8;

// Total pulse length for esc
int escPulseTime = 4000;

int delayTimer = 3600;                        // Setup a delay timer in uS

/////////PINS////////////

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

//IR Pins
int inIR1 = 23;
int inIR2 = 22;
int inIR3 = 21;
int inIR4 = 17;

// Timing Variables for Pulse Width
unsigned long prev1 = 0;
volatile unsigned long R1 = 1500;
unsigned long prev2 = 0;
volatile unsigned long R2 = 1500;
unsigned long prev3 = 0;
volatile unsigned long R3 = 1500;
unsigned long prev4 = 0;
volatile unsigned long R4 = 1500;
unsigned long prev5 = 0;
volatile unsigned long R5 = 1500;
unsigned long prev6 = 0;
volatile unsigned long R6 = 1500;

int esc1PWM;
int esc2PWM;
int esc3PWM;
int esc4PWM;

elapsedMicros elapsedTime;
unsigned long escLoopTime = 0;
unsigned long timer_channel_1 = 0;

float gyroRollCal, gyroPitchCal, gyroYawCal, yawCal;

int w1, w2, w3, w4, wr;   // [rad/s] Rotor Velocities
float F1, F2, F3, F4;     // Fault Magnitude

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
float u1, u2, u3, u4;                           // Thrust Forces {Heave, Pitch, Roll, Yaw}
float u1f, u2f, u3f, u4f;                           // Fault Thrust Forces {Heave, Pitch, Roll, Yaw}
float w1Des = 1, w2Des = 1, w3Des = 1, w4Des = 1;          // Rotor Velocities {Front (ccw), Right (cw), Back (ccw), Left(cw)}
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

//Sliding Mode Controller//
float pitch, roll, yaw;             //Euler Angles
float pitchD1, rollD1, yawD1;       //Euler Angular Rates
float ePitch, eRoll, eYaw;          //Error
float ePitchD1, eRollD1, eYawD1;    //Error Derivative
float sPitch, sRoll, sYaw;          //Sliding Surfaces

//Misc//
float yawADD;           //Speed up loop
int start = 0;              //Motor start
int temp;               //IMU temp Variable

// Get maximum value for selected PWM resolution (100% Duty)
int pwmMax = 256;
// Initializing pulse for ESCs, 25% duty
int escInit = pwmMax/4;
boolean first_angle;
unsigned long loopTimer;
unsigned long timer1;
long printTimer;
unsigned long prevLoop = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  
  //Serial.begin(5000000);

  //Warning LED//
  pinMode(led,OUTPUT);
  digitalWrite(led,HIGH); //Setup has begun LED ON
  
  pinMode(inIR1,INPUT); 
  pinMode(inIR2,INPUT); 
  pinMode(inIR3,INPUT); 
  pinMode(inIR4,INPUT); 
  

 
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
{
  // Filter expects 70 samples per second
  // Based on a Bluefruit M0 Feather ... rate should be adjuted for other MCUs
  filter.begin(10);
}                    
   
                                              
//delay(60000);
  //Gyro Calibration//
  for(calcInt = 0; calcInt < 2000; calcInt++ )
  {
    gyro_signalen();
    gyroRollCal += gyroRoll;
    gyroPitchCal += gyroPitch;
    gyroYawCal += gyroYaw ;
    yawCal += yaw;
    delayMicroseconds(100);
  }
   //Average DC Offset//
   gyroRollCal = gyroRollCal/2000;
   gyroPitchCal = gyroPitchCal/2000;
   gyroYawCal = gyroYawCal/2000;

  //Setup ESC Pins//
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
  loopTimer = micros();
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  debug();                         

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

  //Read IMU//
  gyro_signalen();
    
  //Gyro Rate Filter [rad/s]//
  rollD1 = (rollD1 * 0.7) + (gyroRoll* 0.3);
  pitchD1 = (pitchD1 * 0.7) + (gyroPitch* 0.3);
  yawD1 = (yawD1 * 0.7) + (gyroYaw * 0.3);
       
  //Rotor Angular Velocities [rad/s]//
  w1=IRsense*analogRead(inIR1);       // Front Right (CCW)
  w2=IRsense*analogRead(inIR2);       // Back Right (CW)
  w3=IRsense*analogRead(inIR3);       // Back Left (CCW)
  w4=IRsense*analogRead(inIR4);       // Front Left (CW)

  //Minimum Angular Velocities [rad/s]//
  w1 = (w1 < 300) ? 0 : w1;           // Front Right (CCW)
  w2 = (w2 < 300) ? 0 : w2;           // Back Right (CW)
  w3 = (w3 < 300) ? 0 : w3;           // Back Left (CCW)
  w4 = (w4 < 300) ? 0 : w4;           // Front Left (CW)
  wr=w2-w1+w4-w3;                     // Relative Rotor Velocity

  //Fault Magnitude//
  F1 = 1 - w1 / w1Des;
  F2 = 1 - w2 / w2Des;
  F3 = 1 - w3 / w3Des;
  F4 = 1 - w4 / w4Des;

  //Fault Saturation//
  F1 = isnan(F1) ? 0 : (F1 < 0) ? 0 : (F1 > 0.4) ? 0.4 : F1;
  F2 = isnan(F2) ? 0 : (F2 < 0) ? 0 : (F2 > 0.4) ? 0.4 : F2;
  F3 = isnan(F3) ? 0 : (F3 < 0) ? 0 : (F3 > 0.4) ? 0.4 : F3;
  F4 = isnan(F4) ? 0 : (F4 < 0) ? 0 : (F4 > 0.4) ? 0.4 : F4;

  //Fault Forces//
  u1f = Kf*((F1*w1*w1) + (F2*w2*w2) + (F3*w3*w3) + (F4*w4*w4));
  u2f = Kf*len*((F3*w3*w3) - (F1*w1*w1));
  u3f = Kf*len*((F4*w4*w4) - (F2*w2*w2));
  u4f = Km*((F1*w1*w1) - (F2*w2*w2) + (F3*w3*w3) - (F4*w4*w4));

  //Desired Angular Acceleration [rad/s2]//
  pitchDesD2 = (pitchDesD1 - pitchDesD1old)/dt;
  rollDesD2 = (rollDesD1 - rollDesD1old)/dt;
  yawDesD2 = (yawDesD1 - yawDesD1old)/dt;

  //Desired Angular Velocity [rad/s]//
  pitchDesD1 = -(R2-1500.0)*maxPitch/500.0;
  rollDesD1 = (R1-1500.0)*maxRoll/500.0;
  yawDesD1 = (R4-1500.0)*maxYaw/500.0;

  //Transmitter Dead Band//
  pitchDesD1 = (abs(pitchDesD1) < 10.0*deg2rad) ? 0 : pitchDesD1;
  rollDesD1 = (abs(rollDesD1) < 10.0*deg2rad) ? 0 : rollDesD1;
  yawDesD1 = (abs(yawDesD1) < 10.0*deg2rad) ? 0 : yawDesD1;

  //Desired Angular Position [rad]//
  pitchDes=0;
  rollDes=0;
  yawDes+=yawDesD1*dt;
 
  //Error Calculations [rad]//
  ePitch = pitchDes - pitch;
  eRoll = rollDes - roll;
  eYaw =  yawDes - yaw;
  eYaw = 0;
  
  //Error Time Derivatives [rad/s]//
  ePitchD1 = pitchDesD1 - pitchD1;
  eRollD1 = rollDesD1 - rollD1;  
  eYawD1 = yawDesD1 - yawD1;

  //Sliding Surfaces//
  sPitch = cPitch*ePitch + ePitchD1;
  sRoll = cRoll*eRoll + eRollD1;
  sYaw = cYaw*eYaw + eYawD1;
  
  //Sliding Surface Saturation//
  sPitch = (sPitch > satBound) ? satBound : (sPitch < -satBound) ? -satBound : sPitch;
  sRoll = (sRoll > satBound) ? satBound : (sRoll < -satBound) ? -satBound : sRoll;
  sYaw = (sYaw > satBound) ? satBound : (sYaw < -satBound) ? -satBound : sYaw;

  //Thrust Inputs//
  throttle = R3;                                                                                        //We need the throttle signal as a base signal.
  throttle = (throttle > 1800.0) ? 1800.0 : throttle;                                                   // Need room to control
  u1 = 4.0*Kf*((throttle-1000.0)*vMax/1000.0)*((throttle-1000.0)*vMax/1000.0);                          // [N] Heave
  u2 = ((kPitch*sPitch) + (cPitch*ePitchD1) + pitchDesD2  /*(a2*rollD1*wr)*/ - (a1*rollD1*yawD1))/b1;   // [Nm] Pitch Torque
  u3 = ((kRoll*sRoll) + (cRoll*eRollD1) + rollDesD2 /*- (a4*pitchD1*wr)*/ - (a3*pitchD1*yawD1))/b2;     // [Nm] Roll Torque
  u4 = ((kYaw*sYaw) + (cYaw*eYawD1) + yawDesD2 - (a5*pitchD1*rollD1))/b3;                               // [Nm] Yaw Torque

  //Fault Forces//
  //u1 += u1f;
  //u2 += u2f;
  //u3 += u3f;
  //u4 += u4f;

  //Desired Rotor Angular Velocities [rad/s]//
  w1Des = sqrt(((u1-2*u2/len)/Kf + u4/Km)/4);   // Front Right (CCW)
  w2Des = sqrt(((u1-2*u3/len)/Kf - u4/Km)/4);   // Back Right (CW)
  w3Des = sqrt(((u1+2*u2/len)/Kf + u4/Km)/4);   // Back Left (CCW)
  w4Des = sqrt(((u1+2*u3/len)/Kf - u4/Km)/4);   // Front Left (CW)

  //Nan Check//
  w1Des = isnan(w1Des) ? 1 : w1Des;
  w2Des = isnan(w2Des) ? 1 : w2Des;
  w3Des = isnan(w3Des) ? 1 : w3Des;
  w4Des = isnan(w4Des) ? 1 : w4Des;

  //ESC Pulse Widths [us]//
  esc1 = w1Des*1000.0/vMax + 1000.0; // Front (CCW)
  esc2 = w2Des*1000.0/vMax + 1000.0; // Right (CW)
  esc3 = w3Des*1000.0/vMax + 1000.0; // Back (CCW)
  esc4 = w4Des*1000.0/vMax + 1000.0; // Left (CW)
  
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

  //Write ESC PWM//
  analogWrite(escOut1,esc1PWM);
  analogWrite(escOut2,esc2PWM);
  analogWrite(escOut3,esc3PWM);
  analogWrite(escOut4,esc4PWM);


  while(timer1 < 4000) timer1 = micros() - loopTimer;                                     //We wait until 4000us are passed.
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

  // Get new data samples
  gyro.getEvent(&gyro_event);
#if AHRS_VARIANT == NXP_FXOS8700_FXAS21002
  accelmag.getEvent(&accel_event, &mag_event);
#else
  accel.getEvent(&accel_event);
  mag.getEvent(&mag_event);
#endif

  // Apply mag offset compensation (base values in uTesla)
  float magx = mag_event.magnetic.x - mag_offsets[0];
  float magy = mag_event.magnetic.y - mag_offsets[1];
  float magz = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = magx * mag_softiron_matrix[0][0] + magy * mag_softiron_matrix[0][1] + magz * mag_softiron_matrix[0][2];
  float my = magx * mag_softiron_matrix[1][0] + magy * mag_softiron_matrix[1][1] + magz * mag_softiron_matrix[1][2];
  float mz = magx * mag_softiron_matrix[2][0] + magy * mag_softiron_matrix[2][1] + magz * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  gyroPitch = gyro_event.gyro.y - gyroPitchCal;
  gyroRoll = gyro_event.gyro.x - gyroRollCal;
  gyroYaw = gyro_event.gyro.z - gyroYawCal;

  // Update the filter
  filter.update(gyroRoll/deg2rad, gyroPitch/deg2rad, gyroYaw/deg2rad,
                accel_event.acceleration.x, accel_event.acceleration.y, accel_event.acceleration.z,
                mx, my, mz);

  pitch = filter.getPitch()*deg2rad;
  roll = filter.getRoll()*deg2rad;
  yaw = filter.getYaw()*deg2rad - yawCal;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Debugging Function
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void debug(){

/** Debug Section

  //Receiver Signals//
  Serial.println("R1");
  Serial.println(R1);
  Serial.println("R2");
  Serial.println(R2);
  Serial.println("R3");
  Serial.println(R3);
  Serial.println("R4");
  Serial.println(R4);

  //Gyro Angles//
  Serial.println("ROLL");
  Serial.println(roll/deg2rad);
  Serial.println("PITCH");
  Serial.println(pitch/deg2rad);
  Serial.println("YAW");
  Serial.println(yaw/deg2rad);

  //ESC//
  Serial.println("ESC1");
  Serial.println(esc1);
  Serial.println("ESC2");
  Serial.println(esc2);
  Serial.println("ESC3");
  Serial.println(esc3);
  Serial.println("ESC4");
  Serial.println(esc4);

  //w//
  Serial.println("w1Des");
  Serial.println(w1Des);
  Serial.println("w2Des");
  Serial.println(w2Des);
  Serial.println("w3Des");
  Serial.println(w3Des);
  Serial.println("w4Des");
  Serial.println(w4Des);


  Serial.println("u1");
  Serial.println(u1);
  Serial.println("u2");
  Serial.println(u2);
  Serial.println("u3");
  Serial.println(u3);
  Serial.println("u4");
  Serial.println(u4);

  
  Serial.println("ePitchD1");
  Serial.println(ePitchD1);
  Serial.println("eRollD1");
  Serial.println(eRollD1);
  Serial.println("eYawD1");
  Serial.println(eYawD1);

   

  Serial.println("ePitchD1");
  Serial.println(ePitchD1);
  Serial.println("eRollD1");
  Serial.println(eRollD1);
  Serial.println("eYawD1");
  Serial.println(eYawD1);

    Serial.println("F1");
  Serial.println(F1);
  Serial.println("F2");
  Serial.println(F2);
  Serial.println("F3");
  Serial.println(F3);
  Serial.println("F4");
  Serial.println(F4);
  

   //ESC//
  Serial.println("ESC1");
  Serial.println(esc1);
  Serial.println("ESC2");
  Serial.println(esc2);
  Serial.println("ESC3");
  Serial.println(esc3);
  Serial.println("ESC4");
  Serial.println(esc4);
  Serial.println(roll/deg2rad);
  



  Serial.println("ROLL");
  Serial.println(roll/deg2rad);
  Serial.println("GYRO ROLL");
  Serial.println(gyroRoll/deg2rad);
  Serial.println("PITCH");
  Serial.println(pitch/deg2rad);
  Serial.println("YAW");
  Serial.println(yaw/deg2rad); 

    Serial.println("w1");
  Serial.println(w1);
  Serial.println("w2");
  Serial.println(w2);
  Serial.println("w3");
  Serial.println(w3);
  Serial.println("w4");
  Serial.println(w4);

  Serial.println("w1Des");
  Serial.println(w1Des);
  Serial.println("w2Des");
  Serial.println(w2Des);
  Serial.println("w3Des");
  Serial.println(w3Des);
  Serial.println("w4Des");
  Serial.println(w4Des);



  Serial.println("u1");
  Serial.println(u1);
  Serial.println("u2");
  Serial.println("u3");
  Serial.println(u3);
  Serial.println("u4");
  Serial.println(u4);

  
  //Gyro Angles//
  Serial.println("ROLL");
  Serial.println(roll/deg2rad);
  Serial.println("PITCH");
  Serial.println(pitch/deg2rad);
  Serial.println("YAW");
  Serial.println(yaw/deg2rad);

     
     **/ 


  Serial.println("ESC1");
  Serial.println(esc1);
  Serial.println("ESC2");
  Serial.println(esc2);
  Serial.println("ESC3");
  Serial.println(esc3);
  Serial.println("ESC4");
  Serial.println(esc4);


     }
