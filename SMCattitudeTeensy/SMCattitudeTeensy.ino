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

///////////////////////////////////////CHANGE LOG//////////////////////////////////////
// (3/1/2019)/////////
// - Gyro register function changed to match Teensy Code
// - Gyro Signal function changed to match Teensy Code
// - Interrputs added
// - Added int led = 13
// - Added Configuration\ESC variables
// - Made changes to Setup to match Keith's Flight Controller Code
// - Buad Rate for Teensy code set at 115200, compared to Arduino's 57600
// (3/2/2019)/////////
// - Added various sections from Keith's Flight Controller Code
// - Removed old methods for calcuating esc pulses and reading receiver signals
// - Changed some integers to floats to solve some large value errors
// - Some slight style changes throughout
// - Changed the names of a few variables to match old SMC naming scheme
// - Changed eYaw to 0 to try and counter yaw's drift
// - Put functions that only run once into setup except the gyro registers function
// - Converted Keith's values in degrees to radians
// - Negated inverted receiver channels
// - Changed loop time used in smc() from 4000 [us] to measured loop time.
// - Removed some variables that are no longer used.
// - Increased Transmitter dead band from 10 [us] to 20 [us]
// - Added if statements to handle nan values 
// - Removed portion of code that converts receiver signals
// - Similified esc max and min output if statement
// - Removed battery volatge compensation because hardware is not ready yet.
//////////////////////////////////////////////////////////////////////////////////////

//Additional Libraries//
#include <Arduino.h>
#include <Wire.h>             // Gyroscope Communication

//Unit Conversions//
float deg2rad = PI/180;       // [rad/deg]

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

//Max Receiver Inputs//
float maxRoll = 164.0*deg2rad;
float maxPitch = 164.0*deg2rad;
float maxYaw = 70.0*deg2rad;

//Hardware Paramaters//
float gyroSense = 65.5/deg2rad;               // [LSB/(rad/s)] Gyro Sensitivity
float vMax = 555;                             // [rad/s] Maximum Motor Velocity
float dt;                                     // [s] Main Loop Period

///Quadcopter Physical Parameters//
float Kf = 0.0000158;                         // [N-s2] Aerodynamic Force Constant
float Km = 0.00000029;                          // [N-m-s2] Aerodynamic Moment Constant
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
bool debug = true;

// Total pulse length for esc
int escPulseTime = 4000;

int pidDiv = 3;                            // Adjust scaling for rad/s response (500-8)/pidDiv
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

// Receiver Pulses {Pitch, Roll, Heave, Yaw}

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

int16_t accX, accY, accZ, GyX, GyY, GyZ;
int16_t accX_Cal,accY_Cal,accZ_Cal;
int16_t GyX_Cal = 0;
int16_t GyY_Cal = 0;
int16_t GyZ_Cal = 0;

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
int calcInt, gAdd = 0x68;
boolean gyroAnglesSet;

//Accelerometer//
float accRoll, accPitch;        //Euler Angles
long accX, accY, accZ, accA;    //Accelerations {x, y, z, Total Vector}

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
const int MPU_addr=0x68;  // I2C address of the MPU-6050
boolean first_angle;
unsigned long loopTimer = 1563;
unsigned long timer1;
long printTimer;
unsigned long prevLoop = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  
  Serial.begin(5000000);

  //Warning LED//
  pinMode(led,OUTPUT); 
  digitalWrite(led,HIGH); //Setup has begun LED ON
                     
  //Setup I2C Gyro Communication
  Wire.begin();
  Wire.setClock(5000000);  
                                                 
  //Set Gyro Registers//
  gyroRegisters();

  //Gyro Calibration//
  for(calcInt =0; calcInt<2000; calcInt++ )
  {
    gyro_signalen();
    GyX_Cal = GyX+GyX_Cal;
    GyY_Cal = GyY+GyY_Cal;
    GyZ_Cal = GyZ+GyZ_Cal;
    delay(100);
  }
   //Average DC Offset//
   GyX_Cal = GyX_Cal/2000;
   GyY_Cal = GyY_Cal/2000;
   GyZ_Cal = GyZ_Cal/2000;


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
  
  loopTimer = micros();                                                    // Set the timer for the next loop.
  digitalWrite(led,LOW); //Setup Complete LED
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
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
  Serial.println("w1");
  Serial.println(w1);
  Serial.println("w2");
  Serial.println(w2);
  Serial.println("w3");
  Serial.println(w3);
  Serial.println("w4");
  Serial.println(w4);


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
  
   **/ 
   
  //Setup timer for IMU loop calcs and get current measurement
  timer1 = elapsedTime;

  gyro_signalen();
    
  //Gyro Rate Filter [rad/s]//
  rollD1 = (rollD1 * 0.7) + (gyroRoll* 0.3);
  pitchD1 = (pitchD1 * 0.7) + (gyroPitch* 0.3);
  yawD1 = (yawD1 * 0.7) + (gyroYaw * 0.3);

 //Gyro_angle calculations

  dt = loopTimer * 0.000001;
  //Gyro Angles [rad]//
  pitch += gyroPitch * dt;       
  roll += gyroRoll * dt;
  yaw += gyroYaw * dt;        

  //Yawing Angle Adjustment//
  pitch -= roll * sin(gyroYaw * dt);  //If the IMU has yawed transfer the roll angle to the pitch angle.                       
  roll += pitch * sin(gyroYaw * dt);  //If the IMU has yawed transfer the pitch angle to the roll angle.
                                   
  //Accelerometer angle calculations
  accA = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));       //Calculate the total accelerometer vector.
  
  if(abs(accY) < accA){                                        //Prevent the asin function to produce a NaN
    accPitch = asin((float)accY/accA);          //Calculate the pitch angle.
  }
  if(abs(accX) < accA){                                        //Prevent the asin function to produce a NaN
    accRoll = -asin((float)accX/accA);          //Calculate the roll angle.
  }

  
  if(!first_angle){
    pitch = accPitch;                                                 //Set the pitch angle to the accelerometer angle.
    roll = accRoll;                                                   //Set the roll angle to the accelerometer angle.
    first_angle = true;
 }
 else{
   pitch = pitch * 0.998 + accPitch * 0.002;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
   roll = roll * 0.998 + accRoll * 0.002;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
 }

  //Start Motors//
  if(R3 < 1050 && R4 < 1050)start = 1; //For starting the motors: throttle low and yaw left (step 1).
  if(start == 1 && R3 < 1050 && R4 > 1450){ //When yaw stick is back in the center position start the motors (step 2).
    start = 2;

    pitch = accPitch;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    roll = accRoll;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyroAnglesSet = true;                                                 //Set the IMU started flag.

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

  smc();                                                            //PID inputs are known. So we can calculate the pid output.  

  throttle = R3;                                      //We need the throttle signal as a base signal.

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

  delayMicroseconds(delayTimer);
  loopTimer = elapsedTime  - timer1;

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
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  accX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  accY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  accZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  gyroRoll = (GyY-GyY_Cal)/gyroSense;
  gyroPitch = (GyX-GyX_Cal)/gyroSense;
  gyroYaw = (GyZ-GyZ_Cal)/gyroSense;

}



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Sliding Mode Controller
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void smc(){

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
  throttle = (throttle > 1800.0) ? 1800.0 : throttle; // Need room to control
  u1 = 4.0*Kf*((throttle-1000.0)*vMax/1000.0)*((throttle-1000.0)*vMax/1000.0); // [N] Heave
  u2 = ((kPitch*sPitch) + (cPitch*ePitchD1) + pitchDesD2 - (a1*rollD1*yawD1))/b1; // [Nm] Pitch Torque
  u3 = ((kRoll*sRoll) + (cRoll*eRollD1) + rollDesD2 - (a3*pitchD1*yawD1))/b2; // [Nm] Roll Torque
  u4 = ((kYaw*sYaw) + (cYaw*eYawD1) + yawDesD2 - (a5*pitchD1*rollD1))/b3; // [Nm] Yaw Torque

  //Rotor Velocities [rad/s]//
  w1 = sqrt((u1+2*u2)/Kf - u4/Km)/2; // Front (CCW)
  w2 = sqrt((u1-2*u3)/Kf + u4/Km)/2; // Right (CW)
  w3 = sqrt((u1-2*u2)/Kf - u4/Km)/2; // Back (CCW)
  w4 = sqrt((u1+2*u3)/Kf + u4/Km)/2; // Left (CW)

  if(isnan(w1)){
    w1 = 0;
  }
    if(isnan(w2)){
    w2 = 0;
  }
    if(isnan(w3)){
    w3 = 0;
  }
    if(isnan(w4)){
    w4 = 0;
  }
  
  //ESC Pulse Widths [us]//
  esc1 = w1*1000.0/vMax + 1000.0; // Front (CCW)
  esc2 = w2*1000.0/vMax + 1000.0; // Right (CW)
  esc3 = w3*1000.0/vMax + 1000.0; // Back (CCW)
  esc4 = w4*1000.0/vMax + 1000.0; // Left (CW)
  
  //Save Last Loop [rad/s]//
  rollDesD1old=rollDesD1;
  pitchDesD1old=pitchDesD1;
  yawDesD1old=yawDesD1;
 
}

void gyroRegisters(){
  //Setup the MPU-6050
    Wire.beginTransmission(gAdd); //Start communication with the address found during search.
    Wire.write(0x6B); //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00); //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission(); //End the transmission with the gyro.

    Wire.beginTransmission(gAdd); //Start communication with the address found during search.
    Wire.write(0x1B); //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0b00001000); //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission(); //End the transmission with the gyro

    Wire.beginTransmission(gAdd); //Start communication with the address found during search.
    Wire.write(0x1C); //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0b00010000); //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission(); //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gAdd); //Start communication with the address found during search
    Wire.write(0x1B); //Start reading @ register 0x1B
    Wire.endTransmission(); //End the transmission
    Wire.requestFrom(gAdd, 1); //Request 1 bytes from the gyro
    while(Wire.available() < 1); //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){ //Check if the value is 0x08
      digitalWrite(13,HIGH); //Turn on the warning led
      while(1)delay(10); //Stay in this loop for ever
    }

   Wire.beginTransmission(gAdd); //Start communication with the address found during search
   Wire.write(0x1A); //We want to write to the CONFIG register (1A hex)
   Wire.write(0b00000011); //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
   Wire.endTransmission(); //End the transmission with the gyro

   Wire.beginTransmission(gAdd); //Start communication with the address found during search
   Wire.write(0x24);  // I2C Control
   Wire.write(0b00001001);  // 500kHz
   Wire.endTransmission(); //End the transmission with the gyro
}
