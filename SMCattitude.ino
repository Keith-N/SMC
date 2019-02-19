///////////////////////////////////////////////////////////////////////////////////////
//Sliding Mode Quadcopter Flight Controller V1 (SMCattitude.ino)
///////////////////////////////////////////////////////////////////////////////////////
//Controller designed by Sital Khatiwada. Controller programmed and implemented by 
//Aaron Cantara and Michael Goulet at the University of New Hampshire 2018.
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
#include <EEPROM.h>           // Reading EEPROM Calibration Information

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
float accSense = 57.296*deg2rad;              // [(rad/s)/LSB] Accelerometer Sensitivity
float dt = 0.004;                             // [s] Main Loop Period (1/250 [Hz])
float vMax = 555;                             // [rad/s] Maximum Motor Velocity
byte diode = 65;                              // Diode Voltage Compensation

///Quadcopter Physical Parameters//
float Kf = 0.0000158;                         // [N-s2] Aerodynamic Force Constant
float Km = 0.00000029;                        // [N-m-s2] Aerodynamic Moment Constant
float Ixx = 0.01548;                          // [kg/m2] Moment of Inertia Around x Axis
float Iyy = 0.01565;                          // [kg/m2] Moment of Inertia Around y Axis
float Izz = 0.03024;                          // [kg/m2] Moment of Inertia Around z Axis
float Jr = 0.00006;                           // [kg/m2] Rotor Inertia
float len = 0.17;                             // [m] Quadcopter Arm Length
float g = 9.81;                               // [m/s] Gravitational Acceleration
float mass = 0.98;                            // [kg] Quadcopter Total Mass

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
unsigned long tCH1, tCH2, tCH3, tCH4;
unsigned long t1, t2, t3, t4, currentTime;
unsigned long loopTimer, escTimer, escLoopTimer;

//Receiver//
volatile int R1, R2, R3, R4;                            // Receiver Pulses {Pitch, Roll, Heave, Yaw}
float pitchDes, rollDes, yawDes;                        // Desired Euler Angles
float pitchDesD1, rollDesD1, yawDesD1;                  // Desired Euler Angular Rates
float pitchDesD1old, rollDesD1old, yawDesD1old;         // Last Loop Rate
float pitchDesD2, rollDesD2, yawDesD2;                  // Rate Derivative
int receiverIN[5];
byte lastCH1, lastCH2, lastCH3, lastCH4;
float throttle;

//Gyroscope//
float gyroPitch, gyroRoll, gyroYaw;   //Euler Angles
int gyroAxis[4], calcInt, gAdd;
boolean gyroAnglesSet;
double gyroAxisCal[4];

//Accelerometer//
int accAxis[4];
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
int start;              //Motor start
int batV;               //Battery Voltage
int temp;               //IMU temp Variable
byte eeData[36];        //EEprom Data
byte highByte, lowByte;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){
  //Serial.begin(57600);
  
  //Initial General Setup Tasks//
  for(start = 0; start <= 35; start++)eeData[start] = EEPROM.read(start); //Import EEPROM Data
  start = 0;                                                                //Set start back to zero.
  gAdd = eeData[32];                                                        //Gyro Address
  Wire.begin();                                                             //Start the I2C as master.
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz
  DDRD |= B11110000;                                                        //Set digital ports 4, 5, 6 and 7 as outputs.
  DDRB |= B00110000;                                                        //Set digital ports 12 and 13 as outputs.
  digitalWrite(13,HIGH);                                                    //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eeData[33] != 'J' || eeData[34] != 'M' || eeData[35] != 'B')delay(10);

  if(eeData[31] == 2 || eeData[31] == 3)delay(10); //If setup is completed without MPU-6050 stop the flight controller program 

  //Gyro DC Offset Calibration//
  gyroRegisters();                                                          //Set the specific gyro registers.
  for (calcInt = 0; calcInt < 1250 ; calcInt ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }
  for (calcInt = 0; calcInt < 2000 ; calcInt ++){                           //Take 2000 readings for calibration.
    if(calcInt % 15 == 0)digitalWrite(13, !digitalRead(13));                //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyroAxisCal[1] += gyroAxis[1];                                       //Ad roll value to gyroRoll_cal.
    gyroAxisCal[2] += gyroAxis[2];                                       //Ad pitch value to gyroPitch_cal.
    gyroAxisCal[3] += gyroAxis[3];                                       //Ad yaw value to gyroYaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  gyroAxisCal[1] /= 2000;                                                 //Average roll offset
  gyroAxisCal[2] /= 2000;                                                 //Average pitch offset
  gyroAxisCal[3] /= 2000;                                                 //Average yaw offset

  //Enable Pin Change Interrupts//
  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Pin 8
  PCMSK0 |= (1 << PCINT1);                                                  //Pin 9
  PCMSK0 |= (1 << PCINT2);                                                  //Pin 10
  PCMSK0 |= (1 << PCINT3);                                                  //Pin 11

  //Receiver On & Throttle Low?//
  while(R3 < 990 || R3 > 1020 || R4 < 1400){
    R3 = convertReceiver(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    R4 = convertReceiver(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
    start ++;                                                               //While waiting increment start whith every loop.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while waiting for the receiver inputs.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
    if(start == 125){                                                       //Every 125 loops (500ms).
      digitalWrite(13, !digitalRead(13));                                   //Change the led status.
      start = 0;                                                            //Start again at 0.
    }
  }

  //Final General Setup Tasks//
  start = 0;                                                                //Set start back to 0.
  batV = (analogRead(0) + 65) * 1.2317;
  loopTimer = micros();                                                    //Set the timer for the next loop.
  digitalWrite(13,LOW);                                                    //Turn off startup indicator LED
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
    
  //Gyro Rate Filter//
  rollD1 = (rollD1 * 0.7) + (gyroRoll* 0.3);   //[rad/s]
  pitchD1 = (pitchD1 * 0.7) + (gyroPitch* 0.3); //[rad/s]
  yawD1 = (yawD1 * 0.7) + (gyroYaw * 0.3); //[rad/s] 
  
  //Gyro Angles [rad]//
  pitch += gyroPitch*dt;
  roll += gyroRoll*dt;                                      
  yaw += gyroYaw*dt;                                       
  
  //Yawing Angle Adjustment//
  yawADD=sin(gyroYaw*dt);
  pitch -= roll * yawADD;                  //If the IMU has yawed transfer the roll angle to the pitch angle.
  roll += pitch * yawADD;                  //If the IMU has yawed transfer the pitch angle to the roll angle.

  //Accelerometer angle calculations
  accA = sqrt((accX*accX)+(accY*accY)+(accZ*accZ));       //Calculate the total accelerometer vector.
  
  if(abs(accY) < accA){                                        //Prevent the asin function to produce a NaN
    accPitch = asin((float)accY/accA)*accSense;          //Calculate the pitch angle.
  }
  if(abs(accX) < accA){                                        //Prevent the asin function to produce a NaN
    accRoll = asin((float)accX/accA)*-accSense;          //Calculate the roll angle.
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  accPitch += 1.785060514*PI/180.0;                                                   //Accelerometer calibration value for pitch.
  accRoll -= 0;                                                    //Accelerometer calibration value for roll.

  //Complementary Filter//
  pitch = pitch * 0.9996 + accPitch * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  roll = roll * 0.9996 + accRoll * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

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

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  batV = batV * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(batV < 1000 && batV > 600)digitalWrite(13, HIGH);


  throttle = R3;                                      //We need the throttle signal as a base signal.

  if (start == 2){                                                          //The motors are started.
    if (batV < 1240 && batV > 800){                   //Is the battery connected?
      //Voltage Compensation//
      esc1 += esc1 * ((1240 - batV)/(float)3500);            
      esc2 += esc2 * ((1240 - batV)/(float)3500);           
      esc3 += esc3 * ((1240 - batV)/(float)3500);              
      esc4 += esc4 * ((1240 - batV)/(float)3500);             
    } 
    //Keep Motors Running//
    if (esc1 < 1100) esc1 = 1100;                                       
    if (esc2 < 1100) esc2 = 1100;                                     
    if (esc3 < 1100) esc3 = 1100;                                        
    if (esc4 < 1100) esc4 = 1100;                                         

    //Max Pulse//
    if(esc1 > 2000)esc1 = 2000;                                        
    if(esc2 > 2000)esc2 = 2000;                                        
    if(esc3 > 2000)esc3 = 2000;                                        
    if(esc4 > 2000)esc4 = 2000;                                      
  }

  //Min Pulse//
  else{
    esc1 = 1000;                                                 
    esc2 = 1000;                                                      
    esc3 = 1000;                                                   
    esc4 = 1000;                                       
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating ESC Pulses
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  if(micros() - loopTimer > 4050)digitalWrite(13, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loopTimer < 4000);                                      //We wait until 4000us are passed.
  loopTimer = micros();                                                    //Set the timer for the next loop.

  //ESC Falling Edge Calculations//
  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  tCH1 = esc1 + loopTimer;                                  
  tCH2 = esc2 + loopTimer;                                  
  tCH3 = esc3 + loopTimer;                                     
  tCH4 = esc4 + loopTimer;                                    
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  gyro_signalen();

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    escLoopTimer = micros();                                              //Read the current time.
    if(tCH1 <= escLoopTimer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(tCH2 <= escLoopTimer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(tCH3 <= escLoopTimer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(tCH4 <= escLoopTimer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reading Receiver Signals 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(PCINT0_vect){
  currentTime = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(lastCH1 == 0){                                                //Input 8 changed from 0 to 1.
      lastCH1 = 1;                                                   //Remember current input state.
      t1 = currentTime;                                               //Set t1 to currentTime.
    }
  }
  else if(lastCH1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    lastCH1 = 0;                                                     //Remember current input state.
    receiverIN[1] = currentTime - t1;                             //Channel 1 is currentTime - t1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(lastCH2 == 0){                                                //Input 9 changed from 0 to 1.
      lastCH2 = 1;                                                   //Remember current input state.
      t2 = currentTime;                                               //Set t2 to currentTime.
    }
  }
  else if(lastCH2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    lastCH2 = 0;                                                     //Remember current input state.
    receiverIN[2] = currentTime - t2;                             //Channel 2 is currentTime - t2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(lastCH3 == 0){                                                //Input 10 changed from 0 to 1.
      lastCH3 = 1;                                                   //Remember current input state.
      t3 = currentTime;                                               //Set t3 to currentTime.
    }
  }
  else if(lastCH3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    lastCH3 = 0;                                                     //Remember current input state.
    receiverIN[3] = currentTime - t3;                             //Channel 3 is currentTime - t3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(lastCH4 == 0){                                                //Input 11 changed from 0 to 1.
      lastCH4 = 1;                                                   //Remember current input state.
      t4 = currentTime;                                               //Set t4 to currentTime.
    }
  }
  else if(lastCH4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    lastCH4 = 0;                                                     //Remember current input state.
    receiverIN[4] = currentTime - t4;                             //Channel 4 is currentTime - t4.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reading Gyroscope and Accelerometer Angles
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  //Read the MPU-6050
  if(eeData[31] == 1){
    Wire.beginTransmission(gAdd);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gAdd,14);                                      //Request 14 bytes from the gyro.
    
    R1 = convertReceiver(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    R2 = convertReceiver(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    R3 = convertReceiver(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    R4 = convertReceiver(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    accAxis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the accX variable.
    accAxis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the accY variable.
    accAxis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the accZ variable.
    temp = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temp variable.
    gyroAxis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyroAxis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyroAxis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }

  if(calcInt == 2000){
    gyroAxis[1] -= gyroAxisCal[1];                                       //Only compensate after the calibration.
    gyroAxis[2] -= gyroAxisCal[2];                                       //Only compensate after the calibration.
    gyroAxis[3] -= gyroAxisCal[3];                                       //Only compensate after the calibration.
  }
  gyroRoll = gyroAxis[eeData[28] & 0b00000011]/gyroSense;                      //Set gyroRoll to the correct axis that was stored in the EEPROM.
  if(eeData[28] & 0b10000000)gyroRoll *= -1;                          //Invert gyroRoll if the MSB of EEPROM bit 28 is set.
  gyroPitch = gyroAxis[eeData[29] & 0b00000011]/gyroSense;                     //Set gyroPitch to the correct axis that was stored in the EEPROM.
  if(eeData[29] & 0b10000000)gyroPitch *= -1;                         //Invert gyroPitch if the MSB of EEPROM bit 29 is set.
  gyroYaw = gyroAxis[eeData[30] & 0b00000011]/gyroSense;                       //Set gyroYaw to the correct axis that was stored in the EEPROM.
  if(eeData[30] & 0b10000000)gyroYaw *= -1;                           //Invert gyroYaw if the MSB of EEPROM bit 30 is set.

  accX = accAxis[eeData[29] & 0b00000011];                           //Set accX to the correct axis that was stored in the EEPROM.
  if(eeData[29] & 0b10000000)accX *= -1;                              //Invert accX if the MSB of EEPROM bit 29 is set.
  accY = accAxis[eeData[28] & 0b00000011];                           //Set accY to the correct axis that was stored in the EEPROM.
  if(eeData[28] & 0b10000000)accY *= -1;                              //Invert accY if the MSB of EEPROM bit 28 is set.
  accZ = accAxis[eeData[30] & 0b00000011];                           //Set accZ to the correct axis that was stored in the EEPROM.
  if(eeData[30] & 0b10000000)accZ *= -1;                              //Invert accZ if the MSB of EEPROM bit 30 is set.
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
  pitchDesD1 = (R2-1500)*maxPitch/500;
  rollDesD1 = (R1-1500)*maxRoll/500;
  yawDesD1 = (R4-1500)*maxYaw/500;

  //Transmitter Dead Band//
  pitchDesD1 = (abs(pitchDesD1) < 5*deg2rad) ? 0 : pitchDesD1;
  rollDesD1 = (abs(rollDesD1) < 5*deg2rad) ? 0 : rollDesD1;
  yawDesD1 = (abs(yawDesD1) < 5*deg2rad) ? 0 : yawDesD1;

  //Desired Angular Position [rad]//
  pitchDes=0;
  rollDes=0;
  yawDes+=yawDesD1*dt;
 
  //Error Calculations [rad]//
  ePitch = pitchDes - pitch;
  eRoll = rollDes - roll;
  eYaw =  yawDes - yaw;
  
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
  throttle = (throttle > 1800) ? 1800 : throttle; // Need room to control
  u1 = 4.0*Kf*((throttle-1000)*vMax/1000.0)*((throttle-1000)*vMax/1000.0); // [N] Heave
  u2 = ((kPitch*sPitch) + (cPitch*ePitchD1) + pitchDesD2 - (a1*rollD1*yawD1))/b1; // [Nm] Pitch Torque
  u3 = ((kRoll*sRoll) + (cRoll*eRollD1) + rollDesD2 - (a3*pitchD1*yawD1))/b2; // [Nm] Roll Torque
  u4 = ((kYaw*sYaw) + (cYaw*eYawD1) + yawDesD2 - (a5*pitchD1*rollD1))/b3; // [Nm] Yaw Torque

  //Rotor Velocities [rad/s]//
  w1 = sqrt((u1+2*u2)/Kf + u4/Km)/2; // Front (CCW)
  w2 = sqrt((u1-2*u3)/Kf - u4/Km)/2; // Right (CW)
  w3 = sqrt((u1-2*u2)/Kf + u4/Km)/2; // Back (CCW)
  w4 = sqrt((u1+2*u3)/Kf - u4/Km)/2; // Left (CW)

  //Remove Negative Values//
  w1 = (w1 < 0) ? 0 : w1;
  w2 = (w2 < 0) ? 0 : w2;
  w3 = (w3 < 0) ? 0 : w3;
  w4 = (w4 < 0) ? 0 : w4;
  
  //ESC Pulse Widths [us]//
  esc1 = w1*1000.0/vMax + 1000.0; // Front (CCW)
  esc2 = w2*1000.0/vMax + 1000.0; // Right (CW)
  esc3 = w3*1000.0/vMax + 1000.0; // Back (CCW)
  esc4 = w4*1000.0/vMax + 1000.0; // Left (CW)

  //Serial.println(pitch/deg2rad);
  
  //Save Last Loop [rad/s]//
  rollDesD1old=rollDesD1;
  pitchDesD1old=pitchDesD1;
  yawDesD1old=yawDesD1;

 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Convert Receiver Signals
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This part uses data stored in the EEPROM to convert the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.

int convertReceiver(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeData[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeData[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiverIN[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeData[channel * 2 + 15] << 8) | eeData[channel * 2 + 14];        //Store the low value for the specific receiver input channel
  center = (eeData[channel * 2 - 1] << 8) | eeData[channel * 2 - 2];       //Store the center value for the specific receiver input channel
  high = (eeData[channel * 2 + 7] << 8) | eeData[channel * 2 + 6];         //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                    //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void gyroRegisters(){
  //Setup the MPU-6050
  if(eeData[31] == 1){
    Wire.beginTransmission(gAdd);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gAdd);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gAdd);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gAdd);                                              //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gAdd, 1);                                                 //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      digitalWrite(12,HIGH);                                                   //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }

    Wire.beginTransmission(gAdd);                                              //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

  }  
}
