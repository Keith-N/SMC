///////////////////////////////////////////////////////////////////////////////////////
//Sliding Mode Quadcopter Flight Controller V1 (SMC_attitude.ino)
///////////////////////////////////////////////////////////////////////////////////////
//Controller designed by Sital Khatiwada. Controller Programmed and implemented by 
//Aaron Cantara and Michael Goulet at the University of New Hampshire 2018.
///////////////////////////////////////////////////////////////////////////////////////
//Description: 
///////////////////////////////////////////////////////////////////////////////////////
//Non linear sliding mode quadcopter controller. Version 1 is designed to control
//the attitude only using the pitch, roll, and yaw readings from the gyroscope and
//accelerometer.
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

//Auto-Leveling//
boolean autoLevel = true;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//User Initialized Global Variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Auto Level Gain//
float k_AL =0.01;

//Equivalent Control Gain//
float c_roll = 3.34;                    
float c_pitch = 2.23;                   
float c_yaw = 3.50;                     

//Discontinuous Control Gain//
float k_roll = 34.2;      
float k_pitch = 43.6;         
float k_yaw = 4.7466;

//Aerodynamic Constants//
float Kf = 0.0000158;                         // [N-s2] Force Constant
float Km = 0.00000029;                        // [N-m-s2] Moment Constant

///Quadcopter Physical Parameters//
float Ixx = 0.01548;                          // [kg/m2] Moment of Inertia Around x Axis
float Iyy = 0.01565;                          // [kg/m2] Moment of Inertia Around y Axis
float Izz = 0.03024;                          // [kg/m2] Moment of Inertia Around z Axis
float Jr = 0.00006;                           // [kg/m2] Rotor Inertia
float len = 0.17;                             // [m] Quadcopter Arm Length
float g = 9.81;                               // [m/s] Gravitational Acceleration
float mass = 0.98;                            // [kg] Quadcopter Total Mass

//Hardware Paramaters//
float gyroSense = 65.5/deg2rad;               // [LSB/(rad/s)] Gyro Sensitivity
float accSense = 57.296*deg2rad;              // [(rad/s)/LSB] Accelerometer Sensitivity
float dt = 0.004;                             // [s] Main Loop Period (1/250 [Hz])
float vmax = 555;                             // [rad/s] Maximum Motor Velocity
byte diode = 65;                              // Diode Voltage Compensation

//Max Receiver Inputs//
float max_roll = 30.0*deg2rad;
float max_pitch = 30.0*deg2rad;
float max_yaw = 40.0*deg2rad;

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
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
float loop_timer_1, time_;

//Receiver//
volatile int R1, R2, R3, R4;
float throttle;
float pitch_des, roll_des, yaw_des;
float pitchD1_des, rollD1_des, yawD1_des;
float pitchD1_des_last, rollD1_des_last, yawD1_des_last;   // Last Loop Rate
float pitchD2_des, rollD2_des, yawD2_des;                  // Rate Derivative
int receiver_input[5];
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;

//Gyroscope//
float pitch, roll, yaw;
float pitchD1, rollD1, yawD1;
int gyro_axis[4], cal_int, gyro_address;
boolean gyro_angles_set;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];

//Accelerometer//
long acc_x, acc_y, acc_z, acc_total_vector;
int acc_axis[4];
float roll_acc, pitch_acc;

//Sliding Mode Controller//
float s_pitch, s_roll, s_yaw;
float pitch_e, roll_e, yaw_e;
float pitchD1_e, rollD1_e, yawD1_e;

//Speed Up Main Loop//
float vmax_con=vmax*vmax/1000000.0;
float yawADD;

//Misc//
int start;
int battery_voltage;
int temperature;
byte eeprom_data[36];
byte highByte, lowByte;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Setup routine
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup(){
  Serial.begin(115200);
  
  //Copy the EEPROM data for fast access data.
  for(start = 0; start <= 35; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;                                                                //Set start back to zero.
  gyro_address = eeprom_data[32];                                           //Store the gyro address in the variable.

  Wire.begin();                                                             //Start the I2C as master.
  
  TWBR = 12;                                                                //Set the I2C clock speed to 400kHz.

  //Arduino (Atmega) pins default to inputs, so they don't need to be explicitly declared as inputs.
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  DDRB |= B00110000;                                                        //Configure digital poort 12 and 13 as output.

  //Use the led on the Arduino for startup indication.
  digitalWrite(13,HIGH);                                                    //Turn on the warning led.

  //Check the EEPROM signature to make sure that the setup program is executed.
  while(eeprom_data[33] != 'J' || eeprom_data[34] != 'M' || eeprom_data[35] != 'B')delay(10);

  //The flight controller needs the MPU-6050 with gyro and accelerometer
  //If setup is completed without MPU-6050 stop the flight controller program  
  if(eeprom_data[31] == 2 || eeprom_data[31] == 3)delay(10);

  set_gyro_registers();                                                     //Set the specific gyro registers.

  for (cal_int = 0; cal_int < 1250 ; cal_int ++){                           //Wait 5 seconds before continuing.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delayMicroseconds(3000);                                                //Wait 3000us.
  }

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){                           //Take 2000 readings for calibration.
    if(cal_int % 15 == 0)digitalWrite(13, !digitalRead(13));                //Change the led status to indicate calibration.
    gyro_signalen();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
    //We don't want the esc's to be beeping annoyingly. So let's give them a 1000us puls while calibrating the gyro.
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3);                                                               //Wait 3 milliseconds before the next loop.
  }
  //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= 2000;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= 2000;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= 2000;                                                 //Divide the yaw total by 2000.

  PCICR |= (1 << PCIE0);                                                    //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                                  //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                                  //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                                  //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                                  //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

  //Wait until the receiver is active and the throtle is set to the lower position.
  while(R3 < 990 || R3 > 1020 || R4 < 1400){
    R3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us
    R4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us
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
  start = 0;                                                                //Set start back to 0.

  //Load the battery voltage to the battery_voltage variable.
  //65 is the voltage compensation for the diode.
  //12.6V equals ~5V @ Analog 0.
  //12.6V equals 1023 analogRead(0).
  //1260 / 1023 = 1.2317.
  //The variable battery_voltage holds 1050 if the battery voltage is 10.5V.
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();                                                    //Set the timer for the next loop.
  loop_timer_1 = micros();                                                    //Set the timer for the next loop.

  //When everything is done, turn off the led.
  digitalWrite(13,LOW);                                                     //Turn off the warning led.
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Main program loop
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){

  //Gyro Rate Filter//
  rollD1 = deg2rad*(rollD1 * 0.7) + (gyro_roll* 0.3);   //[rad/s]
  pitchD1 = deg2rad*(pitchD1 * 0.7) + (gyro_pitch* 0.3); //[rad/s]
  yawD1 = deg2rad*(yawD1 * 0.7) + (gyro_yaw * 0.3); //[rad/s] 
  
  //Gyro Angles [rad]//
  pitch += gyro_pitch*dt;
  roll += gyro_roll*dt;                                      
  yaw += gyro_yaw*dt;                                       
  
  //Yawing Angle Adjustment//
  yawADD=sin(gyro_yaw*dt);
  pitch -= roll * yawADD;                  //If the IMU has yawed transfer the roll angle to the pitch angle.
  roll += pitch * yawADD;                  //If the IMU has yawed transfer the pitch angle to the roll angle.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
  
  if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    pitch_acc = asin((float)acc_y/acc_total_vector)*accSense;          //Calculate the pitch angle.
  }
  if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
    roll_acc = asin((float)acc_x/acc_total_vector)*-accSense;          //Calculate the roll angle.
  }
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
  pitch_acc += 1.785060514*PI/180.0;                                                   //Accelerometer calibration value for pitch.
  roll_acc -= 0;                                                    //Accelerometer calibration value for roll.

  //Complementary Filter//
  pitch = pitch * 0.9996 + pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
  roll = roll * 0.9996 + roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

  //Start Motors//
  if(R3 < 1050 && R4 < 1050)start = 1; //For starting the motors: throttle low and yaw left (step 1).
  if(start == 1 && R3 < 1050 && R4 > 1450){ //When yaw stick is back in the center position start the motors (step 2).
    start = 2;

    pitch = pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
    roll = roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.
    gyro_angles_set = true;                                                 //Set the IMU started flag.

    //Reset the SMC controller for a bumpless start.
    pitch_des = 0;
    roll_des = 0;
    yaw_des = 0;
    pitch = 0;
    roll = 0;
    yaw = 0;
    pitchD1_des_last = 0;
    rollD1_des_last = 0;
    yawD1_des_last = 0;
    
  }
  
  //Stop Motors//
  if(start == 2 && R3 < 1050 && R4 > 1950)start = 0;

  smc();                                                            //PID inputs are known. So we can calculate the pid output.  

  //The battery voltage is needed for compensation.
  //A complementary filter is used to reduce noise.
  //0.09853 = 0.08 * 1.2317.
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;

  //Turn on the led if battery voltage is to low.
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(13, HIGH);


  throttle = R3;                                      //We need the throttle signal as a base signal.

  if (start == 2){                                                          //The motors are started.
    if (battery_voltage < 1240 && battery_voltage > 800){                   //Is the battery connected?
      esc1 += esc1 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-1 pulse for voltage drop.
      esc2 += esc2 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-2 pulse for voltage drop.
      esc3 += esc3 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-3 pulse for voltage drop.
      esc4 += esc4 * ((1240 - battery_voltage)/(float)3500);              //Compensate the esc-4 pulse for voltage drop.
    } 

    if (esc1 < 1100) esc1 = 1100;                                         //Keep the motors running.
    if (esc2 < 1100) esc2 = 1100;                                         //Keep the motors running.
    if (esc3 < 1100) esc3 = 1100;                                         //Keep the motors running.
    if (esc4 < 1100) esc4 = 1100;                                         //Keep the motors running.

    if(esc1 > 2000)esc1 = 2000;                                           //Limit the esc-1 pulse to 2000us.
    if(esc2 > 2000)esc2 = 2000;                                           //Limit the esc-2 pulse to 2000us.
    if(esc3 > 2000)esc3 = 2000;                                           //Limit the esc-3 pulse to 2000us.
    if(esc4 > 2000)esc4 = 2000;                                           //Limit the esc-4 pulse to 2000us.  
  }

  else{
    esc1 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-1.
    esc2 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-2.
    esc3 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-3.
    esc4 = 1000;                                                           //If start is not 2 keep a 1000us pulse for ess-4.
  }

  ////////////////////////////////////////////////////////////////////////////////////////////////////
  //Creating ESC Pulses
  ////////////////////////////////////////////////////////////////////////////////////////////////////

  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Because of the angle calculation the loop time is getting very important. If the loop time is 
  //longer or shorter than 4000us the angle calculation is off. If you modify the code make sure 
  //that the loop time is still 4000us and no longer! More information can be found on 
  //the Q&A page: 
  //! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! ! !
  //Serial.println(micros() - loop_timer);
  if(micros() - loop_timer > 4050)digitalWrite(13, HIGH);                   //Turn on the LED if the loop time exceeds 4050us.
  
  //All the information for controlling the motor's is available.
  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.
  
  //There is always 1000us of spare time. So let's do something usefull that is very time consuming.
  //Get the current gyro and receiver data and scale it to degrees per second for the pid calculations.
  gyro_signalen();

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
  time_ = micros() - loop_timer_1;
  Serial.print(time_,0);
  Serial.print(",");
  Serial.print(roll, 3);
  Serial.print(",");
  Serial.print(pitch, 3);
  Serial.print(",");
  Serial.print(yaw, 3);
  Serial.print(",");
  Serial.print(rollD1, 3);
  Serial.print(",");
  Serial.print(pitchD1, 3);
  Serial.print(",");
  Serial.print(yawD1, 3);
  Serial.print(",");
  Serial.print(u1, 3);
  Serial.print(",");
  Serial.print(u2, 3);
  Serial.print(",");
  Serial.print(u3, 3);
  Serial.print(",");
  Serial.print(u4, 3);
  Serial.println(";");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Reading Receiver Signals 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ISR(PCINT0_vect){
  current_time = micros();
  //Channel 1=========================================
  if(PINB & B00000001){                                                     //Is input 8 high?
    if(last_channel_1 == 0){                                                //Input 8 changed from 0 to 1.
      last_channel_1 = 1;                                                   //Remember current input state.
      timer_1 = current_time;                                               //Set timer_1 to current_time.
    }
  }
  else if(last_channel_1 == 1){                                             //Input 8 is not high and changed from 1 to 0.
    last_channel_1 = 0;                                                     //Remember current input state.
    receiver_input[1] = current_time - timer_1;                             //Channel 1 is current_time - timer_1.
  }
  //Channel 2=========================================
  if(PINB & B00000010 ){                                                    //Is input 9 high?
    if(last_channel_2 == 0){                                                //Input 9 changed from 0 to 1.
      last_channel_2 = 1;                                                   //Remember current input state.
      timer_2 = current_time;                                               //Set timer_2 to current_time.
    }
  }
  else if(last_channel_2 == 1){                                             //Input 9 is not high and changed from 1 to 0.
    last_channel_2 = 0;                                                     //Remember current input state.
    receiver_input[2] = current_time - timer_2;                             //Channel 2 is current_time - timer_2.
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                                    //Is input 10 high?
    if(last_channel_3 == 0){                                                //Input 10 changed from 0 to 1.
      last_channel_3 = 1;                                                   //Remember current input state.
      timer_3 = current_time;                                               //Set timer_3 to current_time.
    }
  }
  else if(last_channel_3 == 1){                                             //Input 10 is not high and changed from 1 to 0.
    last_channel_3 = 0;                                                     //Remember current input state.
    receiver_input[3] = current_time - timer_3;                             //Channel 3 is current_time - timer_3.

  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                                    //Is input 11 high?
    if(last_channel_4 == 0){                                                //Input 11 changed from 0 to 1.
      last_channel_4 = 1;                                                   //Remember current input state.
      timer_4 = current_time;                                               //Set timer_4 to current_time.
    }
  }
  else if(last_channel_4 == 1){                                             //Input 11 is not high and changed from 1 to 0.
    last_channel_4 = 0;                                                     //Remember current input state.
    receiver_input[4] = current_time - timer_4;                             //Channel 4 is current_time - timer_4.
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Reading Gyroscope and Accelerometer Angles
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                   //Start communication with the gyro.
    Wire.write(0x3B);                                                       //Start reading @ register 43h and auto increment with every read.
    Wire.endTransmission();                                                 //End the transmission.
    Wire.requestFrom(gyro_address,14);                                      //Request 14 bytes from the gyro.
    
    R1 = convert_receiver_channel(1);                 //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    R2 = convert_receiver_channel(2);                 //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    R3 = convert_receiver_channel(3);                 //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    R4 = convert_receiver_channel(4);                 //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.
    
    while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
    acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
    acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
    acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
    temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
    gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
    gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[eeprom_data[28] & 0b00000011]/gyroSense;                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)gyro_roll *= -1;                          //Invert gyro_roll if the MSB of EEPROM bit 28 is set.
  gyro_pitch = gyro_axis[eeprom_data[29] & 0b00000011]/gyroSense;                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)gyro_pitch *= -1;                         //Invert gyro_pitch if the MSB of EEPROM bit 29 is set.
  gyro_yaw = gyro_axis[eeprom_data[30] & 0b00000011]/gyroSense;                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)gyro_yaw *= -1;                           //Invert gyro_yaw if the MSB of EEPROM bit 30 is set.

  acc_x = acc_axis[eeprom_data[29] & 0b00000011];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  if(eeprom_data[29] & 0b10000000)acc_x *= -1;                              //Invert acc_x if the MSB of EEPROM bit 29 is set.
  acc_y = acc_axis[eeprom_data[28] & 0b00000011];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  if(eeprom_data[28] & 0b10000000)acc_y *= -1;                              //Invert acc_y if the MSB of EEPROM bit 28 is set.
  acc_z = acc_axis[eeprom_data[30] & 0b00000011];                           //Set acc_z to the correct axis that was stored in the EEPROM.
  if(eeprom_data[30] & 0b10000000)acc_z *= -1;                              //Invert acc_z if the MSB of EEPROM bit 30 is set.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Sliding Mode Controller
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void smc(){

  //Auto Leveling//
  pitch_des -= autoLevel*pitch*k_AL;                               //Calculate the pitch angle correction
  roll_des -= autoLevel*roll*k_AL;                                 //Calculate the roll angle correction

  //Desired Angular Acceleration [rad/s2]//
  pitchD2_des = (pitchD1_des - pitchD1_des_last)/dt;
  rollD2_des = (rollD1_des - rollD1_des_last)/dt;
  yawD2_des = (yawD1_des - yawD1_des_last)/dt;

  //Desired Angular Velocity [rad/s]//
  pitchD1_des = (R2-1500)*max_pitch/500;
  rollD1_des = (R1-1500)*max_roll/500;
  yawD1_des = (R4-1500)*max_yaw/500;

  //Transmitter Dead Band//
  pitchD1_des = (abs(pitchD1_des) < 5*deg2rad) ? 0 : pitchD1_des;
  rollD1_des = (abs(rollD1_des) < 5*deg2rad) ? 0 : rollD1_des;
  yawD1_des = (abs(yawD1_des) < 5*deg2rad) ? 0 : yawD1_des;

  //Desired Angular Position [rad]//
  pitch_des+=pitchD1_des*dt;
  roll_des+=rollD1_des*dt;
  yaw_des+=yawD1_des*dt;
 
  //Error Calculations [rad]//
  pitch_e = pitch_des - pitch;
  roll_e = roll_des - roll;
  yaw_e =  yaw_des - yaw;
  
  //Error Time Derivatives [rad/s]//
  pitchD1_e = pitchD1_des - pitchD1;
  rollD1_e = rollD1_des - rollD1;  
  yawD1_e = yawD1_des - yawD1;

  //Sliding Surfaces//
  s_pitch = c_pitch*pitch_e + pitchD1_e;
  s_roll = c_roll*roll_e + rollD1_e;
  s_yaw = c_yaw*yaw_e + yawD1_e;
  
  //Sliding Surface Saturation//
  s_pitch = (s_pitch > 1) ? 1 : (s_pitch < -1) ? -1 : s_pitch;
  s_roll = (s_roll > 1) ? 1 : (s_roll < -1) ? -1 : s_roll;
  s_yaw = (s_yaw > 1) ? 1 : (s_yaw < -1) ? -1 : s_yaw;

  //Thrust Inputs//
  throttle = (throttle > 1800) ? 1800 : throttle; // Need room to control
  u1 = 4.0*Kf*((throttle-1000)*vmax/1000.0)*((throttle-1000)*vmax/1000.0); // [N] Heave
  u2 = ((k_pitch*s_pitch) + (c_pitch*pitchD1_e) + pitchD2_des - (a1*rollD1*yawD1))/b1; // [Nm] Pitch Torque
  u3 = ((k_roll*s_roll) + (c_roll*rollD1_e) + rollD2_des - (a3*pitchD1*yawD1))/b2; // [Nm] Roll Torque
  u4 = ((k_yaw*s_yaw) + (c_yaw*yawD1_e) + yawD2_des - (a5*pitchD1*rollD1))/b3; // [Nm] Yaw Torque

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
  esc1 = w1*1000.0/vmax + 1000.0; // Front (CCW)
  esc2 = w2*1000.0/vmax + 1000.0; // Right (CW)
  esc3 = w3*1000.0/vmax + 1000.0; // Back (CCW)
  esc4 = w4*1000.0/vmax + 1000.0; // Left (CW)
  
  //Save Last Loop [rad/s]//
  rollD1_des_last=rollD1_des;
  pitchD1_des_last=pitchD1_des;
  yawD1_des_last=yawD1_des;

  

 
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Convert Receiver Signals
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//This part converts the actual receiver signals to a standardized 1000 – 1500 – 2000 microsecond value.
//The stored data in the EEPROM is used.
int convert_receiver_channel(byte function){
  byte channel, reverse;                                                       //First we declare some local variables
  int low, center, high, actual;
  int difference;

  channel = eeprom_data[function + 23] & 0b00000111;                           //What channel corresponds with the specific function
  if(eeprom_data[function + 23] & 0b10000000)reverse = 1;                      //Reverse channel when most significant bit is set
  else reverse = 0;                                                            //If the most significant is not set there is no reverse

  actual = receiver_input[channel];                                            //Read the actual receiver value for the corresponding function
  low = (eeprom_data[channel * 2 + 15] << 8) | eeprom_data[channel * 2 + 14];  //Store the low value for the specific receiver input channel
  center = (eeprom_data[channel * 2 - 1] << 8) | eeprom_data[channel * 2 - 2]; //Store the center value for the specific receiver input channel
  high = (eeprom_data[channel * 2 + 7] << 8) | eeprom_data[channel * 2 + 6];   //Store the high value for the specific receiver input channel

  if(actual < center){                                                         //The actual receiver value is lower than the center value
    if(actual < low)actual = low;                                              //Limit the lowest value to the value that was detected during setup
    difference = ((long)(center - actual) * (long)500) / (center - low);       //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 + difference;                                  //If the channel is reversed
    else return 1500 - difference;                                             //If the channel is not reversed
  }
  else if(actual > center){                                                                        //The actual receiver value is higher than the center value
    if(actual > high)actual = high;                                            //Limit the lowest value to the value that was detected during setup
    difference = ((long)(actual - center) * (long)500) / (high - center);      //Calculate and scale the actual value to a 1000 - 2000us value
    if(reverse == 1)return 1500 - difference;                                  //If the channel is reversed
    else return 1500 + difference;                                             //If the channel is not reversed
  }
  else return 1500;
}

void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[31] == 1){
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                          //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                                    //End the transmission with the gyro.

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1B);                                                          //We want to write to the GYRO_CONFIG register (1B hex)
    Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search.
    Wire.write(0x1C);                                                          //We want to write to the ACCEL_CONFIG register (1A hex)
    Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
    Wire.endTransmission();                                                    //End the transmission with the gyro

    //Let's perform a random register check to see if the values are written correct
    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1B);                                                          //Start reading @ register 0x1B
    Wire.endTransmission();                                                    //End the transmission
    Wire.requestFrom(gyro_address, 1);                                         //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
    if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
      digitalWrite(12,HIGH);                                                   //Turn on the warning led
      while(1)delay(10);                                                       //Stay in this loop for ever
    }

    Wire.beginTransmission(gyro_address);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro    

  }  
}
