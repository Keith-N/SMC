///////////////////////////////////////////////////////////////////////////////////////
//Sliding Mode Flight Controller V5 (SMCpositionTeensy.ino)
///////////////////////////////////////////////////////////////////////////////////////
//Description: Quadcopter flight controller featuring a nonlinear sliding mode
//             controller for attitude and a linear PID controller for position.
///////////////////////////////////////////////////////////////////////////////////////
//Technical Specifications
//  * Board = Teensy 3.5
///////////////////////////////////////////////////////////////////////////////////////

//Header Files//
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Mahony.h>
#include <Madgwick.h>
#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <stdlib.h>

//IMU Type//
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

//Defined Constants//
float deg2rad = PI/180;  
#define HEDGEHOG_BUF_SIZE 50 
#define HEDGEHOG_CM_DATA_SIZE 0x10
#define HEDGEHOG_MM_DATA_SIZE 0x16
#define HEDGEHOG_RAW_IMU_DATA_SIZE 0x20
#define HEDGEHOG_IMU_FUSION_DATA_SIZE 0x2A
#define POSITION_DATAGRAM_ID 0x0001
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define RAW_IMU_DATAGRAM_ID 0x0003
#define IMU_FUSION_DATAGRAM_ID 0x0005
#define HWSERIAL Serial1

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SMC GAINS (ATTITUDE)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Misc//
int satBound = 1;     //Saturation Function Bound  

//Equivalent Control Gains//
float cRoll = 2.5;                    
float cPitch = cRoll;                   
float cYaw = 3;                     

//Discontinuous Control Gains//
float kRoll = 25;      
float kPitch = kRoll;         
float kYaw = 8.5;

//Max Receiver Inputs//
float maxRoll = 164.0*deg2rad;
float maxPitch = 164.0*deg2rad;
float maxYaw = 70.0*deg2rad;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID GAINS (POSITION)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Proportional Gains//
float kpx = 50;
float kpy = kpx;
float kpz = 2;

//Integral Gains//
float kix = 0;
float kiy = 0;
float kiz = 0.0;

//Derivative Gains//
float kdx = 5;
float kdy = kdx;
float kdz = 1;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//IMU CALIBRATION AND FILTER SELECTION
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Gyroscope//
float pitchD1cal = 0.007961757;
float rollD1cal = 0.0287253;
float yawD1cal = -0.006502958;

//Accelerometer//
float accXcal = 0.339279779;
float accYcal = -0.047814309;
float accZcal = 0.182600612;

//Magnetometer//
float mag_offsets[3] = { 17.71F, -36.34F, 41.47F };
float mag_softiron_matrix[3][3] = {{ 0.985, -0.024,  0.021 },
                                    { -0.024,  0.979, -0.041 },
                                    { 0.021, -0.041,  1.040 }};
float mag_field_strength = 38.57F;
float yawCal = -0.62831853 - 0.44;

//Filter Selection//
//Mahony filter; //Weak
Madgwick filter; //Strong (Computationally Expensive)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//USER INITIALIZED GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///Quadcopter Physical Parameters//
float Kf = 0.0000158;                         // [N-s2] Aerodynamic Force Constant
float Km = 0.00000029;                        // [N-m-s2] Aerodynamic Moment Constant
float Ixx = 0.016;                            // [kg/m2] Moment of Inertia Around x Axis
float Iyy = 0.016;                            // [kg/m2] Moment of Inertia Around y Axis
float Izz = 0.031;                            // [kg/m2] Moment of Inertia Around z Axis
float Jr = 0.00006;                           // [kg/m2] Rotor Inertia
float len = 0.225;                            // [m] Quadcopter Arm Length
float g = 9.81;                               // Gravitational Acceleration
float m = 0;                                // Mass of the Quadrotor

//ESC Pins//
int escOut1 = 5;
int escOut2 = 6;
int escOut3 = 10;
int escOut4 = 20;

//Receiver Pins//
int ch1 = 7;
int ch2 = 8;
int ch3 = 14;
int ch4 = 35;
int ch5 = 33;
int ch6 = 34;

//Fifth Order Polynomial Coefficients//
/*float p0 = -3319;
float p1 = 52.77;
float p2 = -0.245;
float p3 = 0.000549;
float p4 = -0.0000005792;
float p5 = 0.0000000002309;*/

// Linear Rotor Speed to ESC Pulse Fit Coefficients (9x4.7 Props)
//float p1 = 1.215;
//float p2 = 868.1;

// Linear Rotor Speed to ESC Pulse Fit Coefficients (8x4.5 Props)
float p1 = 1.279;
float p2 = 774.9;

//Misc//
int pwmFreq = 250;
int pwmRes = 8;
float dt = 0.004;
int escPulseTime = 4000;
int led = 13;
int battPin = 32;
float vMax = 555;                             // [rad/s] Maximum Motor Velocity

float IRsense = 13.75*PI/30;                  // [rad/Vs] IR Sensor Sensitivity
float wavg; 
//IR Pins
int inIR1 = 23;
int inIR2 = 22;
int inIR3 = 21;
int inIR4 = 17;
int count = 0;
float w1a_total = 0;
float w1a_avg = 0;
float w2a_total = 0;
float w2a_avg = 0;
float w3a_total = 0;
float w3a_avg = 0;
float w4a_total = 0;
float w4a_avg = 0;
float w_total = 0;
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SYSTEM INITIALIZED GLOBAL VARIABLES
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Inertia Terms//
float a1=(Iyy-Izz)/Ixx, a2=Jr/Ixx, a3=(Izz-Ixx)/Iyy, a4=Jr/Iyy, a5=(Ixx-Iyy)/Izz;
float b1=len/Ixx, b2=len/Iyy, b3=1/Izz;

//Electronic Speed Controllers//
float u1, u2, u3, u4;                    // [N] Thrust Forces {Heave, Pitch, Roll, Yaw}
float esc1, esc2, esc3, esc4;            // [us] ESC Pulse Widths {Front (ccw), Right (cw), Back (ccw), Left(cw)}
int esc1PWM, esc2PWM, esc3PWM, esc4PWM;  // ESC Pulse Widths in Analog Write form                
int pwmMax = 256;                        // Get maximum value for selected PWM resolution (100% Duty)
int escInit = pwmMax/4;                  // Initializing pulse for ESCs, 25% duty

//Receiver//
volatile unsigned long R1 = 1500, R2 = 1500, R3 = 1500, R4 = 1500, R5 = 1500, R6 = 1500;          // [us] Receiver Pulses {Pitch, Roll, Heave, Yaw}
unsigned long prev1, prev2, prev3, prev4, prev5, prev6; // [us] Receiver Input Rising Edge Time 
float pitchDes, rollDes, yawDes;                        // Desired Euler Angles
float pitchDesD1, rollDesD1, yawDesD1;                  // Desired Euler Angular Rates
float pitchDesD1old, rollDesD1old, yawDesD1old;         // Last Loop Rate
float pitchDesD2, rollDesD2, yawDesD2;                  // Rate Derivative
float throttle;

//Sliding Mode Controller//
float pitch, roll, yaw;                     //[rad] Filtered Euler Angles
float gyroPitch, gyroRoll, gyroYaw;         //[ras/s] Raw Euler Angular Rates
float pitchD1, rollD1, yawD1;               //[ras/s] Filtered Euler Angular Rates
float ePitch, eRoll, eYaw;                  //Error
float ePitchD1, eRollD1, eYawD1;            //Error Derivative
float sPitch, sRoll, sYaw;                  //Sliding Surfaces
float F1, F2, F3, F4;                       // Fault Magnitude
float F1p, F2p, F3p, F4p;                   // Fault Magnitude

//Rotor Angular Velocities [rad/s]//
//Note: {1,2,3,4} = {Front (ccw), Right (cw), Back (ccw), Left(cw)}
float wr;                           // Relative Rotor Velocity (w2a - w1a + w4a - w3a)
float w1, w2, w3, w4;               // Input Rotor Velocities 
float w1a, w2a, w3a, w4a;           // Measured Rotor Velocities
float w1Des, w2Des, w3Des, w4Des;   // Desired Rotor Velocities

//Marvelmind Hedgehog//
int cnt = 1;                  //Loop counter
// int hedgehog_pos_updated = 0; // New Data Flag
// bool high_resolution_mode;
// byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
// byte hedgehog_serial_buf_ofs = 0;
// unsigned int hedgehog_data_id;
// typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
// typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;
int hedgehog_pos_updated;// flag of new data from hedgehog received
bool high_resolution_mode;

unsigned long timer=0;

// IMU sensors raw data
int16_t imu_acc_x,imu_acc_y,imu_acc_z;
int16_t imu_gyro_x,imu_gyro_y,imu_gyro_z;
int16_t imu_compass_x,imu_compass_y,imu_compass_z;
uint32_t imu_raw_timestamp;
int imu_raw_updated;

// IMU fusion data
int32_t imu_fusion_x, imu_fusion_y, imu_fusion_z;// imu fusion position
int16_t imu_fusion_qw, imu_fusion_qx, imu_fusion_qy, imu_fusion_qz;// quaternion
int16_t imu_fusion_vx, imu_fusion_vy, imu_fusion_vz;// imu fusion speed
int16_t imu_fusion_ax, imu_fusion_ay, imu_fusion_az;//imu fusion acceleration
uint32_t imu_fusion_timestamp;
int imu_fusion_updated;
///

byte hedgehog_serial_buf[HEDGEHOG_BUF_SIZE];
byte hedgehog_serial_buf_ofs;
unsigned int hedgehog_data_id;

typedef union {byte b[2]; unsigned int w;int wi;} uni_8x2_16;
typedef union {byte b[4];float f;unsigned long v32;long vi32;} uni_8x4_32;

unsigned long microsPerReading, microsPrevious;

//Position PID Controller//
float hx, hy, hz;            // [mm] Global coordinates of hedgehog (X,Y,Z)
float ux, uy, uz;            // [N] Position Forces
float X, Y, Z;               // [m] Global Coordinates of hedgehog (X,Y,Z) 
float x, y, z;               // [m] Local Coordinates of hedgehog (X,Y,Z) 
float xDes, yDes, zDes;      // [m] Desired Local Coordinates
float ex, ey, ez;            // Position Error
float exOld, eyOld, ezOld;   // Last Position Error
float eix, eiy, eiz;         // Integrated Error
float exD1, eyD1, ezD1;      // Position Error Derivative
float gpsCal[4];             // Zero-Start Calibration

//Misc//
int start = 0;                      //Motor start
int calcInt;                        //Calibration Counter  
int batt;                           //Battery Voltage 
//float RS1 = 4.75;                   //[kohms] Resistor 1
//float RS2 = 1.5;                    //[kohms] Resistor 2
//float vDiv = RS2/(RS1+RS2);         //Voltage Divider          
unsigned long loopTimer, timer1;    //Loop Timers
float sinPitch, sinRoll, sinYaw;    //Euler Angle Sines
float cosPitch, cosRoll, cosYaw;    //Euler Angle Cosines

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//READING RECEIVER SIGNALS (PIN INTERRUPTS)
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
//READING IMU
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void gyro_signalen(){

  sensors_event_t gyro_event;
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  gyro.getEvent(&gyro_event);
  accelmag.getEvent(&accel_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float xMag = mag_event.magnetic.x - mag_offsets[0];
  float yMag = mag_event.magnetic.y - mag_offsets[1];
  float zMag = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = xMag * mag_softiron_matrix[0][0] + yMag * mag_softiron_matrix[0][1] + zMag * mag_softiron_matrix[0][2];
  float my = xMag * mag_softiron_matrix[1][0] + yMag * mag_softiron_matrix[1][1] + zMag * mag_softiron_matrix[1][2];
  float mz = xMag * mag_softiron_matrix[2][0] + yMag * mag_softiron_matrix[2][1] + zMag * mag_softiron_matrix[2][2];

  //Gyroscope Angular Rates [rad/s]//
  gyroRoll = gyro_event.gyro.x - rollD1cal;
  gyroPitch = gyro_event.gyro.y - pitchD1cal;
  gyroYaw = -gyro_event.gyro.z - yawD1cal;

  //Rate Filter//
  pitchD1 = (pitchD1 * 0.7) + (gyroPitch * 0.3);
  rollD1 = (rollD1 * 0.7) + (gyroRoll * 0.3);   
  yawD1 = (yawD1 * 0.7) + (gyroYaw * 0.3);     

  //Accelerometer Vectors//
  float accX = accel_event.acceleration.x - accXcal;
  //Serial.print(accX);
  //Serial.print("               ");
  float accY = accel_event.acceleration.y - accYcal;
  //Serial.print(accX);
  //Serial.print("               ");
  float accZ = accel_event.acceleration.z - accZcal;
  //Serial.println(accX);

   //Angle Filter//
  filter.update(rollD1/deg2rad, pitchD1/deg2rad, -yawD1/deg2rad,
                accX, accY, accZ,
                mx, my, mz);

 
  //Angles [rad]//  
  pitch = filter.getPitch()*deg2rad;
  roll = filter.getRoll()*deg2rad;
  yaw = asin(sin(-filter.getYaw()*deg2rad)) - yawCal;

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID CONTROL LOOP (POSITION)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void pid(){

  //Global Coordinates//
  X = (hx - gpsCal[1])/1000.0;
  Y = (hy - gpsCal[2])/1000.0;
  Z = (hz - gpsCal[3])/1000.0;
    
  //Local Coordinates//
  sinYaw = sin(yaw);
  cosYaw = cos(yaw);
  x=cosYaw*X + sinYaw*Y;
  y=-sinYaw*X + cosYaw*Y;
  z=Z;

  //Desired Position [m]//
  xDes = 0;
  yDes = 0; 
  //zDes = (R3-1000.0)/1000.0;
  //zDes = (zDes < 0) ? 0 : zDes; 
  zDes = 0.25;
  
  //Error//
  ex = xDes - x;       
  ey = yDes - y;       
  ez = zDes - z; 

  /*if(ex <= 0.01){
    ex = 0;
  }  
  if(ey <= 0.01){
    ey = 0;
  }  
  if(ez <= 0.01){
    ez = 0;
  }*/

  //Error Derivative//
  exD1 = (ex - exOld)*16.0;  
  eyD1 = (ey - eyOld)*16.0; 
  ezD1 = (ez - ezOld)*16.0; 

  //Error Integral//
  if(start == 2){
  eix += ex;       
  eiy += ey; 
  eiz += ez;
  eix = (eix > 0.5) ? 0.5 : eix;
  eiy = (eiy > 0.5) ? 0.5 : eiy;
  eiz = (eiz > 15) ? 15 : eiz;
  }
          
  //Control Outputs [N]//
  ux = kpx*ex + kdx*exD1 + kix*eix;
  uy = kpy*ey + kdy*eyD1 + kiy*eiy;
  uz = kpz*ez + kdz*ezD1 + kiz*eiz;

  //Desired Angles [rad]//
  pitchDes = 0; //-atan(-uy/sqrt(ux*ux+(uz+g)*(uz+g)));
  rollDes = 0; // -atan(ux/(uz+g));
  yawDes = 0;
  
  //Save Last Loop//
  exOld = ex;
  eyOld = ey;
  ezOld = ez;

  //if (R5 < 1950)uz = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//READING INDOOR GPS
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*
NOTE:
 * HEDGEHOG serial data (TX2 pin) to digital pin 0 (RXD)
 * Hedgehog ground to Arduino ground
 * IMPORTANT: When uploading code to Arduino, unplug the pin 0. This is a serial pin for the Arduino, and code will not upload if plugged in!
 * Beacons need to be setup as in the YouTube Marvelmind Indoor GPS unboxing video
 * Hedgehog (mobile beacon) will be audibly ticking when this is working.
 * Submap must be frozen
 * Returned data in [mm]
 * Baud rate: 500000 (Marvelmind)
*/
// Calculate CRC-16 of hedgehog packet
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
}// hedgehog_set_crc16


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
 // Serial.println(HWSERIAL.available());
  
  while(HWSERIAL.available() > 0)
    {
 //     Serial.println("Entering Loop");
      //Serial.println(packet_received);
      if (hedgehog_serial_buf_ofs>=HEDGEHOG_BUF_SIZE) 
      {
        hedgehog_serial_buf_ofs= 0;// restart bufer fill
        break;// buffer overflow
      }
      total_received_in_loop++;
      if (total_received_in_loop>100) break;// too much data without required header
      
      incoming_byte= HWSERIAL.read();
//      Serial.print("Incoming byte:");
//      Serial.println(incoming_byte);
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
                       (hedgehog_data_id == POSITION_DATAGRAM_HIGHRES_ID) ||
                       (hedgehog_data_id == RAW_IMU_DATAGRAM_ID) ||
                       (hedgehog_data_id == IMU_FUSION_DATAGRAM_ID);
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
            case RAW_IMU_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_RAW_IMU_DATA_SIZE);
              break;
            }
            case IMU_FUSION_DATAGRAM_ID:
            {
              good_byte= (incoming_byte == HEDGEHOG_IMU_FUSION_DATA_SIZE);
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
//Serial.print("Buf_ofs:");
//Serial.println(hedgehog_serial_buf_ofs);
      
      if (hedgehog_serial_buf_ofs>5)
        {
          packet_size=  7 + hedgehog_serial_buf[4];
//          Serial.print("Packet size is:");
//          Serial.println(packet_size);
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
              x= 10*long(un16.wi);

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              y= 10*long(un16.wi);
              
              // height of hedgehog, cm==>mm (FW V3.97+)
              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              z= 10*long(un16.wi);
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= false;
              return;
              break;
            }

            case POSITION_DATAGRAM_HIGHRES_ID:
            {
              // coordinates of hedgehog (X,Y), mm
              un32.b[0]= hedgehog_serial_buf[9];
              un32.b[1]= hedgehog_serial_buf[10];
              un32.b[2]= hedgehog_serial_buf[11];
              un32.b[3]= hedgehog_serial_buf[12];
              x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[13];
              un32.b[1]= hedgehog_serial_buf[14];
              un32.b[2]= hedgehog_serial_buf[15];
              un32.b[3]= hedgehog_serial_buf[16];
              y= un32.vi32;
              
              // height of hedgehog, mm 
              un32.b[0]= hedgehog_serial_buf[17];
              un32.b[1]= hedgehog_serial_buf[18];
              un32.b[2]= hedgehog_serial_buf[19];
              un32.b[3]= hedgehog_serial_buf[20];
              z= un32.vi32;
              
              hedgehog_pos_updated= 1;// flag of new data from hedgehog received
              high_resolution_mode= true;
              return;
              break;
            }

            case RAW_IMU_DATAGRAM_ID:
            {
              un16.b[0]= hedgehog_serial_buf[5];
              un16.b[1]= hedgehog_serial_buf[6];
              imu_acc_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[7];
              un16.b[1]= hedgehog_serial_buf[8];
              imu_acc_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[9];
              un16.b[1]= hedgehog_serial_buf[10];
              imu_acc_z= un16.wi;

              //

              un16.b[0]= hedgehog_serial_buf[11];
              un16.b[1]= hedgehog_serial_buf[12];
              imu_gyro_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[13];
              un16.b[1]= hedgehog_serial_buf[14];
              imu_gyro_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[15];
              un16.b[1]= hedgehog_serial_buf[16];
              imu_gyro_z= un16.wi;

              //

              un16.b[0]= hedgehog_serial_buf[17];
              un16.b[1]= hedgehog_serial_buf[18];
              imu_compass_x= un16.wi;

              un16.b[0]= hedgehog_serial_buf[19];
              un16.b[1]= hedgehog_serial_buf[20];
              imu_compass_y= un16.wi;

              un16.b[0]= hedgehog_serial_buf[21];
              un16.b[1]= hedgehog_serial_buf[22];
              imu_compass_z= un16.wi; 

              un32.b[0]= hedgehog_serial_buf[29];
              un32.b[1]= hedgehog_serial_buf[30];
              un32.b[2]= hedgehog_serial_buf[31];
              un32.b[3]= hedgehog_serial_buf[32];
              imu_raw_timestamp= un32.vi32;

              imu_raw_updated= 1;
              return;
              break;
            }

            case IMU_FUSION_DATAGRAM_ID:
            {
              un32.b[0]= hedgehog_serial_buf[5+0];
              un32.b[1]= hedgehog_serial_buf[5+1];
              un32.b[2]= hedgehog_serial_buf[5+2];
              un32.b[3]= hedgehog_serial_buf[5+3];
              imu_fusion_x= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[9+0];
              un32.b[1]= hedgehog_serial_buf[9+1];
              un32.b[2]= hedgehog_serial_buf[9+2];
              un32.b[3]= hedgehog_serial_buf[9+3];
              imu_fusion_y= un32.vi32;

              un32.b[0]= hedgehog_serial_buf[13+0];
              un32.b[1]= hedgehog_serial_buf[13+1];
              un32.b[2]= hedgehog_serial_buf[13+2];
              un32.b[3]= hedgehog_serial_buf[13+3];
              imu_fusion_z= un32.vi32;

              //////

              un16.b[0]= hedgehog_serial_buf[17+0];
              un16.b[1]= hedgehog_serial_buf[17+1];
              imu_fusion_qw= un16.wi;

              un16.b[0]= hedgehog_serial_buf[19+0];
              un16.b[1]= hedgehog_serial_buf[19+1];
              imu_fusion_qx= un16.wi;

              un16.b[0]= hedgehog_serial_buf[21+0];
              un16.b[1]= hedgehog_serial_buf[21+1];
              imu_fusion_qy= un16.wi;

              un16.b[0]= hedgehog_serial_buf[23+0];
              un16.b[1]= hedgehog_serial_buf[23+1];
              imu_fusion_qz= un16.wi;

              //////

              un16.b[0]= hedgehog_serial_buf[25+0];
              un16.b[1]= hedgehog_serial_buf[25+1];
              imu_fusion_vx= un16.wi;

              un16.b[0]= hedgehog_serial_buf[27+0];
              un16.b[1]= hedgehog_serial_buf[27+1];
              imu_fusion_vy= un16.wi;

              un16.b[0]= hedgehog_serial_buf[29+0];
              un16.b[1]= hedgehog_serial_buf[29+1];
              imu_fusion_vz= un16.wi;

              //////

              un16.b[0]= hedgehog_serial_buf[31+0];
              un16.b[1]= hedgehog_serial_buf[31+1];
              imu_fusion_ax= un16.wi;

              un16.b[0]= hedgehog_serial_buf[33+0];
              un16.b[1]= hedgehog_serial_buf[33+1];
              imu_fusion_ay= un16.wi;

              un16.b[0]= hedgehog_serial_buf[35+0];
              un16.b[1]= hedgehog_serial_buf[35+1];
              imu_fusion_az= un16.wi;

              //////

              un32.b[0]= hedgehog_serial_buf[39+0];
              un32.b[1]= hedgehog_serial_buf[39+1];
              un32.b[2]= hedgehog_serial_buf[39+2];
              un32.b[3]= hedgehog_serial_buf[39+3];
              imu_raw_timestamp= un32.vi32;
              
              imu_fusion_updated= 1;
              pid();                  // PID CONTROLLER IMPEMENTED HERE!
              Serial.println("IMU data is:");
              Serial.print(imu_compass_x); Serial.print('\t');
              Serial.print(imu_compass_y); Serial.print('\t');
              Serial.print(x); Serial.print("\t");
              Serial.print(y); Serial.print("\t");
              Serial.print(z); Serial.print("\t");
              Serial.print("\n");
              return;
              break;
            }
          }//switch(hedgehog_data_id)
        }// checksum success 
    }//if (packet_received)  
}

// MAREVELMIND HEDGEHOG SUPPORT INITIALIZE////// 
void setup_hedgehog() 
{
  HWSERIAL.begin(500000);
  
  hedgehog_serial_buf_ofs= 0;
  hedgehog_pos_updated= 0;

  imu_raw_updated= 0;
  imu_fusion_updated= 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SMC CONTROL LOOP (ATTITUDE)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void smc() {

  //Desired Angular Acceleration [rad/s2]//
  pitchDesD2 = (pitchDesD1 - pitchDesD1old)/dt;
  rollDesD2 = (rollDesD1 - rollDesD1old)/dt;
  yawDesD2 = (yawDesD1 - yawDesD1old)/dt;

  //Desired Angular Velocity [rad/s]//
  pitchDesD1 = (R2-1500.0)*maxPitch/500.0;
  rollDesD1 = (R1-1500.0)*maxRoll/500.0;
  if(R3>1150) yawDesD1 = (R4-1500.0)*maxRoll/500.0;

  //Transmitter Dead Band//
  pitchDesD1 = (abs(pitchDesD1) < 10.0*deg2rad) ? 0 : pitchDesD1;
  rollDesD1 = (abs(rollDesD1) < 10.0*deg2rad) ? 0 : rollDesD1;
  yawDesD1 = (abs(yawDesD1) < 10.0*deg2rad) ? 0 : yawDesD1;
 
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
  //u1 = (m*g + uz)/(cos(pitch)*cos(roll)); // [N] Heave;           // Might want to implement NED sooner rather than later
  //u1 = m*g + uz;
  u1 = (throttle-1000.0)*30.0/1000.0; // [N] Heave
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
  // Fifth Order Fit
  //esc1 = p5*w1*w1*w1*w1*w1 + p4*w1*w1*w1*w1 + p3*w1*w1*w1 + p2*w1*w1 + p1*w1 + p0;
  //esc2 = p5*w2*w2*w2*w2*w2 + p4*w2*w2*w2*w2 + p3*w2*w2*w2 + p2*w2*w2 + p1*w2 + p0;
  //esc3 = p5*w3*w3*w3*w3*w3 + p4*w3*w3*w3*w3 + p3*w3*w3*w3 + p2*w3*w3 + p1*w3 + p0; 
  //esc4 = p5*w4*w4*w4*w4*w4 + p4*w4*w4*w4*w4 + p3*w4*w4*w4 + p2*w4*w4 + p1*w4 + p0;
  
  // Linear Fit
  esc1 = p1*w1 + p2;
  esc2 = p1*w2 + p2;
  esc3 = p1*w3 + p2;
  esc4 = p1*w4 + p2;

 /* //Remove Negative Values//
  w1 = (w1 < 0) ? 0 : w1;
  w2 = (w2 < 0) ? 0 : w2;
  w3 = (w3 < 0) ? 0 : w3;
  w4 = (w4 < 0) ? 0 : w4;*/
  
 /* //ESC Pulse Widths [us]//
  esc1 = w1*1000.0/vMax + 1000.0; // Front (CCW)
  esc2 = w2*1000.0/vMax + 1000.0; // Right (CW)
  esc3 = w3*1000.0/vMax + 1000.0; // Back (CCW)
  esc4 = w4*1000.0/vMax + 1000.0; // Left (CW)*/
  

  //Save Last Loop [rad/s]//
  rollDesD1old=rollDesD1;
  pitchDesD1old=pitchDesD1;
  yawDesD1old=yawDesD1;
  
  }

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SETUP ROUTINE
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup(){

  Serial.begin(500000);
  Serial1.begin(500000);

  Serial.println("Starting setup!");
  //Warning LED//
  pinMode(led,OUTPUT); 
  digitalWrite(led,HIGH); //Setup has begun LED ON
  Serial.println("Setup has Begun!");
  // Initialize the sensors.
  if(!gyro.begin())
  {
    Serial.println("Ooops, no gyro detected ... Check your wiring!");
    while(1);
  }
  if(!accelmag.begin(ACCEL_RANGE_4G))
  {
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
    while(1);
  }
  filter.begin(250); // Filter expects 250 samples per second

  Serial.print("Waiting IMU Data...");
  Serial.print("\n");
  //Stuff added from the Marvelmind IMU sensor fusion code ////////////////////////////
  setup_hedgehog(); //Marvelmind hedgehog support initialize
  microsPerReading = 1000000; //Unsure about this piece of code
  microsPrevious = micros();

  //Indoor GPS Calibration//
  calcInt = 0;
  for (calcInt = 0; calcInt < 2000 ; calcInt ++){                           // Take 2000 readings for calibration.
    loop_hedgehog();                                                        // Read gps coordinates
    gpsCal[1] += hx;
    gpsCal[2] += hy;
    gpsCal[3] += hz;                                                              // Wait                                          
    delay(3);
  }

Serial.println("Hedgehog Calibrated!");

  //Average Position Offset//
  gpsCal[1] /= 2000.0F;  // x
  gpsCal[2] /= 2000.0F;  // y
  gpsCal[3] /= 2000.0F;  // z 

  // Setup ESC Pins
  pinMode(escOut1, OUTPUT);
  pinMode(escOut2, OUTPUT);
  pinMode(escOut3, OUTPUT);
  pinMode(escOut4, OUTPUT);

  Serial.println("ESC Pins Calibrated!");
  
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
Serial.println("ESCs INITIALIZED!");
  //Enable Pin Change Interrupts//
  attachInterrupt(ch1,ch1Int,CHANGE);
  attachInterrupt(ch2,ch2Int,CHANGE);
  attachInterrupt(ch3,ch3Int,CHANGE);
  attachInterrupt(ch4,ch4Int,CHANGE);
  attachInterrupt(ch5,ch5Int,CHANGE);
  attachInterrupt(ch6,ch6Int,CHANGE);      
Serial.println("Pin Change Interrupts");

  //IR pins
  pinMode(inIR1,INPUT); 
  pinMode(inIR2,INPUT); 
  pinMode(inIR3,INPUT); 
  pinMode(inIR4,INPUT);                                                   

Serial.println("Everything good! Calibrated!");

  batt=(analogRead(battPin)+52.7515)*1250.0/937.44;
  digitalWrite(led,LOW); //Setup Complete LED
  loopTimer = micros();                                                    // Set the timer for the next loop.
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN PROGRAM LOOP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(){
  ///////Altitude Control Dependencies//////
  // Initailize Mass Paramter (Took off at m= 2.5 with a drained battery)
  if (start == 2){
  m += 0.001;
  }
  if(m >= 2.3){
    m = 2.3;
  }
  if(start == 0){
    m = 0;
  }

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

  // Motor Speed Calculations
  w1a=IRsense*analogRead(inIR1);       // Front Right (CCW)
  w2a=IRsense*analogRead(inIR2);       // Back Right (CW)
  w3a=IRsense*analogRead(inIR3);       // Back Left (CCW)
  w4a=IRsense*analogRead(inIR4);       // Front Left (CW)

  //Control Loops//
  throttle = R3;   //We need the throttle signal as a base signal.
  loop_hedgehog(); //Get X,Y,Z coords and run pid() loop for desired angles
  gyro_signalen(); //Read IMU angles
  smc();           //Get ESC inputs from SMC loop

  //Read Battery Voltage//
  batt = batt * 0.92 + (analogRead(battPin) + 52.7515) * 0.1075268817;

  //Turn on the led if battery voltage is too low.
  //if(batt < 1000 && batt > 600)digitalWrite(led, HIGH);

  if (start == 2){
/*
  //Battery Voltage Compensation//
  if (batt < 1240 && batt > 800){         //Is the battery connected?
    esc1 += esc1 * ((1240 - batt)/3500.0);             
    esc2 += esc2 * ((1240 - batt)/3500.0);             
    esc3 += esc3 * ((1240 - batt)/3500.0);              
    esc4 += esc4 * ((1240 - batt)/3500.0);              
  } 
     
     */                                                         //The motors are started.
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
 
 // Stuff added from the Marvelmind IMU sensor fusion code ////////////////////////////
  while(micros() - loopTimer < 4000.0F);
  loopTimer = micros(); 

   unsigned long microsNow = micros();
//           Serial.print(imu_raw_updated); Serial.print('\t');
//           Serial.print(imu_fusion_updated); Serial.print('\n');
//           delay(1000);

   if (microsNow - microsPrevious >= microsPerReading){
   if (imu_raw_updated || imu_fusion_updated == 1)
     {// new raw IMU data from hedgehog available
       imu_raw_updated= 0;// clear new data flag  
       imu_fusion_updated=0;
//           Serial.println("IMU data is:");
//           Serial.print(imu_compass_x); Serial.print('\t');
//           Serial.print(imu_compass_y); Serial.print('\t');
//           Serial.print(x); Serial.print("\t");
//           Serial.print(y); Serial.print("\t");
//           Serial.print(z); Serial.print("\t");
//           Serial.print("\n");
           
       }

   if (imu_fusion_updated) 
     {
       imu_fusion_updated= 0;
     }

      microsPrevious = micros();
   }

}