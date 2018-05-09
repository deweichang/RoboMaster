#include <stdint.h>
#include <math.h>
#include "MPU9250.h"
#include "FlexCAN.h"
//#include <Wire.h>
#include "I2C.h"

//#include "RemoteControl.h"
//------------DEFINE VARIABLES---------------------------------------------
const float pi = 4.0 * atan(1); //define PI = 3.1415926
const int baudRate = 1000000;
int16_t gimbal_yaw_iq = 0, gimbal_pitch_iq = 0; //declare gimbal_iq --> motor speed command
uint16_t curr_ang_cnts[6] = {0, 0, 0, 0, 0, 0};
uint16_t curr_vel_cnts[6] = {0, 0, 0, 0, 0, 0};
int LoopPeriod = 1000;// * microsecond  //Hz
long previousMillis = 0; //used to calculate dt
unsigned long currentMillis;
float dt;
int prev_yaw_ang_diff = 0, prev_pitch_ang_diff = 0;
int prev_yaw_cnt_diff = 0, prev_pitch_cnt_diff = 0;
int prev_yaw_cnt_diff2 = 0, prev_pitch_cnt_diff2 = 0;
int prev_cnt_diff = 0;
float posP[2] = {0, 0}, posI[2] = {0, 0}, posD[2] = {0, 0}, velP[2] = {0, 0}, velI[2] = {0, 0}; //int16_t
float prev_pos_error[2] = {0, 0}, prev_pos_error2[2] = {0, 0}, prev_vel_error[2] = {0, 0};
float pos_output[2] = {0, 0};
int16_t prev_vel_output = 0;
float gxx[3] = {0, 0, 0}, gzz[3] = {0, 0, 0};
float gx_fil = 0, gz_fil = 0;
int yaw_ang_ref = 335, pitch_ang_ref = 45;
float yaw_pos_error[4] = {0, 0, 0, 0};
float pitch_pos_error[4] = {0, 0, 0, 0};
//-------------------Finite State Machine---------------------------
const int Initial = 100;
const int Horizontal = 101;
const int Vertical = 102;
int state = Initial;
char readIn;
int counter = 0;
int yawMove = 335, pitchMove = 45;
int horDir = 1, verDir = -1;
//-------------------CANBUS SET UP--------------------------
FlexCAN CANbus(baudRate, 1);
static CAN_message_t tx_message, rx_message;
//--------------------IMU setup----------------------------------------------
MPU9250 IMU(0x68, 0);
float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
int beginStatus;
//-------------------TIMER SETUP------------------------------------------
IntervalTimer myTimer;
//-------------------MAIN FUNCTIONS----------------------------------------
void setup() {
  //-----------REMOTE CONTROL---------------------------
  //Serial1.begin(100000, SERIAL_8E1);
  //-----------INITIALIZE SERIAL PORT-------------------
  Serial.begin(115200);
  //------------LED-------------------------------------
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //------------CAN TX(GIMBAL)--------------------------
  //CANbus.begin();
  tx_message.ext = 0;
  tx_message.id = 0x1FF;
  tx_message.len = 8;
  rx_message.ext = 0;
  rx_message.id = 0x202;
  rx_message.len = 8;
  CAN_filter();
  //-----------TIMER INI--------------------------------
  myTimer.begin(EncoderRead, LoopPeriod);
  //-----------IMU INI----------------------------------
  beginStatus = IMU.begin(ACCEL_RANGE_4G, GYRO_RANGE_250DPS);
}
//------------------------------------ALL SUB_FUNCTIONS-----------------------------------------------------------------
void IMURead() {
  if (beginStatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
  }
  else {
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    //gxx[0] = gx; gx_fil = 0.5*gxx[0] + 0.3*gxx[1] + 0.2*gxx[2]; gxx[2] = gxx[1]; gxx[1] = gxx[0]; //moving average filter
    gx_fil = butterworthFilter(gx); //gxx[1] = gx_fil;
    gzz[0] = gz; gz_fil = (gzz[0] + gzz[1] + gzz[2]) / 3; gzz[2] = gzz[1]; gzz[1] = gzz[0];
    //gxx[0] = gx; gx_fil = (gxx[0] + gxx[1] + gxx[2]) / 3; gxx[2] = gxx[1]; gxx[1] = gxx[0];
  }
}
void ctrl_loop(int yaw, int pitch) {
  gimbal_yaw_iq = CascadeControlYaw(yaw);
  gimbal_pitch_iq = CascadeControlPitch(pitch);
}

void Set_CM_Speed(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq)
{
  tx_message.buf[0] = (unsigned char)(gimbal_yaw_iq >> 8);
  tx_message.buf[1] = (unsigned char)gimbal_yaw_iq;
  tx_message.buf[2] = (unsigned char)(gimbal_pitch_iq >> 8);
  tx_message.buf[3] = (unsigned char)gimbal_pitch_iq ;
  tx_message.buf[4] = 0x00;
  tx_message.buf[5] = 0x00;
  tx_message.buf[6] = 0x00;
  tx_message.buf[7] = 0x00;
  CANbus.write(tx_message);
}
void debug() {
  //Serial.print("  Encoder: "); Serial.print(curr_ang_cnts[5]);
  /*Serial.print("  YAW Pos Error: ");  Serial.print(prev_pos_error[0]);
  Serial.print("  PosP: "); Serial.print(posP[0]);
  Serial.print("  PosI: "); Serial.print(posI[0]);
  Serial.print("  PosD: "); Serial.print(posD[0]);
  Serial.print("  Des_vel: ");  Serial.print(pos_output[0]);
  Serial.print("  Ctrl u: "); Serial.println(gimbal_yaw_iq);*/
  
  /*Serial.print("  PIT Pos Error: ");  Serial.print(prev_pos_error[1]);
  Serial.print("  PosP: "); Serial.print(posP[1]);
  Serial.print("  PosI: "); Serial.print(posI[1]);
  Serial.print("  PosD: "); Serial.print(posD[1]);
  Serial.print("  Des_vel: ");  Serial.print(pos_output[1]);
  Serial.print("  Ctrl u: ");  Serial.print(gimbal_pitch_iq);
  Serial.print("  gx: ");  Serial.println(gx_fil / pi * 180, 3);
  Serial.println(" ");*/

  Serial.print(" Yaw Angle: ");
  Serial.print(curr_ang_cnts[4]);
  Serial.print("  Pitch Angle: ");
  Serial.println(curr_ang_cnts[5]);
  Serial.print(curr_ang_cnts[0]);
  Serial.print("  ");
  Serial.print(curr_ang_cnts[1]);
  Serial.print("  ");
  Serial.print(curr_ang_cnts[2]);
  Serial.print("  ");
  Serial.println(curr_ang_cnts[3]);
  
  //Serial.print(" pos_output[1] "); Serial.print(pos_output[1]);
  //Serial.print("  Pitch_Vel: ");  Serial.println(gx_fil / pi * 180, 3);
  //Serial.print("  gx: ");  Serial.println(gx_fil / pi * 180, 3);
  //Serial.print("  dt: ");  Serial.println(dt*1000);
}


//-------------------------------MAIN LOGIC(TIMER)----------------------------------------------------------------------
void EncoderRead() {
  if (CANbus.available()) {
    while (CANbus.read(rx_message)) {
      curr_ang_cnts[rx_message.id - 0x201] = (uint16_t)(rx_message.buf[0] << 8) + (uint16_t)rx_message.buf[1];
    }
  }
}
//----------------------------------------------------------------------------------------------------------------------
void loop()
{
  //currentMillis = millis();
  IMURead();//Read IMU information
  ////Set_CM_Speed(gimbal_yaw_iq, gimbal_pitch_iq); //Output everything
  dt = 0.001;//(currentMillis - previousMillis) * 0.001;
  ////ctrl_loop(yaw_ang_ref, pitch_ang_ref); // Calculate for the control effort, prepare the outputs

//-----------------------------------comment from me and uncomment ////lines to recover to orginal one
  if (Serial.available()) {
    readIn = Serial.read(); //type an character in serial monitor
  }

//you have to be in Initial state (Press i) to go into Horizontal (Press h) or Vertival state (Press v)
  switch (state) {
    case Initial: //initial status --> pointing forward with barrel lifting in the middle
      ctrl_loop(335, 45);
      if (readIn == 'h') {
        state = Horizontal;
      }
      else if (readIn == 'v'){
        state = Vertical;
      }
      break;

    case Horizontal:
      ctrl_loop(yawMove, 45);
      counter ++;
      //increment by 5 degree every second
      if (counter == 300) {
        yawMove += horDir * 10;
        counter = 0;
      }
      //connecting upper limit and lower limit of the encoder
      if (yawMove >= 365) { //==
        yawMove = 0;
      }
      if (yawMove <= -5) { //==
        yawMove = 360;
      }
      //change direction when approaching limit (90 deg)
      if (horDir == 1 && yawMove == 60) { //65
        horDir = -1;
      }
      else if (horDir == -1 && yawMove == 240) { //245
        horDir = 1;
      }
      //ctrl_loop(270, 45);

      if (readIn == 'i') {
        yawMove = 335; horDir = 1; counter = 0; //reset every global variable
        state = Initial;
      }
      else if (readIn == 'v'){
        state = Vertical;
      }
      break;

    case Vertical:
      ctrl_loop(335, pitchMove);
      counter++;
      //increment by 5 degree per second
      if (counter == 300) {
        pitchMove += verDir * 5;
        counter = 0;
      }
      //connecting upper limit and lower limit of the encoder
      if (pitchMove == 365) {
        pitchMove = 0;
      }
      if (pitchMove == -5) {
        pitchMove = 360;
      }
      //change direction when approaching limit (90 deg)
      if (verDir == 1 && pitchMove == 65) {
        verDir = -1;
      }
      else if (verDir == -1 && pitchMove == 5) {
        verDir = 1;
      }
      //ctrl_loop(25,45);

      if (readIn == 'i') {
        pitchMove = 45; verDir = 1; counter = 0; //reset every global variable
        state = Initial;
      }
      else if (readIn == 'h'){
        state = Horizontal;
      }
      break;
  }
  //Set_CM_Speed(gimbal_yaw_iq, gimbal_pitch_iq); 
  Set_CM_Speed(0, 0); 
//-----------------------------------comment to me and uncomment ////lines to recover to orginal one
  debug();
  //previousMillis = currentMillis;

}
