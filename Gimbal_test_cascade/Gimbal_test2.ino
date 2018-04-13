#include <stdint.h>
#include <math.h>
//#include "RemoteControl.h"
#include <FlexCAN.h>
//can setup----------------------------------------------
const int baudRate = 1000000;
FlexCAN CANbus(baudRate, 1);
static CAN_message_t tx_message, rx_message;
int16_t gimbal_yaw_iq = 0;
int16_t gimbal_pitch_iq = 0;
uint16_t curr_ang_cnts[6] = {0, 0, 0, 0, 0, 0};
uint16_t curr_vel_cnts[6] = {0, 0, 0, 0, 0, 0};
long previousMillis = 0;
unsigned long currentMillis;
float dt;
int prev_yaw_ang_diff = 0, prev_pitch_ang_diff = 0;
int prev_yaw_cnt_diff = 0, prev_pitch_cnt_diff = 0;
int prev_yaw_cnt_diff2 = 0, prev_pitch_cnt_diff2 = 0;
int prev_cnt_diff = 0;
int16_t yawP = 0, yawI = 0, yawD = 0, pitchP = 0, pitchI = 0, pitchD = 0;

int yaw_ang_ref = 335, pitch_ang_ref = 45;

//IMU setup----------------------------------------------
#include "MPU9250.h"

// an MPU9250 object with its I2C address 
// of 0x68 (ADDR to GRND) and on Teensy bus 0
MPU9250 IMU(0x68, 0);

//float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
int beginStatus;

//--timer setup------------------------------------------
IntervalTimer myTimer;
/*//remote control-----------------------------------------
  char inputBuffer[18] = { 0 };
  // a useless buffer to dump bad data
  char uselessBuffer[18] = { 0 };
  // Raw data when all keys are released. This is for data correction
  byte idleData[] = { 0x0, 0x4, 0x20, 0x0, 0x1, 0xD8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }; // [5] can be 0x98, 0xD8, or 0x58
  RC_Ctl_t RC_CtrlData;
  char* output = (char*)malloc(100 * sizeof(char));
  //------------------------------------------------------------
*/
void setup() {
  //Serial1.begin(100000, SERIAL_8E1);
  Serial1.begin(9600);

  //CANbus.begin();
  //if using enable pins on a transceiver they need to be set on
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //can setup(gimbal)
  tx_message.ext = 0;
  tx_message.id = 0x1FF;
  tx_message.len = 8;
  rx_message.ext = 0;
  rx_message.id = 0x202;
  rx_message.len = 8;

  CAN_filter();

  //timer shit
  myTimer.begin(main_loop, 10000);

  //IMU
  // serial to display data
  Serial.begin(115200);

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
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
//--------interrupt-----------------------------------------------

/*void Get_CMD() {
  if (Serial1.available() >= 18) {
    Serial1.readBytes(inputBuffer, 18);
    //data correction
    int count = 0;
    int i = 0;
    for (; i < 18; i++) {
      count = 0;
      for (int j = 0; j < 18 && (idleData[j] == inputBuffer[(i + j) % 18] || (j == 5 && (inputBuffer[(i + j) % 18] == 0x98 || inputBuffer[(i + j) % 18] == 0x58))); j++) {
        count++;
      }
      if (count == 18) break;
    }
    if (count == 18 && i > 0) {
      Serial.print(i);
      Serial.println(" bits to correct");
      // dump i bits in the buffer as bad data
      while (Serial1.available() < i) {}
      Serial1.readBytes(uselessBuffer, i);
      return;
    }
    // convert raw data into meaningful data
    RC_CtrlData.rc.ch0 = ((int16_t)inputBuffer[0] | ((int16_t)inputBuffer[1] << 8)) & 0x07FF;
    RC_CtrlData.rc.ch1 = (((int16_t)inputBuffer[1] >> 3) | ((int16_t)inputBuffer[2] << 5)) & 0x07FF;
    RC_CtrlData.rc.ch2 = (((int16_t)inputBuffer[2] >> 6) | ((int16_t)inputBuffer[3] << 2) |
                          ((int16_t)inputBuffer[4] << 10)) & 0x07FF;
    RC_CtrlData.rc.ch3 = (((int16_t)inputBuffer[4] >> 1) | ((int16_t)inputBuffer[5] << 7)) & 0x07FF;
    RC_CtrlData.rc.s1 = ((inputBuffer[5] >> 4) & 0x000C) >> 2;
    RC_CtrlData.rc.s2 = ((inputBuffer[5] >> 4) & 0x0003);
    RC_CtrlData.mouse.x = ((int16_t)inputBuffer[6]) | ((int16_t)inputBuffer[7] << 8);
    RC_CtrlData.mouse.y = ((int16_t)inputBuffer[8]) | ((int16_t)inputBuffer[9] << 8);
    RC_CtrlData.mouse.z = ((int16_t)inputBuffer[10]) | ((int16_t)inputBuffer[11] << 8);
    RC_CtrlData.mouse.press_l = inputBuffer[12];
    RC_CtrlData.mouse.press_r = inputBuffer[13];
    RC_CtrlData.key.v = ((int16_t)inputBuffer[14]) | ((int16_t)inputBuffer[15] << 8);
    //print result
    sprintf(output, "C3: %d", RC_CtrlData.rc.ch3);
    Serial.println(output);
  }
  }
*/

void EncoderRead(){
  if (CANbus.available()){
    while(CANbus.read(rx_message)){
      curr_ang_cnts[rx_message.id-0x201]=(uint16_t)(rx_message.buf[0] << 8) + (uint16_t)rx_message.buf[1];
    }
  }
}

float angle_compute(int motor_id, int ang_ref, int k){
  float ang_act = curr_ang_cnts[motor_id]*360.0/8191;
  int cnt_ref = (int)ang_ref*8191.0/360;
  int cnt_diff = (cnt_ref - curr_ang_cnts[motor_id] + 4096)%8191 - 4096;
  if(abs(cnt_diff) > 4096){
    cnt_diff = cnt_diff + 8191;
  }
  float ang_diff = -cnt_diff*360.0/8191;

  if (k == 1){
    return ang_act;
  }
  else if (k == 2){
    return ang_diff;
  }
}

void ctrl_loop(int yaw, int pitch){
  //int16_t gimbal_yaw_iq = gimbalYawPID(yaw);
  //int16_t gimbal_pitch_iq = gimbalPitchPID(pitch);
  int16_t gimbal_yaw_iq = CascadeControlYaw(yaw);
  int16_t gimbal_pitch_iq = CascadeControlPitch(pitch);
  Set_CM_Speed(gimbal_yaw_iq, 0); //gimbal_pitch_iq

/*
  float ang_act_yaw=curr_ang_cnts[4]*360.0/8191;
  int cnt_ref_yaw = (int)yaw*8191.0/360;
  int cnt_diff_yaw = (cnt_ref_yaw-curr_ang_cnts[4]+4096)%8191-4096;
  if(abs(cnt_diff_yaw)>4096){
    cnt_diff_yaw=cnt_diff_yaw+8191;
  }
  float ang_diff_yaw=-cnt_diff_yaw*360.0/8191;

  float ang_act_pitch=curr_ang_cnts[5]*360.0/8191;
  int cnt_ref_pitch = (int)pitch*8191.0/360;
  int cnt_diff_pitch = (cnt_ref_pitch-curr_ang_cnts[5]+4096)%8191-4096;
  if(abs(cnt_diff_pitch)>4096){
    cnt_diff_pitch=cnt_diff_pitch+8191;
  }
  float ang_diff_pitch=-cnt_diff_pitch*360.0/8191;
*/

  Serial.print("YAW: ang_act: ");
  Serial.print(angle_compute(4, yaw, 1));
  Serial.print(" ang_diff: ");
  Serial.print(angle_compute(4, yaw, 2));
  Serial.print(" P: ");
  Serial.print(yawP);
  Serial.print(" I: ");
  Serial.print(yawI);
  Serial.print(" D: ");
  Serial.print(yawD);
  Serial.print(" Total: ");
  Serial.println(yawP + yawI + yawD);
  Serial.print("PITCH: ang_act: ");
  Serial.print(angle_compute(5, pitch, 1));
  Serial.print(" ang_diff: ");
  Serial.print(angle_compute(5, pitch, 2));
  Serial.print(" P: ");
  Serial.print(pitchP);
  Serial.print(" I: ");
  Serial.print(pitchI);
  Serial.print(" D: ");
  Serial.print(pitchD);
  Serial.print(" Total: ");
  Serial.println(pitchP + pitchI + pitchD);
  Serial.println("");

}

void IMUread(){
  if(beginStatus < 0) {
    delay(1000);
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    delay(10000);
  }
  else{
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    /*Serial.print(gx,6);
    Serial.print("\t");
    Serial.print(gy,6);
    Serial.print("\t");
    Serial.print(gz,6);
    Serial.print("\n");*/
    
    delay(50);
  }
}

void main_loop(){
  
  currentMillis = millis();
  if (Serial.available() > 0){
    int r = Serial.read();
    if (r == 'q'){
      pitch_ang_ref = 10;
      yaw_ang_ref = 300;
    }
    else if (r == 'w'){
      pitch_ang_ref = 20;
    }
    else if (r == 'e'){
      pitch_ang_ref = 30;
    }
    else if (r == 'r'){
      pitch_ang_ref = 45;
    }
    else if (r == 't'){
      pitch_ang_ref = 50;
    }
    else if (r == 'y'){
      pitch_ang_ref = 60;
      yaw_ang_ref = 5;
    }
  }
  dt = (currentMillis - previousMillis)*0.001;
  EncoderRead();
  ctrl_loop(yaw_ang_ref, pitch_ang_ref);
  previousMillis = currentMillis;
}

void loop()
{
  /*
  int16_t gimbal_yaw_iq = motor_pos_ctrl(4,45);
  int16_t gimbal_pitch_iq = 0;
  Set_CM_Speed(gimbal_yaw_iq, 0);
  float ang_act=curr_ang_cnts[4]*360.0/8191;
  int cnt_ref=(int)45*8191.0/360;
  int cnt_diff = (cnt_ref-curr_ang_cnts[4]+4096)%8191-4096;
  if(abs(cnt_diff)>4096){
    cnt_diff=cnt_diff+8191;
  }
  float ang_diff=-cnt_diff*360.0/8191;
  
  Serial.print(ang_diff);
  Serial.print(" ");
  Serial.print(ang_act);
  Serial.print(" ");
  Serial.print((float)gimbal_yaw_iq);
  Serial.print("\n");
  */
  /*
  Serial.print(curr_angs[0]);
  Serial.print(" ");
  Serial.print(curr_angs[1]);
  Serial.print(" ");
  Serial.print(curr_angs[2]);
  Serial.print(" ");
  Serial.print(curr_angs[3]);
  Serial.print(" ");
  Serial.print(curr_angs[4]);
  Serial.print(" ");
  Serial.print(curr_angs[5]);
  Serial.print("\n");
  */
}
