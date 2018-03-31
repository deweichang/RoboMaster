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
int prev_cnt_diff = 0;

int Yaw = 335, Pitch = 45;

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

  //filter
  CAN_filter_t mask;
  mask.id = 0x7FF;
  mask.ext = 0;
  mask.rtr = 0;

  CAN_filter_t Filter1;
  Filter1.id = 0x201;
  Filter1.ext = 0;
  Filter1.rtr = 0;
  CAN_filter_t Filter2;
  Filter2.id = 0x202;
  Filter2.ext = 0;
  Filter2.rtr = 0;
  CAN_filter_t Filter3;
  Filter3.id = 0x203;
  Filter3.ext = 0;
  Filter3.rtr = 0;
  CAN_filter_t Filter4;
  Filter4.id = 0x204;
  Filter4.ext = 0;
  Filter4.rtr = 0;
  CAN_filter_t Filter5;
  Filter5.id = 0x205;
  Filter5.ext = 0;
  Filter5.rtr = 0;
  CAN_filter_t Filter6;
  Filter6.id = 0x206;
  Filter6.ext = 0;
  Filter6.rtr = 0;
  CAN_filter_t Filter7;
  Filter7.id = 0x206;
  Filter7.ext = 0;
  Filter7.rtr = 0;
  CAN_filter_t Filter8;
  Filter8.id = 0x206;
  Filter8.ext = 0;
  Filter8.rtr = 0;
  
  CANbus.begin(mask);
  CANbus.setFilter(Filter1, 0);
  CANbus.setFilter(Filter2, 1);
  CANbus.setFilter(Filter3, 2);
  CANbus.setFilter(Filter4, 3);
  CANbus.setFilter(Filter5, 4);
  CANbus.setFilter(Filter6, 5);
  CANbus.setFilter(Filter7, 6);
  CANbus.setFilter(Filter8, 7);


  //timer shit
  myTimer.begin(main_loop, 10000);
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

void EncoderRead()
{
  if (CANbus.available())
  {
    while(CANbus.read(rx_message)){
      curr_ang_cnts[rx_message.id-0x201]=(uint16_t)(rx_message.buf[0] << 8) + (uint16_t)rx_message.buf[1];
    }
  }
}

int16_t motor_pos_ctrl(int motor_id,float ang_ref){
  float kp = 10;
  float kd = 5.0;
  float ctrl_scale = 10; //7.5
  int cnt_ref=(int)ang_ref*8191.0/360;
  int cnt_diff = (cnt_ref-curr_ang_cnts[motor_id]+4096)%8191-4096;
  
  if (motor_id == 5){
    if (curr_ang_cnts[5]*360.0/8191 > 65){
      kp = 50;
    }
    else {
      kp = 25;
    }

    if (cnt_diff > 4096){ //delta theta = cnt_diff or curr_ang_cnts - last_ang_cnts ??
      cnt_diff -= 8191;
    }
    else if (cnt_diff < -4096){
      cnt_diff += 8191;
    }
    
    ctrl_scale = kp - kd*(1 + prev_cnt_diff)/dt;
  }
  
  if(abs(cnt_diff)>4096){
    cnt_diff=cnt_diff+8191;
  }
  float ang_diff=-cnt_diff*360.0/8191;
  int16_t ctrl=0;
  if(ang_diff>-0.1 && ang_diff<0.1){
    ctrl=0;
  }else{
    ctrl=-ang_diff*ctrl_scale; //**
  }

  if(ctrl>0){ctrl=ctrl+70;}
  else if(ctrl<0){ctrl=ctrl-47;}

  prev_cnt_diff = cnt_diff;
  
  return ctrl;
}

void ctrl_loop(){
  int16_t gimbal_yaw_iq = motor_pos_ctrl(4,335);
  int16_t gimbal_pitch_iq = motor_pos_ctrl(5,45);
  Set_CM_Speed(gimbal_yaw_iq, gimbal_pitch_iq);

  float ang_act_yaw=curr_ang_cnts[4]*360.0/8191;
  int cnt_ref_yaw=(int)335*8191.0/360;
  int cnt_diff_yaw = (cnt_ref_yaw-curr_ang_cnts[4]+4096)%8191-4096;
  if(abs(cnt_diff_yaw)>4096){
    cnt_diff_yaw=cnt_diff_yaw+8191;
  }
  float ang_diff_yaw=-cnt_diff_yaw*360.0/8191;

  float ang_act_pitch=curr_ang_cnts[5]*360.0/8191;
  int cnt_ref_pitch=(int)45*8191.0/360;
  int cnt_diff_pitch = (cnt_ref_pitch-curr_ang_cnts[5]+4096)%8191-4096;
  if(abs(cnt_diff_pitch)>4096){
    cnt_diff_pitch=cnt_diff_pitch+8191;
  }
  float ang_diff_pitch=-cnt_diff_pitch*360.0/8191;
  
  Serial.print("ang_diff_yaw: ");
  Serial.print(ang_diff_yaw);
  Serial.print(" ");
  //Serial.print(cnt_diff);
  //Serial.print(" ");
  Serial.print("ang_act_yaw: ");
  Serial.print(ang_act_yaw);
  Serial.print(" ");
  Serial.print("ang_diff_pitch: ");
  Serial.print(ang_diff_pitch);
  Serial.print(" ");
  Serial.print("ang_act_pitch: ");
  Serial.print(ang_act_pitch);
  Serial.print(" ");
  Serial.print((float)gimbal_yaw_iq);
  Serial.print(" ");
  Serial.print((float)gimbal_pitch_iq);
  Serial.print("\n");
}

void main_loop(){
  currentMillis = millis();
  dt = currentMillis - previousMillis;
  EncoderRead();
  ctrl_loop();
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
