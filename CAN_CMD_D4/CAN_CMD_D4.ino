#include <stdint.h>
#include "RemoteControl.h"
#include <FlexCAN.h>
#include <math.h>
//can setup----------------------------------------------
const int baudRate = 1000000;
FlexCAN CANbus(baudRate, 1);
static CAN_message_t tx_message, rx_message;
int16_t cm1_iq = 0, cm2_iq = 0, cm3_iq = 0, cm4_iq = 0;
uint16_t curr_ang_cnts[6] = {0, 0, 0, 0, 0, 0};
uint16_t curr_vel_cnts[6] = {0, 0, 0, 0, 0, 0};
//timer setup--------------------------------------------
IntervalTimer myTimer;
//remote control-----------------------------------------
char inputBuffer[18] = { 0 };
// a useless buffer to dump bad data
char uselessBuffer[18] = { 0 };
// Raw data when all keys are released. This is for data correction
byte idleData[] = { 0x0, 0x4, 0x20, 0x0, 0x1, 0xD8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }; // [5] can be 0x98, 0xD8, or 0x58
RC_Ctl_t RC_CtrlData;
char* output = (char*)malloc(100 * sizeof(char));
//PID parameters------------------------------------------
float dt, kp, ki, kd;
int16_t error, previousError = 0, P, I, D;
long previousMillis = 0;
unsigned long currentMillis;

volatile uint16_t old_ang[4], new_ang[4];
//volatile int16_t vel[4][3] = {{0, 0 ,0},{0, 0 ,0},{0, 0 ,0},{0, 0 ,0}};
//------------------------------------------------------------

int16_t yawP = 0, yawI = 0, yawD = 0, pitchP = 0, pitchI = 0, pitchD = 0;
int yaw_ang_ref = 335, pitch_ang_ref = 45;
int prev_yaw_ang_diff = 0, prev_pitch_ang_diff = 0;
int prev_yaw_cnt_diff = 0, prev_pitch_cnt_diff = 0;
int prev_yaw_cnt_diff2 = 0, prev_pitch_cnt_diff2 = 0;
int prev_cnt_diff = 0;

void setup() {
  Serial1.begin(100000, SERIAL_8E1);
  //Serial1.begin(9600);
  CANbus.begin();
  //if using enable pins on a transceiver they need to be set on
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //can setup
  tx_message.ext = 0;
  //tx_message_GM.ext = 0;
  tx_message.id = 0x200; //Chassis motor address
  //tx_message_GM.id = 0x1FF; //Gimbal motor address
  tx_message.len = 8;
  //tx_message_GM.len = 8;
  rx_message.ext = 0;
  rx_message.id = 0x202;
  rx_message.len = 8;

  //filter
  //CAN_filter(); //--

  //timer shit
  myTimer.begin(Get_CMD, 1000);
  //myTimer.begin(main_loop, 1000);
}

void EncoderRead(){
  if (CANbus.available()){
    while(CANbus.read(rx_message)){
      curr_ang_cnts[rx_message.id-0x201]=(uint16_t)(rx_message.buf[0] << 8) + (uint16_t)rx_message.buf[1];
      curr_vel_cnts[rx_message.id-0x201]=(int16_t)(rx_message.buf[2] << 8) + (int16_t)rx_message.buf[3];
    }
  }
}

void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
  tx_message.buf[0] = (uint8_t)(cm1_iq >> 8); //shift by 8 bits
  tx_message.buf[1] = (uint8_t)cm1_iq;
  tx_message.buf[2] = (uint8_t)(cm2_iq >> 8);
  tx_message.buf[3] = (uint8_t)cm2_iq;
  tx_message.buf[4] = (uint8_t)(cm3_iq >> 8);
  tx_message.buf[5] = (uint8_t)cm3_iq;
  tx_message.buf[6] = (uint8_t)(cm4_iq >> 8);
  tx_message.buf[7] = (uint8_t)cm4_iq;
  CANbus.write(tx_message);
}
/*
void Set_GM_Speed(int16_t gimbal_yaw_iq, int16_t gimbal_pitch_iq)
{
  tx_message_GM.buf[0] = (unsigned char)(gimbal_yaw_iq >> 8);
  tx_message_GM.buf[1] = (unsigned char)gimbal_yaw_iq;
  tx_message_GM.buf[2] = (unsigned char)(gimbal_pitch_iq >> 8);
  tx_message_GM.buf[3] = (unsigned char)gimbal_pitch_iq ;
  tx_message_GM.buf[4] = 0x00;
  tx_message_GM.buf[5] = 0x00;
  tx_message_GM.buf[6] = 0x00;
  tx_message_GM.buf[7] = 0x00;
  CANbus.write(tx_message_GM);
}
*/

float cal_vel(uint8_t motor_num) //need to consider the case when the count goes over 8191
{
  /*int16_t differ; 
  float avgVel;
  new_ang[motor_num]=curr_ang_cnts[motor_num]*360.0/8191;
  differ= old_ang[motor_num]-new_ang[motor_num];
  old_ang[motor_num]=new_ang[motor_num];
  
  vel[motor_num][0]= -differ/dt;
  avgVel = (vel[motor_num][0]+vel[motor_num][1]+vel[motor_num][2])/3; //average filter
  vel[motor_num][2] = vel[motor_num][1];
  vel[motor_num][1] = vel[motor_num][0];
  Serial.println(avgVel);
  return avgVel;
  */
  int16_t differ, vel;
  new_ang[motor_num]=curr_ang_cnts[motor_num]*360.0/8191;
  differ= old_ang[motor_num]-new_ang[motor_num];
  old_ang[motor_num]=new_ang[motor_num];
  vel = -differ/dt;
  //Serial.println(vel);
  return vel;
}

//--------interrupt-----------------------------------------------

void Get_CMD() {
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
  }
}


void Get_Dir(){
  float V_X = map(RC_CtrlData.rc.ch2, 364, 1684, -100, 100);
  float V_Y = map(RC_CtrlData.rc.ch3, 364, 1684, -100, 100);
  float omega = map(RC_CtrlData.rc.ch0, 364, 1684, -100, 100);
  
  //variables
  float velocity, theta;
  
  // Vw
  float wheel1x, wheel1y, wheel2x, wheel2y, wheel3x, wheel3y, wheel4x, wheel4y;
  float wheel1vb, wheel2vb, wheel3vb, wheel4vb, omegaWheel1, omegaWheel2, omegaWheel3, omegaWheel4;
  
  //constants
  const float radius1 = 1, radius2 = 1, radius3 = 1, radius4 = 1, wheelRadius = 1;
  const float phi1 = 45, phi2 = 45, phi3 = 45, phi4 = 45;
  const float pi = 4.0 * atan(1);
  velocity = sqrt(sq(V_X) + sq(V_Y));
  theta = ((atan2(V_Y, V_X))*180/pi);
  //velocity of 4 wheels @ the center
  wheel1x = (velocity * cos(theta * pi / 180)) - (radius1 * -omega * sin(phi1 * pi / 180));
  wheel1y = (velocity * sin(theta * pi / 180)) + (radius1 * -omega * cos(phi1 * pi / 180));
  wheel2x = ((velocity * cos(theta * pi / 180)) - (radius2 * -omega * sin(phi2 * pi / 180)));
  wheel2y = ((velocity * sin(theta * pi / 180)) - (radius2 * -omega * cos(phi2 * pi / 180)));
  wheel3x = ((velocity * cos(theta * pi / 180)) + (radius3 * -omega * sin(phi3 * pi / 180)));
  wheel3y = ((velocity * sin(theta * pi / 180)) + (radius3 * -omega * cos(phi3 * pi / 180)));
  wheel4x = ((velocity * cos(theta * pi / 180)) + (radius4 * -omega * sin(phi4 * pi / 180)));
  wheel4y = ((velocity * sin(theta * pi / 180)) - (radius4 * -omega * cos(phi4 * pi / 180)));
  //finding w of the wheel
  wheel1vb = (wheel1x / cos(45 * pi / 180));
  wheel2vb = -(wheel2x / cos(45 * pi / 180));
  wheel3vb = -(wheel3x / cos(45 * pi / 180));
  wheel4vb = (wheel4x / cos(45 * pi / 180));
  omegaWheel1 = (wheel1y - (wheel1vb * sin(45 * pi / 180))) / wheelRadius;
  omegaWheel2 = (wheel2y - (wheel2vb * sin(45 * pi / 180))) / wheelRadius;
  omegaWheel4 = (wheel3y - (wheel3vb * sin(45 * pi / 180))) / wheelRadius;
  omegaWheel3 = (wheel4y - (wheel4vb * sin(45 * pi / 180))) / wheelRadius;
  /*sprintf(output, "w1: %f, w2: %f,w3: %f,w4: %f", omegaWheel1, omegaWheel2,omegaWheel4,omegaWheel3);
  Serial.println(output);
  Serial.println(theta);
  Serial.println("");
  */
  int16_t cm1_iq = -1000 * omegaWheel1 / 100;
  int16_t cm2_iq = 1000 * omegaWheel2 / 100;
  int16_t cm3_iq = 1000 * omegaWheel3 / 100;
  int16_t cm4_iq = -1000 * omegaWheel4 / 100;

  //update output with PID
  int16_t cm1_iq_tuned = getPID(cm1_iq,curr_vel_cnts[0]); //cal_vel(0)); //need to use curr_ang_cnts[] to compute velocities
  int16_t cm2_iq_tuned = getPID(cm2_iq,curr_vel_cnts[1]);
  int16_t cm3_iq_tuned = getPID(cm3_iq,curr_vel_cnts[2]);
  int16_t cm4_iq_tuned = getPID(cm4_iq,curr_vel_cnts[3]);

  Serial.print("Ref Output:     ");
  Serial.print("cm1_iq: ");
  Serial.print(cm1_iq);
  Serial.print(" cm2_iq: ");
  Serial.print(cm2_iq);
  Serial.print(" cm3_iq: ");
  Serial.print(cm3_iq);
  Serial.print(" cm4_iq: ");
  Serial.println(cm4_iq);
  
  Set_CM_Speed(cm1_iq_tuned, cm2_iq_tuned, cm3_iq_tuned, cm4_iq_tuned);
  
  Serial.print("Sensor Reading: ");
  Serial.print("cm1_iq: ");
  Serial.print(curr_vel_cnts[0]); //cal_vel(0
  Serial.print(" cm2_iq: ");
  Serial.print(curr_vel_cnts[1]);
  Serial.print(" cm3_iq: ");
  Serial.print(curr_vel_cnts[2]);
  Serial.print(" cm4_iq: ");
  Serial.println(curr_vel_cnts[3]); 
  /*
  cal_vel(0);
  cal_vel(1);
  cal_vel(2);
  cal_vel(3);*/
}

int16_t getPID(int16_t desired, int16_t sensorReading){
  error = desired - sensorReading;
  kp = 1.0;
  ki = 0.0;
  kd = 0.0;
  P = kp*error;
  I += ki*error*dt;
  D = kd*(error - previousError) / dt;
  previousError = error;
  return P + I + D;
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
/*
void ctrl_loop(int yaw, int pitch){
  int16_t gimbal_yaw_iq = gimbalYawPID(yaw);
  int16_t gimbal_pitch_iq = gimbalPitchPID(pitch);
  //Set_CM_Speed(gimbal_yaw_iq, gimbal_pitch_iq);
  Set_GM_Speed(gimbal_yaw_iq, gimbal_pitch_iq);


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

void main_loop(){
  Get_CMD();
  currentMillis = millis();
  dt = (currentMillis - previousMillis)*0.001;
  EncoderRead();
  ctrl_loop(yaw_ang_ref, pitch_ang_ref);
  previousMillis = currentMillis;
  Serial.print("");
}
*/

void loop()
{
  
  currentMillis = millis();
  dt = (currentMillis - previousMillis)*0.001;
  EncoderRead();
 // ctrl_loop(yaw_ang_ref, pitch_ang_ref);
  Get_Dir();
  delay(50);
  previousMillis = currentMillis;
  Serial.print(RC_CtrlData.rc.ch2);
  Serial.print(" ");
  Serial.print(RC_CtrlData.rc.ch3);
  Serial.print(" ");
  Serial.print(RC_CtrlData.rc.ch0);
  Serial.println("\n");
}





