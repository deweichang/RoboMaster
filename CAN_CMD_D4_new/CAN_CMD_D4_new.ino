#include <stdint.h>
#include "RemoteControl.h"
#include <FlexCAN.h>
//can setup----------------------------------------------
const int baudRate = 1000000;
FlexCAN CANbus(baudRate, 1);
static CAN_message_t tx_message, rx_message;
int16_t cm1_iq = 0;
int16_t cm2_iq = 0;
int16_t cm3_iq = 0;
int16_t cm4_iq = 0;

uint16_t curr_ang_cnts[6] = {0, 0, 0, 0, 0, 0};
uint16_t curr_vel_cnts[6] = {0, 0, 0, 0, 0, 0};
//--timer setup------------------------------------------
IntervalTimer myTimer;
//remote control-----------------------------------------
char inputBuffer[18] = { 0 };
// a useless buffer to dump bad data
char uselessBuffer[18] = { 0 };
// Raw data when all keys are released. This is for data correction
byte idleData[] = { 0x0, 0x4, 0x20, 0x0, 0x1, 0xD8, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 }; // [5] can be 0x98, 0xD8, or 0x58
RC_Ctl_t RC_CtrlData;
char* output = (char*)malloc(100 * sizeof(char));
//------------------------------------------------------------

void setup() {
  Serial1.begin(100000, SERIAL_8E1);
  CANbus.begin();
  //if using enable pins on a transceiver they need to be set on
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  //can setup
  tx_message.ext = 0;
  tx_message.id = 0x200;
  tx_message.len = 8;
  rx_message.ext = 0;
  rx_message.id = 0x202;
  rx_message.len = 8;

  //timer shit
  myTimer.begin(Get_CMD, 1000);
}

void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
  tx_message.buf[0] = (uint8_t)(cm1_iq >> 8);
  tx_message.buf[1] = (uint8_t)cm1_iq;
  tx_message.buf[2] = (uint8_t)(cm2_iq >> 8);
  tx_message.buf[3] = (uint8_t)cm2_iq;
  tx_message.buf[4] = (uint8_t)(cm3_iq >> 8);
  tx_message.buf[5] = (uint8_t)cm3_iq;
  tx_message.buf[6] = (uint8_t)(cm4_iq >> 8);
  tx_message.buf[7] = (uint8_t)cm4_iq;
  CANbus.write(tx_message);
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

    //print result
    //sprintf(output, "C2: %d, C3: %d", RC_CtrlData.rc.ch2, RC_CtrlData.rc.ch3);
    //Serial.println(output);

  }
}
/*
  void EncoderRead()
  {
  if (CANbus.available())
  {
    CANbus.read(rx_message);
    //Serial.write(rx_message.buf[3]);
    uint16_t vel = (uint8_t)(rx_message.buf[2] << 8) + (uint8_t)rx_message.buf[3];
    Serial.print(vel, DEC);
    Serial.println();
    //Serial.write(vel);
  }
  }
*/
void Get_Dir() {
  float V_X = map(RC_CtrlData.rc.ch2, 364, 1684, -100, 100);
  float V_Y = map(RC_CtrlData.rc.ch3, 364, 1684, -100, 100);
  float omega = map(RC_CtrlData.rc.ch0, 364, 1684, -100, 100);
  
  //variables
  float velocity;
  float theta;
  
  // Vw
  float wheel1x;
  float wheel1y;
  float wheel2x;
  float wheel2y;
  float wheel3x;
  float wheel3y;
  float wheel4x;
  float wheel4y;
  float vx;
  float vy;

  float wheel1vb;
  float wheel2vb;
  float wheel3vb;
  float wheel4vb;
  float omegaWheel1;
  float omegaWheel2;
  float omegaWheel3;
  float omegaWheel4;
  //constants
  const float radius1 = 1;
  const float radius2 = 1;
  const float radius3 = 1;
  const float radius4 = 1;
  const float wheelRadius = 1;
  const float phi1 = 45;
  const float phi2 = 45;
  const float phi3 = 45;
  const float phi4 = 45;
  const float pi = 4.0 * atan(1);
  velocity = sqrt(sq(V_X) + sq(V_Y));
  //theta = ((atan2(V_Y / V_X)) * pi / 180);
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
  sprintf(output, "w1: %f, w2: %f,w3: %f,w4: %f", omegaWheel1, omegaWheel2,omegaWheel4,omegaWheel3);
  Serial.println(output);
  Serial.println(theta);
  Serial.println("");
  int16_t cm1_iq = -4000 * omegaWheel1 / 100; //infantry: 1000, engineer: 2000, hero: 4000
  int16_t cm2_iq = 4000 * omegaWheel2 / 100;
  int16_t cm3_iq = 4000 * omegaWheel3 / 100;
  int16_t cm4_iq = -4000 * omegaWheel4 / 100;
  Set_CM_Speed(cm1_iq, cm2_iq, cm3_iq, cm4_iq);
  //Set_CM_Speed(-1000, 1000, 1000, -1000);
  


}

void EncoderRead(){
  if (CANbus.available()){
    while(CANbus.read(rx_message)){
      curr_ang_cnts[rx_message.id-0x201]=(uint16_t)(rx_message.buf[0] << 8) + (uint16_t)rx_message.buf[1];
      curr_vel_cnts[rx_message.id-0x201]=(int16_t)(rx_message.buf[2] << 8) + (int16_t)rx_message.buf[3];
    }
  }
}

void loop()
{
  Get_Dir();
  EncoderRead();
  delay(50);
  /*Serial.print(RC_CtrlData.rc.ch2);
  Serial.print(" ");
  Serial.print(RC_CtrlData.rc.ch3);
  Serial.print(" ");
  Serial.print(RC_CtrlData.rc.ch0);
  Serial.println("\n");*/

  Serial.print("1: ");
  Serial.print(curr_vel_cnts[0]);
  Serial.print("  2: ");
  Serial.print(curr_vel_cnts[1]);
  Serial.print("  3: ");
  Serial.print(curr_vel_cnts[2]);
  Serial.print("  4: ");
  Serial.println(curr_vel_cnts[3]);
  
  /*
    int16_t cm1_iq = -1000 * Ctrl_Gain / 100;
    int16_t cm2_iq = 1000 * Ctrl_Gain / 100;
    int16_t cm3_iq = 1000 * Ctrl_Gain / 100;
    int16_t cm4_iq = -1000 * Ctrl_Gain / 100;
  */

}





