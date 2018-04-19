#include <stdint.h>
#include <math.h>
#include "MPU9250.h"
#include "FlexCAN.h"
//#include "RemoteControl.h"
//------------DEFINE VARIABLES---------------------------------------------
const float pi = 4.0 * atan(1);
const int baudRate = 1000000;
int16_t gimbal_yaw_iq = 0;
int16_t gimbal_pitch_iq = 0;
uint16_t curr_ang_cnts[6] = {0, 0, 0, 0, 0, 0};
uint16_t curr_vel_cnts[6] = {0, 0, 0, 0, 0, 0};
int LoopPeriod = 1000;// * microsecond
long previousMillis = 0;
unsigned long currentMillis;
float dt;
int prev_yaw_ang_diff = 0, prev_pitch_ang_diff = 0;
int prev_yaw_cnt_diff = 0, prev_pitch_cnt_diff = 0;
int prev_yaw_cnt_diff2 = 0, prev_pitch_cnt_diff2 = 0;
int prev_cnt_diff = 0;
int16_t posP[2] = {0, 0}, posI[2] = {0, 0}, posD[2] = {0, 0}, velP[2] = {0, 0}, velI[2] = {0, 0};
int prev_pos_error[2] = {0, 0};
float pos_output[2] = {0, 0};
float gxx[3] = {0, 0, 0}, gzz[3] = {0, 0, 0};
float gx_fil = 0, gz_fil = 0;
int yaw_ang_ref = 335, pitch_ang_ref = 45;
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
    gxx[0] = gx; gx_fil = (gxx[0] + gxx[1] + gxx[2]) / 3; gxx[2] = gxx[1]; gxx[1] = gxx[0];
    gzz[0] = gz; gz_fil = (gzz[0] + gzz[1] + gzz[2]) / 3; gzz[2] = gzz[1]; gzz[1] = gzz[0];
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
  //Serial.print("  Des_vel: ");  Serial.println(pos_output[1]);
  Serial.print("  Ctrl u: ");  Serial.print(gimbal_pitch_iq);
  Serial.print("  Pos Error: ");  Serial.println((float)prev_pos_error[1]);
  //Serial.print("  Pitch_Vel: ");  Serial.println(gx_fil / pi * 180, 3);
  //Serial.print("  gz: ");  Serial.println(gz_fil / pi * 180, 3);
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
  Set_CM_Speed(gimbal_yaw_iq, gimbal_pitch_iq); //Output everything
  dt = 0.001;//(currentMillis - previousMillis) * 0.001;
  ctrl_loop(yaw_ang_ref, pitch_ang_ref); // Calculate for the control effort, prepare the outputs
  debug();
  //previousMillis = currentMillis;
}
