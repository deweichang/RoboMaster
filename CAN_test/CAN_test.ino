#include <stdint.h>
#include <FlexCAN.h>
const int baudRate = 50000; //50000
const int delayTime = 50;
FlexCAN CANbus(baudRate,1);
static CAN_message_t tx_message, rx_message;
 
int16_t cm1_iq = 0;
int16_t cm2_iq = 10000;
int16_t cm3_iq = 0;
int16_t cm4_iq = 0;
static uint8_t hex[17] = "0123456789abcdef";
 
void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);
 
 
 
 
// -------------------------------------------------------------
static void hexDump(uint8_t dumpLen, uint8_t *bytePtr)
{
  uint8_t working;
  while ( dumpLen-- ) {
    working = *bytePtr++;
    Serial.write( hex[ working >> 4 ] );
    Serial.write( hex[ working & 15 ] );
  }
  Serial.write('\r');
  Serial.write('\n');
}
 
 
// -------------------------------------------------------------
 
void setup() {
  delay(1000);
  Serial.println(F("CAN Test"));
  CANbus.begin();
 
  //if using enable pins on a transceiver they need to be set on
  pinMode(13,OUTPUT);
  digitalWrite(13,HIGH);
}
 
void Set_CM_Speed(int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
  tx_message.ext = 0;
  tx_message.id = 0x200;
  tx_message.len = 8;
 
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
 
void loop() {
  rx_message.ext = 0;
  rx_message.id = 0x202;
  rx_message.len = 8;
 
  if (CANbus.available())
  {
    CANbus.read(rx_message);
    Serial.print("CAN bus readin: "); hexDump(8, rx_message.buf);
 
  }
 
  Set_CM_Speed(cm1_iq, cm2_iq, cm3_iq, cm4_iq);
  delay(1000);
  Set_CM_Speed(cm1_iq, 0, cm3_iq, cm4_iq);
 
}
