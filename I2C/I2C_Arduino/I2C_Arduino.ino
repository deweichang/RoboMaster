#include <Wire.h>
#include "I2C.h"

int newData = 0;
i2c_data_t data;
I2C<i2c_data_t> i2c(0x04);

void setup() {
	Serial.begin(9600);
	Wire.onRequest(requestHandler);
  Wire.onReceive(receiveHandler);
}

void loop() {
  if(newData) {
    Serial.println(data.x);
    Serial.println(data.y);
    Serial.println(data.z);
    Serial.println(data.angular_velocity_x);
    Serial.println(data.angular_velocity_y);
    Serial.println(data.velocity_z);
    //data.encoder_front_left = 200;
    //data.encoder_rear_left = 200;
    //data.encoder_front_right = -200;
    //data.encoder_rear_right = -200;
    newData = 0;
  }
}

void receiveHandler(int count) {
  i2c.read(data);
  newData = 1;
}

void requestHandler() {
  i2c.write(data);
}
