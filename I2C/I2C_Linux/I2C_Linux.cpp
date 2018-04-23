#include <iostream>
#include <stdlib.h>
#include "I2C.h"
using namespace std;

int main() {
	i2c_data_t data;
	data.encoder_front_left = 100;
	data.encoder_rear_left = 100;
	data.encoder_front_right = -100;
	data.encoder_rear_right = -100;
	char device[20];
	sprintf(device, "/dev/i2c-1");
    I2C<i2c_data_t> i2c(device, 0x04);
    i2c.send(data);
    sleep(1);
    i2c.request(data);
    cout << "received:" << endl;
    cout << data.encoder_front_left << endl;
    cout << data.encoder_rear_left << endl;
    cout << data.encoder_front_right << endl;
    cout << data.encoder_rear_right << endl;
    return 0;
}
