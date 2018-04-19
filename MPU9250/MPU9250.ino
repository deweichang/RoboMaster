/*
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com
2016-10-10 
Copyright (c) 2016 Bolder Flight Systems
Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
//#include <Filters.h>

// an MPU9250 object with its I2C address 
// of 0x68 (ADDR to GRND) and on Teensy bus 0
MPU9250 IMU(0x68, 0);

float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
float gx_new = 0, gy_new = 0, gz_new = 0, gx_prev = 0, gy_prev = 0, gz_prev = 0;
float gx_fil, gy_fil, gz_fil;
float gxx[3] = {0, 0, 0};
int beginStatus;
float filterFrequency = 10000.0;

void setup() {
  // serial to display data
  Serial.begin(115200);

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
}

void loop() {
  if(beginStatus < 0) {
    delay(1000);
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    delay(10000);
  }
  else{
    /* get the individual data sources */
    /* This approach is only recommended if you only
     *  would like the specified data source (i.e. only
     *  want accel data) since multiple data sources
     *  would have a time skew between them.
     */
    // get the accelerometer data (m/s/s)
    IMU.getAccel(&ax, &ay, &az);
  
    // get the gyro data (rad/s)
    IMU.getGyro(&gx, &gy, &gz);
  
    // get the magnetometer data (uT)
    //IMU.getMag(&hx, &hy, &hz);
  
    // get the temperature data (C)
    //IMU.getTemp(&t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    /* get multiple data sources */
    /* In this approach we get data from multiple data
     *  sources (i.e. both gyro and accel). This is 
     *  the recommended approach since there is no time
     *  skew between sources - they are all synced.
     *  Demonstrated are:
     *  1. getMotion6: accel + gyro
     *  2. getMotion7: accel + gyro + temp
     *  3. getMotion9: accel + gyro + mag
     *  4. getMotion10: accel + gyro + mag + temp
     */
  
     /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    // get the magnetometer data (uT)
    //IMU.getMag(&hx, &hy, &hz);
  
    // get the temperature data (C)
    //IMU.getTemp(&t);

    //Exponential fiter
    /*gx_new = 0.7*gx + 0.3*gx_prev;
    gy_new = 0.7*gy + 0.3*gy_prev;
    gz_new = 0.7*gz + 0.3*gz_prev;
    gx_prev = gx_new;
    gy_prev = gy_new;
    gz_prev = gz_new;*/
    
    //FilterOnePole lowpassFilter( LOWPASS, filterFrequency );
    /*gxx = gx;
    gyy = gy;
    gzz = gz;
    gx_fil = lowpassFilter.input(gxx);
    gy_fil = lowpassFilter.input(gyy);
    gz_fil = lowpassFilter.input(gzz);*/

    //moving average filter
    gxx[0] = gx;
    gx_fil = (gxx[0] + gxx[1] + gxx[2]) / 3;
    gxx[2] = gxx[1];
    gxx[1] = gxx[0];

    

  
    // print the data
    printData();
  
    // delay a frame
    delay(50);

    /* getMotion7 */
    // get the accel (m/s/s), gyro (rad/s), and temperature (C) data
    //IMU.getMotion7(&ax, &ay, &az, &gx, &gy, &gz, &t);
    
    // get the magnetometer data (uT)
    //IMU.getMag(&hx, &hy, &hz);
  
    // print the data
    //printData();
  
    // delay a frame
    //delay(50);
  
    /* getMotion9 */
    // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT) data
    //IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
  
    // get the temperature data (C)
    //IMU.getTemp(&t);
  
    // print the data
    //printData();
  
    // delay a frame
    //delay(50);
  
    // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT), and temperature (C) data
    //IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
  
    // print the data
    //printData();
  
    // delay a frame
    //delay(50);
    
  }
}

void printData(){

  // print the data
  /*Serial.print(ax,6);
  Serial.print("\t");
  Serial.print(ay,6);
  Serial.print("\t");
  Serial.print(az,6);
  Serial.print("\t");*/
  double pi = 4*atan(1);
  Serial.print(gx_fil*180/pi,6);
  Serial.print("\t");
  Serial.print(gx*180/pi,6);
  Serial.println("");
  /*Serial.print(gy_new,6);
  Serial.print("\t");
  Serial.print(gz_new,6);
  Serial.print("\n");*/
/*
  Serial.print(hx,6);
  Serial.print("\t");
  Serial.print(hy,6);
  Serial.print("\t");
  Serial.print(hz,6);
  Serial.print("\t");

  Serial.println(t,6);*/
}
