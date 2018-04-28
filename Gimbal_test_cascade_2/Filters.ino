//input data that needs to be filtered, and output clean data (sampling rate = 0.001 sec)
float butterworthFilter(float data) {
  float b0 = 0.1269 / 10000.0;
  float b1 = 0.5077 / 10000.0;
  float b2 = 0.7616 / 10000.0;
  float b3 = 0.5077 / 10000.0;
  float b4 = 0.1269 / 10000.0;
  float b5 = 0;//0.046927020880966 / 10000.0;
  //float a0=1.0;
  float a1 = -3.6756;
  float a2 = 5.0785;
  float a3 = -3.1254;
  float a4 = 0.7227;
  float a5 = 0;
  float x_k0, y_k0;
  static float x_k1 = 0.0, x_k2 = 0.0, x_k3 = 0.0, x_k4 = 0.0, x_k5 = 0.0;
  static float y_k1 = 0.0, y_k2 = 0.0, y_k3 = 0.0, y_k4 = 0.0, y_k5 = 0.0;
  
  x_k0 = data;
  y_k0 = b0 * x_k0 + b1 * x_k1 + b2 * x_k2 + b3 * x_k3 + b4 * x_k4 + b5 * x_k5 - a1 * y_k1 - a2 * y_k2 - a3 * y_k3 - a4 * y_k4 - a5 * y_k5;
  //update previous input data
  x_k5 = x_k4; x_k4 = x_k3; x_k3 = x_k2; x_k2 = x_k1; x_k1 = x_k0;
  //update previous output data
  y_k5 = y_k4; y_k4 = y_k3; y_k3 = y_k2; y_k2 = y_k1; y_k1 = y_k0;
  
  return y_k0;
}

