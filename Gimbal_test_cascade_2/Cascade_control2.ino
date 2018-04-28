//posP, posI, posD, pos_output, and prev_pos_error are global variable arrays, 0 stands for YAW,...
//1 stands for PITCH


float CascadeControlYaw2(float pos_setpoint_yaw) {
  float pos_kp = 100.0;
  float pos_ki = 500.0; //150
  float pos_kd = 0.2; //0.12 //0.5
  int cnt_ref = (int)pos_setpoint_yaw * 8191.0 / 360;
  int pos_error = (cnt_ref - curr_ang_cnts[4]);// + 4096) % 8191 - 4096; //error = ref - reading

  if (pos_error > 4096) {
    pos_error -= 8192;
  }
  else if (pos_error < -4096) {
    pos_error += 8192;
  }

  /*if ( abs(pos_error) < 20) {
    pos_kp = 100;
    pos_ki = 200.0;
    pos_kd = 0.09;//0.09;
  }*/
  
  //-----------------------------------------------------------------------
  //PID Cal
  posP[0] = pos_kp * (pos_error - prev_pos_error[0]);
  posI[0] = pos_ki * (pos_error + prev_pos_error[0]) * dt / 2;
  posD[0] = pos_kd * (pos_error - 2*prev_pos_error[0] + prev_pos_error2[0]) / dt;
  //
  prev_pos_error2[0] = prev_pos_error[0];
  prev_pos_error[0] = pos_error;
  pos_output[0] += posP[0] + posI[0] + posD[0];
  //anti-windup (in terms of error)
  /*if (abs(pos_error) > 450){ //5 deg(115)
    pos_output[0] = posP[0] + posD[0];
  }*/

  float vel_kp = 1;
  float vel_ki = 0;
  float vel_error = pos_output[0] - (gz_fil / pi * 180); //error = ref from pos controller - IMU reading

  velP[0] = vel_kp * (vel_error - prev_vel_error[0]);
  velI[0] = vel_ki * (vel_error + prev_vel_error[0]) * dt / 2;

  prev_vel_error[0] = vel_error;

  static int16_t vel_output;
  vel_output += velP[0] + velI[0];
  //Saturation (prevent it moving too fast)
  if (vel_output > 1500){ //800
    vel_output = 1500;
  }
  else if (vel_output < -1500){
    vel_output = -1500;
  }
  //vel_output = -90;
  return vel_output;
}

float CascadeControlPitch2(float pos_setpoint_pitch) {
  float pos_kp = 25; //4
  float pos_ki = 15.0; //0 //5
  float pos_kd = 0.3; //0.3 
  int cnt_ref = (int)pos_setpoint_pitch * 8191.0 / 360;
  int pos_error = (cnt_ref - curr_ang_cnts[5]); // + 4096) % 8191 - 4096; //error = ref - reading


  if (pos_error > 4096) {
    pos_error -= 8192;
  }
  else if (pos_error < -4096) {
    pos_error += 8192;
  }
  //-----------------------------------------------------------------------
  //PID Cal
  /*if ( abs(pos_error) < 50) {
    pos_kp = 10;
    pos_ki = 8; //7
    pos_kd = 0.08;//0.09;
  }*/

  posP[1] = pos_kp * (pos_error - prev_pos_error[1]);
  posI[1] = pos_ki * (pos_error + prev_pos_error[1]) * dt / 2;
  posD[1] = pos_kd * (pos_error - 2*prev_pos_error[1] + prev_pos_error2[1]) / dt;
  //
  prev_pos_error2[1] = prev_pos_error[1];
  prev_pos_error[1] = pos_error;
  pos_output[1] += posP[1] + posI[1] + posD[1];
  
  /*if (pos_output[1] > 200) {
    pos_output[1] = 200;
    }
    else if (pos_output[1] < -2500) {
    pos_output[1] = -2500;
    }*/
    
  float vel_kp = 1.0;
  float vel_ki = 0.5;
  float vel_error = pos_output[1] - (gx_fil / pi * 180); //error = ref from pos controller - IMU reading

  velP[1] = vel_kp * (vel_error - prev_vel_error[1]);
  velI[1] = vel_ki * (vel_error + prev_vel_error[1]) * dt / 2;
  prev_vel_error[1] = vel_error;

  static int16_t vel_output = 0;
  //vel_output=-0.9967*prev_vel_output-((1.99-1.4)/3.7874*100000)*gx_fil-((0.6-0.99)/3.7874*100000)*gxx[1]+(1-1.4+0.6)/3.7874*100000*pos_output[1];
  vel_output += velP[1] + velI[1];
  
  if (vel_output > 2500){
    vel_output = 2500;
  }
  else if (vel_output < -2500){
    vel_output = -2500;
  }
  //prev_vel_output = vel_output;
  return vel_output;
}
