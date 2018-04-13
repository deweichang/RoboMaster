int16_t posP[2] = {0,0}, posI[2] = {0,0}, posD[2] = {0,0}, velP[2] = {0,0}, velI[2] = {0,0};
int prev_pos_error[2] = {0,0};


float CascadeControlYaw(float pos_setpoint_yaw){
  float pos_kp = 0.5;
  float pos_ki = 0.5;
  float pos_kd = 0.1;
  int cnt_ref = (int)pos_setpoint_yaw*8191.0/360;
  int pos_error = (cnt_ref - curr_ang_cnts[4] + 4096)%8191 - 4096; //error = ref - reading

  if (pos_error > 4096){ 
    pos_error -= 8191;
  }
  else if (pos_error < -4096){
    pos_error += 8191;
  }
  
  if(abs(pos_error) > 4096){
    pos_error += 8191;
  }
  
  float pos_error_ang = pos_error*360.0/8191;
  int16_t pos_output = 0;
  if(pos_error_ang > -0.1 && pos_error_ang < 0.1){
    pos_output = 0;
  }else{

  posP[0] = pos_kp*pos_error;
  posI[0] += pos_ki*pos_error*dt;
  posD[0] = pos_kd*(pos_error - prev_pos_error[0]) / dt;

  prev_pos_error[0] = pos_error;
  float pos_output = posP[0] + posI[0] + posD[0];
  }

  float vel_kp = 1.0;
  float vel_ki = 1.0;
  float vel_error = pos_output - gz; //error = ref from pos controller - IMU reading

  velP[0] = vel_kp*vel_error;
  velI[0] += vel_ki*vel_error*dt;

  int16_t vel_output = velP[0] + velI[0];
  return vel_output;
}

float CascadeControlPitch(float pos_setpoint_pitch){
  float pos_kp = 0.5;
  float pos_ki = 0.5;
  float pos_kd = 0.1;
  int cnt_ref = (int)pos_setpoint_pitch*8191.0/360;
  int pos_error = (cnt_ref - curr_ang_cnts[5] + 4096)%8191 - 4096; //error = ref - reading

  posP[1] = pos_kp*pos_error;
  posI[1] += pos_ki*pos_error*dt;
  posD[1] = pos_kd*(pos_error - prev_pos_error[1]) / dt;

  prev_pos_error[1] = pos_error;
  float pos_output = posP[1] + posI[1] + posD[1];

  float vel_kp = 1.0;
  float vel_ki = 1.0;
  float vel_error = pos_output - gy; //error = ref from pos controller - IMU reading

  velP[1] = vel_kp*vel_error;
  velI[1] += vel_ki*vel_error*dt;

  int16_t vel_output = velP[1] + velI[1];
  return vel_output;
}
