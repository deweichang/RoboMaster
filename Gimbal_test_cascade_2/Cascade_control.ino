


float CascadeControlYaw(float pos_setpoint_yaw) {
  float pos_kp = 1.7;
  float pos_ki = 1.2;
  float pos_kd = 0.12;
  int cnt_ref = (int)pos_setpoint_yaw * 8191.0 / 360;
  int pos_error = (cnt_ref - curr_ang_cnts[4] + 4096) % 8191 - 4096; //error = ref - reading

  if (pos_error > 4096) {
    pos_error -= 8191;
  }
  else if (pos_error < -4096) {
    pos_error += 8191;
  }
  //-----------------------------------------------------------------------
  //PID Cal
  posP[0] = pos_kp * pos_error;
  posI[0] += pos_ki * pos_error * dt;
  posD[0] = pos_kd * (pos_error - prev_pos_error[0]) / dt;
  //
  prev_pos_error[0] = pos_error;
  pos_output[0] = posP[0] + posI[0] + posD[0];

  float vel_kp = 1;
  float vel_ki = 0;
  float vel_error = pos_output[0] - (gz_fil / pi * 180); //error = ref from pos controller - IMU reading

  velP[0] = vel_kp * vel_error;
  velI[0] += vel_ki * vel_error * dt;

  int16_t vel_output = velP[0] + velI[0];
  return vel_output;
}

float CascadeControlPitch(float pos_setpoint_pitch) {
  float pos_kp = 3;
  float pos_ki = 0;
  float pos_kd = 0.3;
  int cnt_ref = (int)pos_setpoint_pitch * 8191.0 / 360;
  int pos_error = (cnt_ref - curr_ang_cnts[5] + 4096) % 8191 - 4096; //error = ref - reading

  if (pos_error > 4096) {
    pos_error -= 8191;
  }
  else if (pos_error < -4096) {
    pos_error += 8191;
  }
  //-----------------------------------------------------------------------
  //PID Cal
  if ( abs(pos_error) < 90) {
    pos_kp = 5;
    pos_ki = 7;
    pos_kd = 0.09;
  }
  /*if (gx_fil > 10) {
    pos_kp = 0;
    pos_ki = 0;
    pos_kd = 0;
    }
    if (gx_fil < -10) {
    pos_kp = 0;
    pos_ki = 0;
    pos_kd = 0;
    }*/
  posP[1] = pos_kp * pos_error;
  posI[1] += pos_ki * pos_error * dt;
  posD[1] = pos_kd * (pos_error - prev_pos_error[1]) / dt;


  prev_pos_error[1] = pos_error;
  pos_output[1] = posP[1] + posI[1] + posD[1];
  /*if (pos_output[1] > 200) {
    pos_output[1] = 200;
    }
    else if (pos_output[1] < -2500) {
    pos_output[1] = -2500;
    }*/
  float vel_kp = 1.5;
  float vel_ki = 0;
  float vel_error = pos_output[1] - (gx_fil / pi * 180); //error = ref from pos controller - IMU reading

  velP[1] = vel_kp * vel_error;
  velI[1] += vel_ki * vel_error * dt;

  int16_t vel_output = velP[1] + velI[1];
  return vel_output;
}
