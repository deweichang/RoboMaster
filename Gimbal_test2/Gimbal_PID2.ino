float gimbalYawPID2(float ang_ref){
  float yaw_kp = 150.0;
  float yaw_ki = 10.0;
  float yaw_kd = 0.0;
  int cnt_ref = (int)ang_ref*8191.0/360;
  int yaw_cnt_diff = (cnt_ref - curr_ang_cnts[4] + 4096)%8191 - 4096; //error

  if (yaw_cnt_diff > 4096){ 
    yaw_cnt_diff -= 8191;
  }
  else if (yaw_cnt_diff < -4096){
    yaw_cnt_diff += 8191;
  }
  
  if(abs(yaw_cnt_diff) > 4096){
    yaw_cnt_diff = yaw_cnt_diff + 8191;
  }
  
  float yaw_ang_diff = yaw_cnt_diff*360.0/8191;
  int16_t yaw_ctrl = 0;
  if(yaw_ang_diff > -0.1 && yaw_ang_diff < 0.1){
    yaw_ctrl = 0;
  }else{
    yawP = yaw_kp*(yaw_cnt_diff - prev_yaw_cnt_diff);
    yawI = yaw_ki*yaw_cnt_diff*dt;
    yawD = yaw_kd*(yaw_cnt_diff - 2*prev_yaw_cnt_diff + prev_yaw_cnt_diff2)/dt;
/*
    if (abs(yaw_ang_diff) > 30){
      yawI = 0;
    }
    */
    //yaw_ctrl = -yaw_ang_diff*ctrl_scale; //**
    yaw_ctrl += yawP + yawI + yawD;
  }
  

  if(yaw_ctrl > 0){yaw_ctrl = yaw_ctrl + 70;}
  else if(yaw_ctrl < 0){yaw_ctrl = yaw_ctrl - 47;}

  prev_yaw_cnt_diff2 = prev_yaw_cnt_diff;
  prev_yaw_cnt_diff = yaw_cnt_diff;
  
  return yaw_ctrl;
}

float gimbalPitchPID2(float ang_ref){
  float pitch_kp = 0.0; //80-0-0 //2
  float pitch_ki = 200;
  float pitch_kd = 0.0; //2
  int cnt_ref = (int)ang_ref*8191.0/360;
  int pitch_cnt_diff = (cnt_ref - curr_ang_cnts[5] + 4096)%8191 - 4096; //error

/*
  if (curr_ang_cnts[5]*360.0/8191 > ang_ref){
    pitch_kp = 3.0; //higher gain to lift the barrel
  }
  else {
    pitch_kp = 1.5;
  }
*/

//in case going beyond the gap between 0 and 360
  if (pitch_cnt_diff > 4096){ 
    pitch_cnt_diff -= 8191;
  }
  else if (pitch_cnt_diff < -4096){
    pitch_cnt_diff += 8191;
  }
  
  
  if(abs(pitch_cnt_diff) > 4096){
    pitch_cnt_diff = pitch_cnt_diff + 8191;
  }
  float pitch_ang_diff = pitch_cnt_diff*360.0/8191; //-
  int16_t pitch_ctrl = 0;
  if(pitch_ang_diff > -0.1 && pitch_ang_diff < 0.1){
    pitch_ctrl = 0;
  }else{
    pitchP = pitch_kp*(pitch_cnt_diff - prev_pitch_cnt_diff);
    pitchI = pitch_ki*pitch_cnt_diff*dt;
    pitchD = pitch_kd*(pitch_cnt_diff - 2*prev_pitch_cnt_diff + prev_pitch_cnt_diff2)/dt;
    //yaw_ctrl = -yaw_ang_diff*ctrl_scale; //**
    /*
    if (abs(pitch_ang_diff) > 10){
      pitchI = 0;
    }
    */
    
    if (pitchD > 0){
      pitchD = 0;
    }
    
    pitch_ctrl += pitchP + pitchI + pitchD;
  }

  if(pitch_ctrl > 0){pitch_ctrl = pitch_ctrl + 70;}
  else if(pitch_ctrl < 0){pitch_ctrl = pitch_ctrl - 47;}

  prev_pitch_cnt_diff2 = prev_pitch_cnt_diff;
  prev_pitch_cnt_diff = pitch_cnt_diff;
  
  return pitch_ctrl;
}
