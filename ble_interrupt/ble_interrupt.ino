bool pin_r_last_value = 0, pin_p_last_value = 0, pin_t_last_value = 0, pin_y_last_value = 0;
int rc_roll_pwm, rc_pitch_pwm, rc_throttle_pwm, rc_yaw_pwm;
unsigned long r_c_time, p_c_time, t_c_time, y_c_time;

void setup() {

  NRF_P1->PIN_CNF[11] = 0x0000000C;
  NRF_P1->PIN_CNF[12] = 0x0000000C;
  NRF_P1->PIN_CNF[15] = 0x0000000C;
  NRF_P1->PIN_CNF[13] = 0x0000000C;

  attachInterrupt(digitalPinToInterrupt(2), rc_roll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rc_pitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), rc_thr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), rc_yaw, CHANGE);

  Serial.begin(9600);
}

void loop() {

  Serial.print("Roll: ");
  Serial.print(rc_roll_pwm);

  Serial.print("  Pitch: ");
  Serial.print(rc_pitch_pwm);
  
  Serial.print("  Throttle: ");
  Serial.print(rc_throttle_pwm);

  Serial.print("  Yaw: ");
  Serial.println(rc_yaw_pwm);
  
}

void rc_roll(){
  if(pin_r_last_value == 0 && (NRF_P1->IN & 0x00000800)){
    pin_r_last_value = 1;
    r_c_time = micros();
  }
  else if(pin_r_last_value == 1 && !(NRF_P1->IN & 0x00000800)){
    pin_r_last_value = 0;
    rc_roll_pwm = micros() - r_c_time;
  }
 
}

void rc_pitch(){
  if(pin_p_last_value == 0 && (NRF_P1->IN & 0x00001000)){
    pin_p_last_value = 1;
    p_c_time = micros();
  }
  else if(pin_p_last_value == 1 && !(NRF_P1->IN & 0x00001000)){
    pin_p_last_value = 0;
    rc_pitch_pwm = micros() - p_c_time;
  }
 
}

void rc_thr(){
  if(pin_t_last_value == 0 && (NRF_P1->IN & 0xFFFF8000)){
    pin_t_last_value = 1;
    t_c_time = micros();
  }
  else if(pin_t_last_value == 1 && !(NRF_P1->IN & 0xFFFF8000)){
    pin_t_last_value = 0;
    rc_throttle_pwm = micros() - t_c_time;
  }
 
}

void rc_yaw(){
  if(pin_y_last_value == 0 && (NRF_P1->IN & 0x00002000)){
    pin_y_last_value = 1;
    y_c_time = micros();
  }
  else if(pin_y_last_value == 1 && !(NRF_P1->IN & 0x00002000)){
    pin_y_last_value = 0;
    rc_yaw_pwm = micros() - y_c_time;
  }
 
}
