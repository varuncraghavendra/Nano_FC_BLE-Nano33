bool pin_r_last_value = 0, pin_p_last_value = 0, pin_t_last_value = 0, pin_y_last_value = 0;
int rc_roll_pwm, rc_pitch_pwm, rc_throttle_pwm, rc_yaw_pwm;
unsigned long r_c_time, p_c_time, t_c_time, y_c_time;

void setup() {
  //pinMode(2, INPUT_PULLUP);

  NRF_P1->PIN_CNF[13] = 0x0000000C;

  NRF_P0->PIN_CNF[13] = 0x00000001;

  //NVIC_EnableIRQ(GPIOTE_IRQn);
  //NRF_GPIOTE->CONFIG[0]
  
  Serial.begin(9600);
}

void loop() {

  //Serial.println(NRF_P1->PIN_CNF[11]);

  //if(NRF_P1->IN & 0x00002000) Serial.println("HIGH!!");
  //if(!(NRF_P1->IN & 0x00002000)) Serial.println("LOW..");

  
  NRF_P0->OUT &= ~(1 << 13); //clear
  Serial.println(NRF_P0->OUTSET & 0x00002000);
  delay(5000);
  NRF_P0->OUT |= (1 << 13); //set
  Serial.println(NRF_P0->OUTSET & 0x00002000);
  delay(5000);
  
}
