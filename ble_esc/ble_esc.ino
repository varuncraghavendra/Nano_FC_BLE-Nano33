#define MOTOR1_PIN 8
#define MOTOR2_PIN 27
#define MOTOR3_PIN 2
#define MOTOR4_PIN 1

long c_time, end_time;
long loop_time;

long esc_pulse;

void setup(){
  delay(2000);

  NRF_P0->PIN_CNF[27] = 0x00000001;
  NRF_P1->PIN_CNF[2] = 0x00000001;
  NRF_P1->PIN_CNF[1] = 0x00000001;
  NRF_P1->PIN_CNF[8] = 0x00000001;


  Serial.begin(9600);
  delay(2000);

  c_time = micros();

  c_time = micros();
  while(micros() - c_time < 5000000){
    
    NRF_P0->OUT |= (1 << 27);
    NRF_P1->OUT |= (1 << 2);
    NRF_P1->OUT |= (1 << 1);
    NRF_P1->OUT |= (1 << 8);
    
    delayMicroseconds(1000);
    
    NRF_P0->OUT &= ~(1 << 27);
    NRF_P1->OUT &= ~(1 << 2);
    NRF_P1->OUT &= ~(1 << 1);
    NRF_P1->OUT &= ~(1 << 8);

    delay(3);

  }

  loop_time = micros();
  
}

void loop(){

  //NRF_P0->OUT |= (1 << 27);
  //NRF_P1->OUT |= (1 << 2);
  //NRF_P1->OUT |= (1 << 1);
  NRF_P1->OUT |= (1 << 8);
  
  delayMicroseconds(1300);
  
  //NRF_P0->OUT &= ~(1 << 27);
  //NRF_P1->OUT &= ~(1 << 2);
  //NRF_P1->OUT &= ~(1 << 1);
  NRF_P1->OUT &= ~(1 << 8);

  while(micros() - loop_time < 4000);
  loop_time = micros();
  
}

/*
void setup() {

  delay(2000);

  NRF_P0->PIN_CNF[27] = 0x00000001;
  NRF_P1->PIN_CNF[2] = 0x00000001;
  NRF_P1->PIN_CNF[1] = 0x00000001;
  NRF_P1->PIN_CNF[8] = 0x00000001;

  NRF_P0->OUT &= ~(1 << 27);

  Serial.begin(9600);

  NRF_P0->OUT |= (1 << 27);

  c_time = micros();

  esc_pulse = 2000 + c_time;

  while(NRF_P0->OUTSET & 0x08000000){
    end_time = micros();
    if(esc_pulse < end_time){
      NRF_P0->OUT &= ~(1 << 27);
    }
  }

  delay(5000);

  NRF_P0->OUT |= (1 << 27);

  c_time = micros();

  esc_pulse = 1000 + c_time;

  while(NRF_P0->OUTSET & 0x08000000){
    end_time = micros();
    if(esc_pulse < end_time){
      NRF_P0->OUT &= ~(1 << 27);
    }
  }
  
  loop_time = micros();
  Serial.println(loop_time);
}

void loop() {

  NRF_P0->OUT |= (1 << 27);

  c_time = micros();

  esc_pulse = 1500 + c_time;

  while(NRF_P0->OUTSET & 0x08000000){
    end_time = micros();
    if(esc_pulse < end_time){
      NRF_P0->OUT &= ~(1 << 27);
    }
  }

  while(micros() - loop_time < 4000);
  loop_time = micros();

}
*/
