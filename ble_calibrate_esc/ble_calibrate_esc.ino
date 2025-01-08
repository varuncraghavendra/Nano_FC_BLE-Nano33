#define MOTOR1_PIN 8
#define MOTOR2_PIN 27
#define MOTOR3_PIN 2
#define MOTOR4_PIN 1

unsigned long c_time;

void setup() {

  //Configure pins 8-11 as output
  NRF_P1->PIN_CNF[MOTOR1_PIN] = 0x00000001;
  NRF_P0->PIN_CNF[MOTOR2_PIN] = 0x00000001;
  NRF_P1->PIN_CNF[MOTOR3_PIN] = 0x00000001;
  NRF_P1->PIN_CNF[MOTOR4_PIN] = 0x00000001;

  Serial.println("Calibrating ESC (1000us - 2000us).....");
  Serial.println();
  Serial.print("Writing 2000us.....");

  c_time = micros();
  while(micros() - c_time < 5000000){
    
    NRF_P1->OUT |= (1 << MOTOR1_PIN);
    NRF_P0->OUT |= (1 << MOTOR2_PIN);
    NRF_P1->OUT |= (1 << MOTOR3_PIN);
    NRF_P1->OUT |= (1 << MOTOR4_PIN);
    
    delayMicroseconds(2000);
    
    NRF_P1->OUT &= ~(1 << MOTOR1_PIN);
    NRF_P0->OUT &= ~(1 << MOTOR2_PIN);
    NRF_P1->OUT &= ~(1 << MOTOR3_PIN);
    NRF_P1->OUT &= ~(1 << MOTOR4_PIN);

  }

  Serial.print("  Done! \nWriting 1000us.....");

  c_time = micros();
  while(micros() - c_time < 5000000){
    
    NRF_P1->OUT |= (1 << MOTOR1_PIN);
    NRF_P0->OUT |= (1 << MOTOR2_PIN);
    NRF_P1->OUT |= (1 << MOTOR3_PIN);
    NRF_P1->OUT |= (1 << MOTOR4_PIN);
    
    delayMicroseconds(1000);
    
    NRF_P1->OUT &= ~(1 << MOTOR1_PIN);
    NRF_P0->OUT &= ~(1 << MOTOR2_PIN);
    NRF_P1->OUT &= ~(1 << MOTOR3_PIN);
    NRF_P1->OUT &= ~(1 << MOTOR4_PIN);

  }

  Serial.println("Done! \n\n ESCs are now calibrated!");

  c_time = micros();
  while(micros() - c_time < 15000000){
    
    NRF_P1->OUT |= (1 << MOTOR1_PIN);
    NRF_P0->OUT |= (1 << MOTOR2_PIN);
    NRF_P1->OUT |= (1 << MOTOR3_PIN);
    NRF_P1->OUT |= (1 << MOTOR4_PIN);
    
    delayMicroseconds(1000);
    
    NRF_P1->OUT &= ~(1 << MOTOR1_PIN);
    NRF_P0->OUT &= ~(1 << MOTOR2_PIN);
    NRF_P1->OUT &= ~(1 << MOTOR3_PIN);
    NRF_P1->OUT &= ~(1 << MOTOR4_PIN);

  }

}

void loop() {

}
