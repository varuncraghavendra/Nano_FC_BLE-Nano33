#include <Wire.h>

#define R2D 57.29578
#define D2R 0.0174533

#define MOTOR1_PIN 8
#define MOTOR2_PIN 27
#define MOTOR3_PIN 2
#define MOTOR4_PIN 1

#define RC_CH1_PIN 11
#define RC_CH2_PIN 12
#define RC_CH3_PIN 15
#define RC_CH4_PIN 13
#define RC_CH5_PIN 14

#define GYRO_PRESET 0.0175
#define ACC_PRESET 0.000244

#define LOOP_RATE 250

#define CF_K 0.997
#define EWMA_K1 0.7
#define EWMA_K2 0.9

#define KP_ROLL_RATE 1.2
#define KI_ROLL_RATE 0.02
#define KD_ROLL_RATE 4.5

#define KP_ROLL 1.87

#define KP_YAW_RATE 3
#define KI_YAW_RATE 0.02
#define KD_YAW_RATE 0

#define PID_MAX_OUTPUT 400

extern TwoWire Wire1;

bool imu_start = false;
bool rc_pins_prev_data[] = {false, false, false, false, false};

int16_t raw_gyro_data[] = {0, 0, 0};
int16_t raw_acc_data[] = {0, 0, 0};

int uav_start_flag = 0;
int esc_pwm[] = {1000, 1000, 1000, 1000};
int rc_channels[] = {1500, 1500, 1000, 1500, 1000};

float acc_data[] = {0., 0., 0.};
float gyro_data[] = {0., 0., 0.};
float gyro_euler[] = {0., 0.};
float acc_euler[] = {0., 0.};
float euler_angles[] = {0., 0.};

float gyro_euler_dps[] = {0., 0., 0.};
float rate_setpoints[] = {0., 0., 0.};
float euler_corrections[] = {0., 0.};

float pid_error_euler[] = {0., 0., 0.};
float pid_prev_error_euler[] = {0., 0., 0.};
float pid_integrator_euler[] = {0., 0., 0.};
float pid_output[] = {0., 0., 0.};

long gyro_offset[] = {0, 0, 0};

unsigned long loop_timer, c_time;
unsigned long rc_channels_timer[] = {0, 0, 0, 0, 0};
unsigned long esc_timer[] = {0, 0, 0, 0};

void setup() {
  Serial.begin(9600);

  Wire1.begin();
  Wire1.setClock(400000);

  //Configure pins 2-6 as input
  NRF_P1->PIN_CNF[RC_CH1_PIN] = 0x0000000C;
  NRF_P1->PIN_CNF[RC_CH2_PIN] = 0x0000000C;
  NRF_P1->PIN_CNF[RC_CH3_PIN] = 0x0000000C;
  NRF_P1->PIN_CNF[RC_CH4_PIN] = 0x0000000C;
  NRF_P1->PIN_CNF[RC_CH5_PIN] = 0x0000000C;

  //Configure pins 8-11 as output
  NRF_P1->PIN_CNF[MOTOR1_PIN] = 0x00000001;
  NRF_P0->PIN_CNF[MOTOR2_PIN] = 0x00000001;
  NRF_P1->PIN_CNF[MOTOR3_PIN] = 0x00000001;
  NRF_P1->PIN_CNF[MOTOR4_PIN] = 0x00000001;

  attachInterrupt(digitalPinToInterrupt(2), rc_roll, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), rc_pitch, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4), rc_thr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(5), rc_yaw, CHANGE);
  attachInterrupt(digitalPinToInterrupt(6), rc_kill, CHANGE);

  delay(5000);

  //Setup Gyro and Accel Registers
  setup_imu();

  //Obtain gyro calibration offset
  calibrate_imu();

  loop_timer = micros();

  //uav_start_flag = 2;
}

void loop() {

  read_raw_imu();

  gyro_euler_dps[0] = -((gyro_euler_dps[0] * EWMA_K1) + (( (float)(raw_gyro_data[0] - gyro_offset[0]) * GYRO_PRESET ) * (1 - EWMA_K1)));
  gyro_euler_dps[1] = -((gyro_euler_dps[1] * EWMA_K1) + (( (float)(raw_gyro_data[1] - gyro_offset[1]) * GYRO_PRESET ) * (1 - EWMA_K1)));
  gyro_euler_dps[2] = -((gyro_euler_dps[2] * EWMA_K1) + (( (float)(raw_gyro_data[2] - gyro_offset[2]) * GYRO_PRESET ) * (1 - EWMA_K1)));

  gyro_data[0] = (float)(raw_gyro_data[0] - gyro_offset[0]) * GYRO_PRESET;
  gyro_data[1] = (float)(raw_gyro_data[1] - gyro_offset[1]) * GYRO_PRESET;
  gyro_data[2] = (float)(raw_gyro_data[2] - gyro_offset[2]) * GYRO_PRESET;

  acc_data[0] = ((float)raw_acc_data[0]) * ACC_PRESET;
  acc_data[1] = ((float)raw_acc_data[1]) * ACC_PRESET;
  acc_data[2] = ((float)raw_acc_data[2]) * ACC_PRESET;

  gyro_euler[0] -= (gyro_data[0] / LOOP_RATE);
  gyro_euler[1] -= (gyro_data[1] / LOOP_RATE);

  gyro_euler[0] += gyro_euler[1] * sin(-gyro_data[2] * D2R / LOOP_RATE);
  gyro_euler[1] -= gyro_euler[0] * sin(-gyro_data[2] * D2R / LOOP_RATE);

  acc_euler[0] = asin(acc_data[1] / sqrt(acc_data[0] * acc_data[0] + acc_data[1] * acc_data[1] + acc_data[2] * acc_data[2])) * R2D;
  acc_euler[1] = -asin(acc_data[0] / sqrt(acc_data[0] * acc_data[0] + acc_data[1] * acc_data[1] + acc_data[2] * acc_data[2])) * R2D;

  if (!imu_start) {
    gyro_euler[0] = acc_euler[0];
    gyro_euler[1] = acc_euler[1];
  }
  else {
    gyro_euler[0] = gyro_euler[0] * CF_K + acc_euler[0] * (1 - CF_K);
    gyro_euler[1] = gyro_euler[1] * CF_K + acc_euler[1] * (1 - CF_K);
  }

  euler_angles[0] = EWMA_K2 * euler_angles[0] + (1 - EWMA_K2) * gyro_euler[0];
  euler_angles[1] = EWMA_K2 * euler_angles[1] + (1 - EWMA_K2) * gyro_euler[1];

  euler_corrections[0] = KP_ROLL * euler_angles[0];
  euler_corrections[1] = KP_ROLL * euler_angles[1];

  if (rc_channels[3] > 1940 && rc_channels[2] < 1050) uav_start_flag = 1;

  if (uav_start_flag == 1 && rc_channels[3] < 1550 && rc_channels[2] < 1050) {
    uav_start_flag = 2;
    imu_start = true;

    pid_integrator_euler[0] = 0.;
    pid_integrator_euler[1] = 0.;
    pid_integrator_euler[2] = 0.;

    pid_prev_error_euler[0] = 0.;
    pid_prev_error_euler[1] = 0.;
    pid_prev_error_euler[2] = 0.;

  }

  if (uav_start_flag == 2 && rc_channels[2] < 1050 && rc_channels[3] < 1050) {
    uav_start_flag = 0;
    imu_start = false;
  }

  //Roll rate input
  if (rc_channels[0] > 1510) rate_setpoints[0] = rc_channels[0] - 1510;
  else if (rc_channels[0] < 1490) rate_setpoints[0] = rc_channels[0] - 1490;
  else rate_setpoints[0] = 0.0;

  rate_setpoints[0] -= euler_corrections[0];
  rate_setpoints[0] /= 6;

  //Pitch rate input
  if (rc_channels[1] > 1510) rate_setpoints[1] = rc_channels[1] - 1510;
  else if (rc_channels[1] < 1490) rate_setpoints[1] = rc_channels[1] - 1490;
  else rate_setpoints[1] = 0.0;

  rate_setpoints[1] -= euler_corrections[1];
  rate_setpoints[1] /= 6;

  //Yaw rate input
  if (rc_channels[3] > 1510 && rc_channels[2] > 1050) rate_setpoints[2] = (rc_channels[3] - 1510) / 3.0;
  else if (rc_channels[3] < 1490 && rc_channels[2] > 1050) rate_setpoints[2] = (rc_channels[3] - 1490) / 3.0;
  else rate_setpoints[2] = 0.0;

  //PID Calculations
  //1) roll rate
  pid_error_euler[0] = gyro_euler_dps[0] - rate_setpoints[0];
  pid_integrator_euler[0] += KI_ROLL_RATE * pid_error_euler[0];
  if (pid_integrator_euler[0] > PID_MAX_OUTPUT) pid_integrator_euler[0] = PID_MAX_OUTPUT;
  else if (pid_integrator_euler[0] < -PID_MAX_OUTPUT) pid_integrator_euler[0] = -PID_MAX_OUTPUT;

  pid_output[0] = KP_ROLL_RATE * pid_error_euler[0] + pid_integrator_euler[0] + KD_ROLL_RATE * (pid_error_euler[0] - pid_prev_error_euler[0]);
  if (pid_output[0] > PID_MAX_OUTPUT) pid_output[0] = PID_MAX_OUTPUT;
  else if (pid_output[0] < -PID_MAX_OUTPUT) pid_output[0] = -PID_MAX_OUTPUT;

  pid_prev_error_euler[0] = pid_error_euler[0];

  //2) pitch rate
  pid_error_euler[1] = gyro_euler_dps[1] - rate_setpoints[1];
  pid_integrator_euler[1] += KI_ROLL_RATE * pid_error_euler[1];
  if (pid_integrator_euler[1] > PID_MAX_OUTPUT) pid_integrator_euler[1] = PID_MAX_OUTPUT;
  else if (pid_integrator_euler[1] < -PID_MAX_OUTPUT) pid_integrator_euler[1] = -PID_MAX_OUTPUT;

  pid_output[1] = KP_ROLL_RATE * pid_error_euler[1] + pid_integrator_euler[1] + KD_ROLL_RATE * (pid_error_euler[1] - pid_prev_error_euler[1]);
  if (pid_output[1] > PID_MAX_OUTPUT) pid_output[1] = PID_MAX_OUTPUT;
  else if (pid_output[1] < -PID_MAX_OUTPUT) pid_output[1] = -PID_MAX_OUTPUT;

  pid_prev_error_euler[1] = pid_error_euler[1];

  //3) yaw rate
  pid_error_euler[2] = gyro_euler_dps[2] - rate_setpoints[2];
  pid_integrator_euler[2] += KI_YAW_RATE * pid_error_euler[2];
  if (pid_integrator_euler[2] > PID_MAX_OUTPUT) pid_integrator_euler[2] = PID_MAX_OUTPUT;
  else if (pid_integrator_euler[2] < -PID_MAX_OUTPUT) pid_integrator_euler[2] = -PID_MAX_OUTPUT;

  pid_output[2] = KP_YAW_RATE * pid_error_euler[2] + pid_integrator_euler[2] + KD_YAW_RATE * (pid_error_euler[2] - pid_prev_error_euler[2]);
  if (pid_output[2] > PID_MAX_OUTPUT) pid_output[2] = PID_MAX_OUTPUT;
  else if (pid_output[2] < -PID_MAX_OUTPUT) pid_output[2] = -PID_MAX_OUTPUT;

  pid_prev_error_euler[2] = pid_error_euler[2];

  if (uav_start_flag == 2) {

    if (rc_channels[2] > 1800) rc_channels[2] = 1800;

    esc_pwm[0] = rc_channels[2] + pid_output[0] - pid_output[1] - pid_output[2];
    esc_pwm[1] = rc_channels[2] - pid_output[0] + pid_output[1] - pid_output[2];
    esc_pwm[2] = rc_channels[2] - pid_output[0] - pid_output[1] + pid_output[2];
    esc_pwm[3] = rc_channels[2] + pid_output[0] + pid_output[1] + pid_output[2];

    if (esc_pwm[0] > 2000) esc_pwm[0] = 2000;
    if (esc_pwm[1] > 2000) esc_pwm[1] = 2000;
    if (esc_pwm[2] > 2000) esc_pwm[2] = 2000;
    if (esc_pwm[3] > 2000) esc_pwm[3] = 2000;

    if (esc_pwm[0] < 1100) esc_pwm[0] = 1100;
    if (esc_pwm[1] < 1100) esc_pwm[1] = 1100;
    if (esc_pwm[2] < 1100) esc_pwm[2] = 1100;
    if (esc_pwm[3] < 1100) esc_pwm[3] = 1100;

  }
  else {
    esc_pwm[0] = 990;
    esc_pwm[1] = 990;
    esc_pwm[2] = 990;
    esc_pwm[3] = 990;
  }
  //Serial.print(euler_angles[0]);Serial.print("\t");
  //Serial.println(euler_angles[1]);
  /*Serial.print(uav_start_flag); Serial.print("\t");
  Serial.print(rc_channels[0]); Serial.print("\t");
  Serial.print(rc_channels[1]); Serial.print("\t");
  Serial.print(rc_channels[2]); Serial.print("\t");
  Serial.print(rc_channels[3]); Serial.print("\t");
  Serial.println(rc_channels[4]);*/
  //Serial.print(gyro_euler_dps[0]);Serial.print("\t");
  //Serial.println(gyro_euler_dps[1]);

  
  c_time = micros();
  NRF_P1->OUT |= (1 << MOTOR1_PIN);
  NRF_P0->OUT |= (1 << MOTOR2_PIN);
  NRF_P1->OUT |= (1 << MOTOR3_PIN);
  NRF_P1->OUT |= (1 << MOTOR4_PIN);

  esc_timer[0] = c_time + esc_pwm[0];
  esc_timer[1] = c_time + esc_pwm[1];
  esc_timer[2] = c_time + esc_pwm[2];
  esc_timer[3] = c_time + esc_pwm[3];

  while (NRF_P1->OUTSET & (1 << MOTOR1_PIN) | NRF_P0->OUTSET & (1 << MOTOR2_PIN) | NRF_P1->OUTSET & (1 << MOTOR3_PIN) | NRF_P1->OUTSET & (1 << MOTOR4_PIN)) {
    c_time = micros();
    if (esc_timer[0] <= c_time) NRF_P1->OUT &= ~(1 << MOTOR1_PIN);
    if (esc_timer[1] <= c_time) NRF_P0->OUT &= ~(1 << MOTOR2_PIN);
    if (esc_timer[2] <= c_time) NRF_P1->OUT &= ~(1 << MOTOR3_PIN);
    if (esc_timer[3] <= c_time) NRF_P1->OUT &= ~(1 << MOTOR4_PIN);
  }

  while (micros() - loop_timer < (1000000/LOOP_RATE));
  //Serial.println(micros() - loop_timer);
  loop_timer = micros();

}

void setup_imu() {

  //Reset the IMU
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x22);
  Wire1.write(0x05);
  Wire1.endTransmission();

  //Set Gyro to 500dps with Sampling Rate 476Hz and LPF1, LPF2 at 100, 21Hz respectively.
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x10);
  Wire1.write(0xA8); // 500dps, 476Hz, 100Hz, 21Hz
  Wire1.endTransmission();

  //Set Acc to +/- 8g with Sampling Rate 476Hz and BW 211Hz, AntiAlias filter 408Hz
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x20);
  Wire1.write(0xB8); // 8g, 476Hz, 211Hz, 408Hz
  Wire1.endTransmission();

  //Enable FIFO
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x23);
  Wire1.write(0x02);
  Wire1.endTransmission();

  //Enable continuous mode operation
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x2E);
  Wire1.write(0xC0);
  Wire1.endTransmission();

}

void read_raw_imu() {

  Wire1.beginTransmission(0x6B);
  Wire1.write(0x18);
  Wire1.endTransmission();

  Wire1.requestFrom(0x6B, 6);
  raw_gyro_data[0] = (Wire1.read() | Wire1.read() << 8);
  raw_gyro_data[1] = (Wire1.read() | Wire1.read() << 8);
  raw_gyro_data[2] = (Wire1.read() | Wire1.read() << 8);

  Wire1.beginTransmission(0x6B);
  Wire1.write(0x28);
  Wire1.endTransmission();

  Wire1.requestFrom(0x6B, 6);
  raw_acc_data[0] = (Wire1.read() | Wire1.read() << 8);
  raw_acc_data[1] = (Wire1.read() | Wire1.read() << 8);
  raw_acc_data[2] = (Wire1.read() | Wire1.read() << 8);

}

void calibrate_imu() {

  for (int i = 0; i < 2000; i++) {

    read_raw_imu();

    gyro_offset[0] += raw_gyro_data[0];
    gyro_offset[1] += raw_gyro_data[1];
    gyro_offset[2] += raw_gyro_data[2];

  }

  gyro_offset[0] /= 2000;
  gyro_offset[1] /= 2000;
  gyro_offset[2] /= 2000;

}

void rc_roll() {
  if (!rc_pins_prev_data[0] && (NRF_P1->IN & (1 << RC_CH1_PIN))) {
    rc_pins_prev_data[0] = true;
    rc_channels_timer[0] = micros();
  }
  else if (rc_pins_prev_data[0] && !(NRF_P1->IN & (1 << RC_CH1_PIN))) {
    rc_pins_prev_data[0] = false;
    rc_channels[0] = micros() - rc_channels_timer[0];
  }
}

void rc_pitch() {
  if (!rc_pins_prev_data[1] && (NRF_P1->IN & (1 << RC_CH2_PIN))) {
    rc_pins_prev_data[1] = true;
    rc_channels_timer[1] = micros();
  }
  else if (rc_pins_prev_data[1] && !(NRF_P1->IN & (1 << RC_CH2_PIN))) {
    rc_pins_prev_data[1] = false;
    rc_channels[1] = micros() - rc_channels_timer[1];
    rc_channels[1] = -rc_channels[1] + 3000;
  }
}

void rc_thr() {
  if (!rc_pins_prev_data[2] && (NRF_P1->IN & (1 << RC_CH3_PIN))) {
    rc_pins_prev_data[2] = true;
    rc_channels_timer[2] = micros();
  }
  else if (rc_pins_prev_data[2] && !(NRF_P1->IN & (1 << RC_CH3_PIN))) {
    rc_pins_prev_data[2] = false;
    rc_channels[2] = micros() - rc_channels_timer[2];
  }
}

void rc_yaw() {
  if (!rc_pins_prev_data[3] && (NRF_P1->IN & (1 << RC_CH4_PIN))) {
    rc_pins_prev_data[3] = true;
    rc_channels_timer[3] = micros();
  }
  else if (rc_pins_prev_data[3] && !(NRF_P1->IN & (1 << RC_CH4_PIN))) {
    rc_pins_prev_data[3] = false;
    rc_channels[3] = micros() - rc_channels_timer[3];
  }
}

void rc_kill() {
  if (!rc_pins_prev_data[4] && (NRF_P1->IN & (1 << RC_CH5_PIN))) {
    rc_pins_prev_data[4] = true;
    rc_channels_timer[4] = micros();
  }
  else if (rc_pins_prev_data[4] && !(NRF_P1->IN & (1 << RC_CH5_PIN))) {
    rc_pins_prev_data[4] = false;
    rc_channels[4] = micros() - rc_channels_timer[4];
  }
}
