#include <Wire.h>

#define R2D 57.29578
#define D2R 0.0174533

//Very important line. Refer to following link for the fix:
//https://forum.arduino.cc/t/advice-i2c-communication-with-internal-sensors-ble-ble-sense/613815
extern TwoWire Wire1;

int16_t raw_gyro_x, raw_gyro_y, raw_gyro_z;
int16_t raw_acc_x, raw_acc_y, raw_acc_z;

float gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z;
long gyro_x_offset, gyro_y_offset, gyro_z_offset;

float raw_angle_roll, raw_angle_pitch, raw_angle_yaw;
float gyro_angle_pitch, gyro_angle_roll;
float acc_angle_pitch, acc_angle_roll;
float angle_roll, angle_pitch, angle_yaw;

bool imu_start = false;

long loop_timer;

void setup() {
  Serial.begin(9600);

  Wire1.begin();

  //For more information, refer to LSM9DS1 datasheet
  //https://www.digikey.pt/htmldatasheets/production/1639232/0/0/1/LSM9DS1-Datasheet.pdf

  //Reset the IMU
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x22);
  Wire1.write(0x05);
  Wire1.endTransmission();

  //Set Gyro to 500dps with Sampling Rate 476Hz and LPF1, LPF2 at 100, 21Hz respectively.
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x10);
  //Wire1.write(0x78);
  //Wire1.write(0xAB);
  //Wire1.write(0x88); // 500dps, 238Hz, 76Hz, 14Hz
  Wire1.write(0xA8); // 500dps, 476Hz, 100Hz, 21Hz
  Wire1.endTransmission();

  //Set Acc to +/- 8g with Sampling Rate 476Hz and BW 211Hz, AntiAlias filter 408Hz
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x20);
  //Wire1.write(0x70);
  //Wire1.write(0xB8);
  //Wire1.write(0x98); // 8g, 238Hz, 105Hz, 408Hz
  Wire1.write(0xB8); // 8g, 476Hz, 211Hz, 408Hz
  Wire1.endTransmission();

  //Enable FIFO
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x23);
  Wire1.write(0x02);
  Wire1.endTransmission();

  //Enable continuous mode operation
  Wire1.beginTransmission(0x6B);
  Wire1.write(0xC0);
  Wire1.endTransmission();
  
  Serial.println("Calibrating the IMU .....");

  for(int i=0;i < 5000;i++){
    
    read_raw_imu();

    gyro_x_offset += raw_gyro_x;
    gyro_y_offset += raw_gyro_y;
    gyro_z_offset += raw_gyro_z;
    
  }

  gyro_x_offset /= 5000;
  gyro_y_offset /= 5000;
  gyro_z_offset /= 5000;

  loop_timer = micros();
}

void loop() {
  /*
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x0F);
  Wire1.endTransmission();

  Wire1.requestFrom(0x6B, 1);
  whoami = Wire1.read();

  Serial.println(whoami);
  */
  read_raw_imu();

  gyro_x = (float)(raw_gyro_x - gyro_x_offset) * 0.0175;
  gyro_y = (float)(raw_gyro_y - gyro_y_offset) * 0.0175;
  gyro_z = (float)(raw_gyro_z - gyro_z_offset) * 0.0175;
  /*gyro_x = (float)(raw_gyro_x - gyro_x_offset) * 500 / 32768;
  gyro_y = (float)(raw_gyro_y - gyro_y_offset) * 500 / 32768;
  gyro_z = (float)(raw_gyro_z - gyro_z_offset) * 500 / 32768;
  */
  acc_x = (float)raw_acc_x * 0.000244;
  acc_y = (float)raw_acc_y * 0.000244;
  acc_z = (float)raw_acc_z * 0.000244;

  gyro_angle_roll -= (gyro_x * 0.004);
  gyro_angle_pitch -= (gyro_y * 0.004);
  
  gyro_angle_pitch -= gyro_angle_roll * sin(gyro_z * D2R * 0.004);
  gyro_angle_roll += gyro_angle_pitch * sin(gyro_z * D2R * 0.004);

  acc_angle_pitch = asin((float)acc_y / sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z))) * R2D;
  acc_angle_roll = -asin((float)acc_x / sqrt((acc_x * acc_x) + (acc_y * acc_y) + (acc_z * acc_z))) * R2D;

  if(!imu_start){
    gyro_angle_pitch = acc_angle_pitch;
    gyro_angle_roll = acc_angle_roll;
    imu_start = true;
  }
  else{
    //Fuse using complementary filter
    gyro_angle_pitch = gyro_angle_pitch * 0.995 + acc_angle_pitch * 0.005;
    gyro_angle_roll = gyro_angle_roll * 0.995 + acc_angle_roll * 0.005;
  }

  //EWMA filter to reduce noise
  angle_pitch = 0.9 * angle_pitch + 0.1 * gyro_angle_pitch;
  angle_roll = 0.9 * angle_roll + 0.1 * gyro_angle_roll;

  Serial.print(angle_pitch);Serial.print("\t");Serial.println(angle_roll);

  while(micros() - loop_timer < 4000);
  loop_timer = micros();
  
}

void read_raw_imu(){

  //Read raw gyro
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x18);
  Wire1.endTransmission();

  Wire1.requestFrom(0x6B, 6);
  raw_gyro_x = (Wire1.read() | Wire1.read() << 8);
  raw_gyro_y = (Wire1.read() | Wire1.read() << 8);
  raw_gyro_z = (Wire1.read() | Wire1.read() << 8);

  //Read raw accel
  Wire1.beginTransmission(0x6B);
  Wire1.write(0x28);
  Wire1.endTransmission();

  Wire1.requestFrom(0x6B, 6);
  raw_acc_x = (Wire1.read() | Wire1.read() << 8);
  raw_acc_y = (Wire1.read() | Wire1.read() << 8);
  raw_acc_z = (Wire1.read() | Wire1.read() << 8);
  
}
