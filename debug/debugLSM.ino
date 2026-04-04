#include <Wire.h>
#include <SparkFunLSM6DS3.h>

#define I2C_SDA 21
#define I2C_SCL 22

LSM6DS3 imu(I2C_MODE, 0x6B);

void setup()
{
  Serial.begin(115200);

  Wire.begin(I2C_SDA, I2C_SCL);

  Serial.println("LSM6DS3 init...");

  if(imu.begin()!=0)
  {
    Serial.println("IMU FAIL");
    while(1);
  }

  Serial.println("IMU OK");
}

void loop()
{
  float ax = imu.readFloatAccelX();
  float ay = imu.readFloatAccelY();
  float az = imu.readFloatAccelZ();

  float gx = imu.readFloatGyroX();
  float gy = imu.readFloatGyroY();
  float gz = imu.readFloatGyroZ();

  Serial.print("ACC: ");

  Serial.print(ax);
  Serial.print(" ");

  Serial.print(ay);
  Serial.print(" ");

  Serial.print(az);

  Serial.print(" | GYRO: ");

  Serial.print(gx);
  Serial.print(" ");

  Serial.print(gy);
  Serial.print(" ");

  Serial.println(gz);

  delay(100);
}