
​//Module was Aliexpress 601N module ,the backside jumper should be welded shut for spi
// Code worked then , the module has an ARM chip for slimevr purposes
///45686 module   bp function bp port

//SCL   SCLK   SCL   SCLK1  PA5

//SDA   SDI    SDA   MOSI1  PA7

//SAO   SDO    SAO   MISO1  PA6

//CS    (port) CS    PA2

​

​

#include <SPI.h>

#include "ICM45686.h"

​

#define CS_PIN PA2  // any GPIO

ICM456xx IMU(SPI, CS_PIN);  // Use default SPI (SPI1)

​

void setup() {

  Serial.begin(115200);

  delay(1000);

  Serial.println("*** ICM45686 SPI Test ***");

​

  SPI.begin();  // uses default pins: PA5=SCK, PA6=MISO, PA7=MOSI

  pinMode(CS_PIN, OUTPUT);                                    

  digitalWrite(CS_PIN, HIGH);  // Deselect

​

  if (IMU.begin() != 0) {

    Serial.println("IMU init failed");

    while (1);

  }

​

  IMU.startAccel(100, 16);

  IMU.startGyro(100, 2000);

  delay(100);

}

​

void loop() {

  inv_imu_sensor_data_t imu_data;

  IMU.getDataFromRegisters(imu_data);

​

  Serial.print("AccelX:"); Serial.println(imu_data.accel_data[0]);

  Serial.print("AccelY:"); Serial.println(imu_data.accel_data[1]);

  Serial.print("AccelZ:"); Serial.println(imu_data.accel_data[2]);

  Serial.print("GyroX:"); Serial.println(imu_data.gyro_data[0]);

  Serial.print("GyroY:"); Serial.println(imu_data.gyro_data[1]);

  Serial.print("GyroZ:"); Serial.println(imu_data.gyro_data[2]);

  Serial.print("Temperature:"); Serial.println(imu_data.temp_data);

  delay(10);  // 100Hz

}

​
