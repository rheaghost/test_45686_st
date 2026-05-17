​//Did not work properly but put in public just as for reference

#include <SPI.h>

#include "ICM45686.h"

​

#define CS_PIN PA2

​

ICM456xx IMU(SPI, CS_PIN);

​

#define IMU_RATE_HZ   8000

#define SERIAL_RATE_HZ 60

​

#define MA_WINDOW 8

​

// Raw buffers

float ax_buf[MA_WINDOW], ay_buf[MA_WINDOW], az_buf[MA_WINDOW];

float gx_buf[MA_WINDOW], gy_buf[MA_WINDOW], gz_buf[MA_WINDOW];

int ma_idx = 0;

​

// EKF state

float q0=1, q1=0, q2=0, q3=0;

float bgx=0, bgy=0, bgz=0;

​

// Covariance (diagonal only for simplicity)

float Pq = 0.01f;

float Pb = 0.001f;

​

uint32_t lastIMU = 0;

uint32_t lastSerial = 0;

​

void setup() {

  Serial.begin(115200);

  delay(1000);

​

  SPI.begin();

  pinMode(CS_PIN, OUTPUT);

  digitalWrite(CS_PIN, HIGH);

​

  if (IMU.begin() != 0) {

    Serial.println("IMU init failed");

    while (1);

  }

​

  IMU.startAccel(IMU_RATE_HZ, 16);

  IMU.startGyro(IMU_RATE_HZ, 2000);

​

  for (int i=0;i<MA_WINDOW;i++) {

    ax_buf[i]=ay_buf[i]=az_buf[i]=0;

    gx_buf[i]=gy_buf[i]=gz_buf[i]=0;

  }

​

  lastIMU = micros();

  lastSerial = micros();

}

// moving average helper

float avg(float *buf) {

  float s = 0;

  for (int i=0;i<MA_WINDOW;i++) s += buf[i];

  return s / MA_WINDOW;

}

​

// ekf core(imu only)

//quarternion normalize

void normalizeQuat() {

  float n = sqrt(q0*q0+q1*q1+q2*q2+q3*q3);

  q0/=n; q1/=n; q2/=n; q3/=n;

}

​

​

//predict step-gyro

void ekfPredict(float gx, float gy, float gz, float dt) {

  gx -= bgx; gy -= bgy; gz -= bgz;

​

  float dq0 = -0.5f*(q1*gx + q2*gy + q3*gz);

  float dq1 =  0.5f*(q0*gx + q2*gz - q3*gy);

  float dq2 =  0.5f*(q0*gy - q1*gz + q3*gx);

  float dq3 =  0.5f*(q0*gz + q1*gy - q2*gx);

​

  q0 += dq0*dt;

  q1 += dq1*dt;

  q2 += dq2*dt;

  q3 += dq3*dt;

​

  normalizeQuat();

​

  Pq += 0.0001f;  // process noise

  Pb += 0.000001f;

}

//upate step-accel gravity

void ekfUpdateAccel(float ax, float ay, float az) {

  float norm = sqrt(ax*ax+ay*ay+az*az);

  if (norm < 0.5f || norm > 1.5f) return;

​

  ax/=norm; ay/=norm; az/=norm;

​

  // expected gravity

  float vx = 2*(q1*q3 - q0*q2);

  float vy = 2*(q0*q1 + q2*q3);

  float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

​

  float ex = (ay*vz - az*vy);

  float ey = (az*vx - ax*vz);

  float ez = (ax*vy - ay*vx);

​

  float k = 0.5f;

​

  bgx += ex * Pb;

  bgy += ey * Pb;

  bgz += ez * Pb;

​

  q0 += (-q1*ex - q2*ey - q3*ez)*k;

  q1 += ( q0*ex + q2*ez - q3*ey)*k;

  q2 += ( q0*ey - q1*ez + q3*ex)*k;

  q3 += ( q0*ez + q1*ey - q2*ex)*k;

​

  normalizeQuat();

}

//euler conversion

​

void quatToEuler(float &roll, float &pitch, float &yaw) {

  roll  = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));

  pitch = asin(2*(q0*q2-q3*q1));

  yaw   = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));

}

​

//main loop

void loop() {

  uint32_t now = micros();

​

  if (now - lastIMU >= 125) { // 8 kHz

    lastIMU += 125;

​

    inv_imu_sensor_data_t d;

    IMU.getDataFromRegisters(d);

​

    ax_buf[ma_idx] = d.accel_data[0];

    ay_buf[ma_idx] = d.accel_data[1];

    az_buf[ma_idx] = d.accel_data[2];

    gx_buf[ma_idx] = d.gyro_data[0];

    gy_buf[ma_idx] = d.gyro_data[1];

    gz_buf[ma_idx] = d.gyro_data[2];

​

    ma_idx = (ma_idx + 1) % MA_WINDOW;

​

    float ax = avg(ax_buf);

    float ay = avg(ay_buf);

    float az = avg(az_buf);

    float gx = avg(gx_buf);

    float gy = avg(gy_buf);

    float gz = avg(gz_buf);

​

    ekfPredict(gx, gy, gz, 1.0f/IMU_RATE_HZ);

    ekfUpdateAccel(ax, ay, az);

  }

​

  if (now - lastSerial >= 16666) { // 60 Hz

    lastSerial += 16666;

​

    float roll, pitch, yaw;

    quatToEuler(roll, pitch, yaw);

​

    Serial.print("RPR:");

    Serial.print(roll*57.3f); Serial.print(",");

    Serial.print(pitch*57.3f); Serial.print(",");

    Serial.print(yaw*57.3f);

​

    Serial.print(" | A:");

    Serial.print(avg(ax_buf)); Serial.print(",");

    Serial.print(avg(ay_buf)); Serial.print(",");

    Serial.print(avg(az_buf));

​

    Serial.print(" | G:");

    Serial.print(avg(gx_buf)); Serial.print(",");

    Serial.print(avg(gy_buf)); Serial.print(",");

    Serial.print(avg(gz_buf));

​

    Serial.print(" | T:");

    //Serial.println(IMU.getTemperature());

    inv_imu_sensor_data_t imu_data;

    IMU.getDataFromRegisters(imu_data);

    Serial.print("Temperature:"); Serial.println(imu_data.temp_data);

​

  }

}
