//===============================================================================================================================================================================================================
//LIBRARIES
//===============================================================================================================================================================================================================
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_tockn.h>
#include <SD.h>

//Set up MPU-6050 functions

MPU6050 mpu6050(Wire);

//===============================================================================================================================================================================================================
//CONSTANTS
//===============================================================================================================================================================================================================
//PINS
const int trigger = 9;
const int echo = 10;
float duration, distance;

//FILTERING
const int max_size = 10;

int ring_buffer_AcX[max_size];
int ring_index_AcX = 0;

int ring_buffer_AcY[max_size];
int ring_index_AcY = 0;

int ring_buffer_AcZ[max_size];
int ring_index_AcZ = 0;

//SONAR

//ACCELEROMETER/GYRO
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
const int MPU_addr = 0x68; // I2C address of the MPU-6050
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;


float accelX;
float accelY;
float accelZ;
float theta; // position data based on the MPU6050 unit
float phi; // Angles of tilt as calculated from MPU6050
float rho;
float vX = 0; float vY = 0;
float offsetX = 0;
float offsetY = 0; // Offsets from starting position, as calculated from MPU6050
//float k;
//float f; // Intermediate variables for position calc
float r;
float t; // Radius and angle of detected point, in polar coordinates, where the starting point is the origin.
float x;
float y; // Outputs (must be integers so matrix works)

int t1;
float dt = 0.010; // Delay per loop in seconds
float dt2 = pow(dt, 2); // Delta t squared (pre-calculated to save time)
float gscale = 250 / 32768; // Scale of gyro rotational speed
//float ascale = 9.8; // Scale of accelerometer (convert g's to m/s/s)
float ascale = 9.8 / 16384; // Scale of accelerometer

//===============================================================================================================================================================================================================
//FUNCTIONS
//===============================================================================================================================================================================================================
int running_avg(int val, int ring_buffer[], int ring_index) {
  //ring buffer
  //scale val so we don't run out of bits
  ring_buffer[ring_index] = val / max_size;

  //summing over the entire ring_buffer
  int sum = 0;
  for (int i = 0; i < max_size; i++) {
    sum = sum + ring_buffer[i];
  }

  //return running avg
  return sum;
}

float getDistance() { // Find the distance (cm) from the sonar sensor to the nearest object in range
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, 1);
  return (duration-10) * 0.01715; // Distance = speed of sound * 1/2 delta t
  // New fix: subtract 10 to account for the original width of the pulse.
  // The echo is as long as the original pulse plus twice the distance / the speed of sound.
}


//===============================================================================================================================================================================================================
//MAIN
//===============================================================================================================================================================================================================
void setup() {
  pinMode(trigger, OUTPUT); // Initialize sonar sensor pins
  pinMode(echo, INPUT);
  Wire.begin();

  Serial.begin(9600);
  Serial.println("Set the device on a flat surface.");
  delay(100);
  // INITIALIZE MPU6050
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

}

void loop() {
  mpu6050.update(); // Library-defined function to grab values from MPU6050
  //READING RAW VALUES FROM MPU_6050
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  //FILTERING RAW VALUES
  int AcX_avg = running_avg(AcX, ring_buffer_AcX, ring_index_AcX);
  ring_index_AcX = (ring_index_AcX + 1) % max_size;

  int AcY_avg = running_avg(AcY, ring_buffer_AcY, ring_index_AcY);
  ring_index_AcY = (ring_index_AcY + 1) % max_size;

  int AcZ_avg = running_avg(AcZ, ring_buffer_AcZ, ring_index_AcZ);
  ring_index_AcZ = (ring_index_AcZ + 1) % max_size;

  // Serial.println(getDistance());
  delay(dt * 1000);
  // Acceleration of gravity, m/s^2, is -9.8 (positive is up.)
  //  accelX = AcX_avg;
  //  accelY = AcY_avg;
  //  accelZ = AcZ_avg;
  // Get horizontal offset
  //phi = PI * mpu6050.getAngleY() / 180; // phi is the angle of vertical tilt: radians
  //rho = PI * mpu6050.getAngleX() / 180; // rho is the angle of sideways tilt: radians
  phi = PI * AcY_avg / 180; // phi is the angle of vertical tilt: radians
  rho = PI * AcX_avg / 180; // rho is the angle of sideways tilt: radians
  // Add acceleration vectors (based on angles) to get true accelerations (eliminate tilt)
  //accelX = ascale * (mpu6050.getAccX() * cos(phi) + (mpu6050.getAccZ() * -1) * sin(phi)); // Calculate acceleration in x (direction of sonar)
  //accelY = ascale * (mpu6050.getAccY() * cos(rho) + (mpu6050.getAccZ() * -1) * sin(phi)); // Calculate acceleration in y (side-to-side)
  accelX = ascale * (AcX_avg * cos(phi) + (AcZ_avg * -1) * sin(phi)); // Calculate acceleration in x (direction of sonar)
  accelY = ascale * (AcY_avg * cos(rho) + (AcZ_avg * -1) * sin(phi)); // Calculate acceleration in y (side-to-side)
  vX = accelX * dt; vY = accelY * dt; // Integrate acceleration to get velocity;
  offsetX = offsetX + vX * dt + 0.5 * accelX * dt2; // Integrate acceleration to get position change
  offsetY = offsetY + vY * dt + 0.5 * accelY * dt2;
  r = getDistance() * cos(phi) + offsetX; // Adjusted radius based on tilt; assume walls are vertical and straight.
  theta = PI * mpu6050.getAngleZ() / 180;
  Serial.print("\nx = ");
  Serial.print(offsetX);
  //Serial.print(accelX);
  Serial.print("\ty =");
  Serial.println(offsetY);
  //Serial.println(accelY);
  //  t = theta+(offsetY/offsetX); // Small-angle approximation to get real angle (if we moved)
  //  x = r*cos(t);
  //  y = r*sin(t);
  //dt = 0.001*(millis() - t1);
  //t1 = millis();
}
