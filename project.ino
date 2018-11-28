//===============================================================================================================================================================================================================
//LIBRARIES
//===============================================================================================================================================================================================================
#include <Wire.h>
//#include <I2Cdev.h>
#include <MPU6050_tockn.h>
#include <SD.h>

// INITIALIZE MPU6050
MPU6050 mpu6050(Wire);

//===============================================================================================================================================================================================================
//CONSTANTS
//===============================================================================================================================================================================================================
//PINS
const int trigger = 8;
const int echo = 9;
const int chip = 10;
float duration, distance;

//FILTERING
const int max_size = 3;

int ring_buffer_dist[max_size];
int ring_index_dist = 0;
int l = 0;

//SONAR

//ACCELEROMETER/GYRO
//// By Arduino User JohnChi
//// August 17, 2014
//// Public Domain
//const int MPU_addr = 0x68; // I2C address of the MPU-6050
//int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float accelXoffset = 0, accelYoffset = 0, accelZoffset = 0;

float rax; float ray; float raz; // Raw accelerations in three directions
float accelX;
float accelY;
float accelZ;
float theta; // position data based on the MPU6050 unit
float phi; // Angles of tilt as calculated from MPU6050
float rho;
float vX = 0; float vY = 0; // Velocities for intermediate integration
float offsetX = 0;
float offsetY = 0; // Offsets from starting position, as calculated from MPU6050
float r;

float x;
float y; // Outputs to .csv file

int t;
float dt = 0.02; // Delay per loop in seconds
float gscale = 250 / 32768; // Scale of gyro rotational speed
float ascale = 981; // Scale of accelerometer (convert g's to cm/s/s)

String filename; // File name on SD card
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
  delay(40); // Wait for noise to decay (based on spec sheet recommendations)
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, 1);
  return (duration - 10) * 0.01715; // Distance = speed of sound * 1/2 delta t
  // New fix: subtract 10 to account for the original width of the pulse.
  // The echo is as long as the original pulse plus twice the distance / the speed of sound.
}


//===============================================================================================================================================================================================================
//MAIN
//===============================================================================================================================================================================================================
void setup() {

  Wire.begin();
  pinMode(trigger, OUTPUT); // Initialize sonar sensor pins
  pinMode(echo, INPUT);

  Serial.begin(9600);
  Serial.println("Set the device on a flat surface.");

  // Initialize MPU6050
  mpu6050.begin();
  //Calculate accelerometer and gyro offsets.
  mpu6050.calcGyroOffsets(false);

  int16_t rx, ry, rz;

  for (int i = 0; i < 3000; i++) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((int)MPU6050_ADDR, 14, (int)true);

    rx = Wire.read() << 8 | Wire.read();
    ry = Wire.read() << 8 | Wire.read();
    rz = Wire.read() << 8 | Wire.read();

    accelXoffset += ((float)rx) / 49152000;
    accelYoffset += ((float)ry) / 49152000;
    accelZoffset += ((float)rz) / 49152000;
  }

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chip)) {
    Serial.println("Card failed, or not present");
    // don't run the program
    while (1);
  }
  // Create a new file and don't overwrite an old one
  int i = 1;
  filename = "output" + String(i) + ".csv";
  while (SD.exists(filename)) {
    i++;
    filename = "output" + String(i) + ".csv";
  }

}

void loop() {

  t = millis();
  
  // Get a filtered distance reading from the sonar
  int dist_avg = running_avg(getDistance(), ring_buffer_dist, ring_index_dist);
  ring_index_dist = (ring_index_dist + 1) % max_size;
  //Serial.println(getDistance());

  // Read values from MPU-6050
  mpu6050.update(); // Library-defined function to grab values from MPU6050
  phi = PI * mpu6050.getGyroAngleY() / 180; // phi is the angle of vertical tilt: radians
  rho = PI * mpu6050.getGyroAngleX() / 180; // rho is the angle of sideways tilt: radians


  rax = mpu6050.getAccX() - accelXoffset;
  ray = mpu6050.getAccY() - accelYoffset;
  raz = -1 * (mpu6050.getAccZ() - accelZoffset); // Accelerometer z axis is upside down and we zeroed it

  // Get horizontal offset
  // Add acceleration vectors (based on angles) to get true accelerations (eliminate tilt)
  accelX = ascale * (rax * cos(phi) + raz * sin(phi)); // Calculate acceleration in x (direction of sonar)
  accelY = ascale * (ray * cos(rho) + raz * sin(phi)); // Calculate acceleration in y (side-to-side)
  vX = vX + accelX * dt;
  vY = vY + accelY * dt; // Integrate acceleration to get velocity;
  //  offsetX = offsetX + vX * dt; // integrate acceleration to get position change.
  //  offsetY = offsetY + vY * dt;
  offsetX = 0; offsetY = 0;
  //r = getDistance() * cos(phi) + offsetX; // Adjusted radius based on tilt; assume walls are vertical and straight.
  r = dist_avg * cos(phi); // Adjusted radius based on tilt; assume walls are vertical and straight.
  theta = -1* PI * mpu6050.getAngleZ() / 180; // The gyro is flipped.
  //  Serial.print("X: "); Serial.print(rax); Serial.print(", Y: "); Serial.print(ray); Serial.print(", Z: "); Serial.println(raz);
  //  Serial.print("X = "); Serial.print(vX); Serial.print(", Y = "); Serial.println(vY);
  // Serial.print("X = "); Serial.print(offsetX); Serial.print(", Y = "); Serial.println(offsetY);
  x = r * cos(theta) + offsetX;
  y = r * sin(theta) + offsetY;
  
  if(l>max_size){ // Let the ring buffer initialize before recording any data
  File output = SD.open(filename, FILE_WRITE);
  output.print(x); output.print(","); output.println(y);
  output.close();}
  
  Serial.print("X = "); Serial.print(x); Serial.print(", Y = "); Serial.println(y);
  l++; //File just got longer; increment the length counter.
  dt = 0.001*(millis() - t); // Read change in time (seconds)
}
