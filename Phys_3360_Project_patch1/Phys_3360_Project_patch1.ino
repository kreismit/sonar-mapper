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

float ring_buffer_dist[max_size];
int ring_index_dist = 0;

float ring_buffer_X[max_size];
float ring_buffer_Y[max_size];
int ring_index_X = 0;
int ring_index_Y = 0;
int l = 0;

//SONAR

//ACCELEROMETER/GYRO
float accelXoffset = 0, accelYoffset = 0, accelZoffset = 0;

float rax, ray, raz; // Raw accelerations in three directions
float accelX, accelY, accelZ, theta, theta_deg; // position data based on the MPU6050 unit
float phi, rho; // Angles of tilt as calculated from MPU6050
double gxo, gyo; // Gyro integration offsets
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
int running_avg(int val, float ring_buffer[], int ring_index) {
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
 
  return (duration) * 0.01715; // Distance = speed of sound * 1/2 delta t
  
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
  Serial.println("Calibrating IMU...");

  // Initialize MPU6050
  mpu6050.begin();
  //Calculate accelerometer and gyro offsets.
  mpu6050.calcGyroOffsets(false);
  delay(1000);
  int16_t rx, ry, rz;

  for (int i = 0; i < 3000; i++) {
    mpu6050.update();

    accelXoffset += mpu6050.getAccX() *0.327;
    accelYoffset += mpu6050.getAccY() *0.327;
    accelZoffset += mpu6050.getAccZ() *0.327;
    
    gxo += mpu6050.getGyroAngleX() / 3000; // Gyro X Offset: averaged over 3000 cycles
    gyo += mpu6050.getGyroAngleY() / 3000; // Gyro Y Offset
    
    // Get difference between gyro-calculated angles and accel-calculated angles
    gxo -= mpu6050.getAccAngleX() / 3000;
    gyo -= mpu6050.getAccAngleY() / 3000;
    // Subtract gxo and gyo from the gyro-calculated angles to get accurate angles which don't depend on acceleration.
  }
  
  Serial.println("");
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chip)) {
    Serial.println("");
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
  Serial.println(" Done!");
}

void loop() {
  
  // Get a filtered distance reading from the sonar
  float dist_avg = running_avg(getDistance(), ring_buffer_dist, ring_index_dist);
  ring_index_dist = (ring_index_dist + 1) % max_size;
  
  // Read values from MPU-6050
  mpu6050.update(); // Library-defined function to grab values from MPU6050


  //rax = mpu6050.getAccX() - accelXoffset;
  //ray = mpu6050.getAccY() - accelYoffset;
  //rax = mpu6050.getAccX();
  //ray = mpu6050.getAccY();
  //raz = -1 * (mpu6050.getAccZ() - accelZoffset); // Accelerometer z axis is upside down and we zeroed it

  //phi = (mpu6050.getGyroAngleY() - gyo) * 0.017453293; // phi is the angle of vertical tilt: radians
  //rho = (mpu6050.getGyroAngleX() - gxo) * 0.017453293; // rho is the angle of sideways tilt: radians
  theta_deg =  mpu6050.getAngleZ();
  theta = theta_deg*(PI/ 180);
  

  // Get horizontal offset
  // Add acceleration vectors (based on angles) to get true accelerations (eliminate tilt)
  //accelX = ascale * (rax * cos(phi)+raz*sin(phi)); // Calculate acceleration in x (direction of sonar)
  //accelY = ascale * (ray * cos(rho)+raz*sin(rho)); // Calculate acceleration in y (side-to-side)
  //vX = vX + accelX * dt;
  //vY = vY + accelY * dt; // Integrate acceleration to get velocity;
  //offsetX = offsetX + vX * dt; // integrate acceleration to get position change.
  //offsetY = offsetY + vY * dt;

  
  r = dist_avg; // The sonar will return about the same distance within a +/- 15-degree angle range.
                // Thus, no need to correct for angle of tilt.


  x = r * cos(theta);
  y = r * sin(theta);


  //WRITE TO SD CARD
  if(l>max_size && ((int)theta_deg)%15==0){ // Let the ring buffer initialize before recording any data/only take measurements every 15 degrees
  
  File output = SD.open(filename, FILE_WRITE);
  output.print(x); output.print(","); output.println(y);
  output.close();}
  
  
  l++; //File just got longer; increment the length counter.
  dt = 0.001*(millis() - t); // Read change in time (seconds)
}
