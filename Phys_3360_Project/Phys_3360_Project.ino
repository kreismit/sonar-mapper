//===============================================================================================================================================================================================================
//LIBRARIES
//===============================================================================================================================================================================================================
#include<Wire.h>


//===============================================================================================================================================================================================================
//CONSTANTS
//===============================================================================================================================================================================================================
//PINS
const int trigPin = 9; 
const int echoPin = 10;

//FILTERING
const int max_size=10;

int ring_buffer_AcX[max_size];
int ring_index_AcX=0;

int ring_buffer_AcY[max_size];
int ring_index_AcY=0;

int ring_buffer_AcZ[max_size];
int ring_index_AcZ=0;

int ring_buffer_GyX[max_size];
int ring_index_GyX=0;

int ring_buffer_GyY[max_size];
int ring_index_GyY=0;

int ring_buffer_GyZ[max_size];
int ring_index_GyZ=0;

//SONAR
float duration, distance; 

//ACCELEROMETER/GYRO
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

//===============================================================================================================================================================================================================
//FUNCTIONS
//===============================================================================================================================================================================================================
int running_avg(int val, int ring_buffer[], int ring_index){
  //ring buffer
  //scale val so we don't run out of bits
  ring_buffer[ring_index]=val/max_size;
  
  //summing over the entire ring_buffer
  int sum=0;
  for (int i=0; i<max_size; i++){
    sum=sum+ring_buffer[i];
  }

  //return running avg
  return sum;
}

//returns distance in cm of object the sensor is detecting
int get_distance (){
 digitalWrite(trigPin, LOW); 
 delayMicroseconds(2); 
 digitalWrite(trigPin, HIGH); 
 delayMicroseconds(10); 
 digitalWrite(trigPin, LOW); 

 duration = pulseIn(echoPin, HIGH);
 distance = (duration*.0343)/2; 
}

//===============================================================================================================================================================================================================
//MAIN
//===============================================================================================================================================================================================================
void setup() {
 pinMode(trigPin, OUTPUT); 
 pinMode(echoPin, INPUT); 

 Wire.begin();
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);
  
 Serial.begin(9600); 
}

void loop() {
 //READING RAW VALUES FROM MPU_6050
   Wire.beginTransmission(MPU_addr);
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
   AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
   AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   //Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
   GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
   GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
   GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

 //FILTERING RAW VALUES
  int AcX_avg=running_avg(AcX, ring_buffer_AcX, ring_index_AcX);
  ring_index_AcX= (ring_index_AcX+1)%max_size;
  
  int AcY_avg=running_avg(AcY, ring_buffer_AcY, ring_index_AcY);
  ring_index_AcY= (ring_index_AcY+1)%max_size;
   
  int AcZ_avg=running_avg(AcZ, ring_buffer_AcZ, ring_index_AcZ);
  ring_index_AcZ= (ring_index_AcZ+1)%max_size;

  int GyX_avg=running_avg(GyX, ring_buffer_GyX, ring_index_GyX);
  ring_index_GyX= (ring_index_GyX+1)%max_size;
  
  int GyY_avg=running_avg(GyY, ring_buffer_GyY, ring_index_GyY);
  ring_index_GyY= (ring_index_GyY+1)%max_size;
   
  int GyZ_avg=running_avg(GyZ, ring_buffer_GyZ, ring_index_GyZ);
  ring_index_GyZ= (ring_index_GyZ+1)%max_size;

//DOING COORDINATE CALCULATIONS FROM MPU_6050 AND SONAR VALUES

 accelX = AcX_avg; accelY = AcY_avg; accelZ = AcZ_avg; theta = 0;
 
 // We just measured and filtered our acclerations in x, y, and z and our angle in previous code.
 // Acceleration of gravity, m/s^2, is -9.8 (positive is up.)
  
 // Intermediate calculations to get angle of tilt and true distance
  k = (-9.8/accelX);
  phi = atan(k); // phi is the angle of vertical tilt
  f = pow(1+pow(k,2), -0.5);
  ax = accelZ*f + accelY*k*f; // Calculate acceleration in x (direction of sonar)
  offsetX = 0.5*ax*pow(dt,2); // Integrate acceleration to get position change
  offsetY = 0.5*accelY*pow(dt,2);
  r = get_distance()*cos(phi) + offsetX; // Adjusted radius based on tilt; assume walls are vertical and straight.
  t = theta+(offsetY/offsetX); // Small-angle approximation to get real angle (if we moved)

  x = r*cos(t);
  y = r*sin(t);

// Serial.print("AcX = ");Serial.print(AcX_avg);
// Serial.print(" | AcY = ");Serial.print(AcY_avg);
// Serial.print(" | AcZ = "); Serial.print(AcZ_avg);
// Serial.print(" | GyX = "); Serial.print(GyX_avg);
// Serial.print(" | GyY = "); Serial.print(GyY_avg);
// Serial.print(" | GyZ = "); Serial.println(GyZ_avg);

 
} 
