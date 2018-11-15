const int trigPin = 9; 
const int echoPin = 10;

//array stuff
const int max_size=10;

int ring_buffer_AcX[max_size];
int ring_index_AcX=0;

int ring_buffer_AcY[max_size];
int ring_index_AcY=0;

int ring_buffer_AcZ[max_size];
int ring_index_AcZ=0;




float duration, distance; 

// MPU-6050 Short Example Sketch
// By Arduino User JohnChi
// August 17, 2014
// Public Domain
#include<Wire.h>
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

void setup() {
//  // put your setup code here, to run once:
// pinMode(trigPin, OUTPUT); 
// pinMode(echoPin, INPUT); 

 Wire.begin();
 Wire.beginTransmission(MPU_addr);
 Wire.write(0x6B);  // PWR_MGMT_1 register
 Wire.write(0);     // set to zero (wakes up the MPU-6050)
 Wire.endTransmission(true);
  
 Serial.begin(9600); 
}

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

void loop() {
//  // put your main code here, to run repeatedly:
// digitalWrite(trigPin, LOW); 
// delayMicroseconds(2); 
// digitalWrite(trigPin, HIGH); 
// delayMicroseconds(10); 
// digitalWrite(trigPin, LOW); 
//
// duration = pulseIn(echoPin, HIGH);
// distance = (duration*.0343)/2; 
// delay(100);
// Serial.print("Distance: "); 
// Serial.println(distance); 
//
// delay(100);


 Wire.beginTransmission(MPU_addr);
 Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
 Wire.endTransmission(false);
 Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
 AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
 AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
 AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
 Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
 GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
 GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
 GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

int AcX_avg=running_avg(AcX, ring_buffer_AcX, ring_index_AcX);
ring_index_AcX= (ring_index_AcX+1)%max_size;

int AcY_avg=running_avg(AcY, ring_buffer_AcY, ring_index_AcY);
ring_index_AcY= (ring_index_AcY+1)%max_size;
 
int AcZ_avg=running_avg(AcZ, ring_buffer_AcZ, ring_index_AcZ);
ring_index_AcZ= (ring_index_AcZ+1)%max_size;


 Serial.print("AcX = ");Serial.print(AcX_avg);
 Serial.print(" | AcY = ");Serial.print(AcY_avg);
 Serial.print(" | AcZ = "); Serial.println(AcZ_avg);
// Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
// Serial.print(" | GyX = "); Serial.print(GyX);
// Serial.print(" | GyY = "); Serial.print(GyY);
// Serial.print(" | GyZ = "); Serial.println(GyZ);
 //delay(333);

 
} 
