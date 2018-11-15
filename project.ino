#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "SD.h"

int trigger = 9;
int echo = 10;
int duration;

int accelX; int accelY; int accelZ; float theta; // position data based on the MPU6050 unit
float phi; // Angle of tilt as calculated from MPU6050
float offsetX; float offsetY; // Offsets from starting position, as calculated from MPU6050
float ax; float k; float f; // Intermediate variables for position calc
float r; float t; // Radius and angle of detected point, in polar coordinates, where the starting point is the origin.
int x; int y; // Outputs (must be integers so matrix works)

int dt = 25; //Delay per loop in milliseconds

float getDistance(){
  digitalWrite(trigger, LOW);
  delayMicroseconds(2);
  digitalWrite(trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigger, LOW);
  duration = pulseIn(echo, 1);
  return duration*0.01715; // Distance = speed of sound * 1/2 delta t
}

void setup(){
  pinMode(trigger, OUTPUT);
  pinMode(echo, INPUT);
  Serial.begin(9600);
}

void loop(){
  // Serial.println(getDistance());
  delay(dt);
  // Bogus values:
  accelX = 0; accelY = 0; accelZ = 0; theta = 0;
  // We just measured and filtered our acclerations in x, y, and z and our angle in previous code.
  // Acceleration of gravity, m/s^2, is -9.8 (positive is up.)
  
  // Intermediate calculations to get angle of tilt and true distance
  k = (-9.8/accelX);
  phi = atan(k); // phi is the angle of vertical tilt
  f = pow(1+pow(k,2), -0.5);
  ax = accelZ*f + accelY*k*f; // Calculate acceleration in x (direction of sonar)
  offsetX = 0.5*ax*pow(dt,2); // Integrate acceleration to get position change
  offsetY = 0.5*accelY*pow(dt,2);
  r = getDistance()*cos(phi) + offsetX; // Adjusted radius based on tilt; assume walls are vertical and straight.
  t = theta+(offsetY/offsetX); // Small-angle approximation to get real angle (if we moved)
  x = r*cos(t);
  y = r*sin(t);
}
