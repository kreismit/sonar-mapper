// Define pins 
#define SDA_PORT PORTC
#define SDA_PIN 4 // = A4
#define SCL_PORT PORTC
#define SCL_PIN 5 // = A5

// Libraries
#include <SoftWire.h>

// initialize and instantiate stuff
SoftWire Wire = SoftWire();
float duration, distance; 
const int MPU_addr=0x68;  // I2C address of the MPU-6050
const int sonar = 0xE0; // I2C address of MB1242
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ; // Acceleration and gyro readings
float xoffset, yoffset, zoffset; // for gyro calibration
float roll = 0, pitch = 0, yaw = 0;
int range;
int tic, toc; float dt; // vars for measuring time delay
bool console = false; // Enable debugging output?
// true --> show on Arduino Serial Monitor
// false --> show on PLX-DAQ
float x, y, z; // vars for position outputs
int d = 30; // number of milliseconds to delay

/*                   Arduino I2C for a MaxSonar                         */
//////////////////////////////////////////////////////////////////////////
//  Arduino I2C for a MaxSonar by Carl Myhre is licensed under a        //
//  Creative Commons Attribution-ShareAlike 4.0 International License.  //
//  Original Author:  Carl Myhre, 10-02-2014, Revision: 1.0             //
//  Modifications by:                                                   //
//                                                                      //
//  Revision History: 1.0 -- 10-02-2014 -- Created initial code build   //
//                                                                      //
//  The original I2C libraries were created by Peter Fleury             //
//    http://homepage.hispeed.ch/peterfleury/avr-software.html          //
//                                                                      //
//  These libraries were adapted by Bernhard Nebel for use on Arduino   //
//    https://github.com/felias-fogg/SoftI2CMaster                      //
//                                                                      //
//  Special Thanks to MaxBotix Inc. for sponsoring this project!        //
//    http://www.maxbotix.com -- High Performance Ultrasonic Sensors    //
//                                                                      //
//  For more information on installing the I2C libraries for Arduino    //
//    visit http://playground.arduino.cc/Main/SoftwareI2CLibrary        //
//////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////
// Function: Start a range reading on the sensor //
///////////////////////////////////////////////////
//Uses the I2C library to start a sensor at the given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte bit8address = the address of the sensor that we want to command a range reading
//OUPUTS: bit  errorlevel = reports if the function was successful in taking a range reading: 1 = the function
//  had an error, 0 = the function was successful
boolean start_sensor(byte bit8address){
  boolean errorlevel = 0;
  bit8address = bit8address & B11111110;               //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;   //Run i2c_start(address) while doing so, collect any errors where 1 = there was an error.
  errorlevel = !i2c_write(81) | errorlevel;            //Send the 'take range reading' command. (notice how the library has error = 0 so I had to use "!" (not) to invert the error)
  i2c_stop();
  return errorlevel;
}



///////////////////////////////////////////////////////////////////////
// Function: Read the range from the sensor at the specified address //
///////////////////////////////////////////////////////////////////////
//Uses the I2C library to read a sensor at the given address
//Collects errors and reports an invalid range of "0" if there was a problem.
//INPUTS: byte  bit8address = the address of the sensor to read from
//OUPUTS: int   range = the distance in cm that the sensor reported; if "0" there was a communication error
int read_sensor(byte bit8address){
  boolean errorlevel = 0;
  int range = 0;
  byte range_highbyte = 0;
  byte range_lowbyte = 0;
  bit8address = bit8address | B00000001;  //Do a bitwise 'or' operation to force the last bit to be 'one' -- we are reading from the address.
  errorlevel = !i2c_start(bit8address) | errorlevel;
  range_highbyte = i2c_read(0);           //Read a byte and send an ACK (acknowledge)
  range_lowbyte  = i2c_read(1);           //Read a byte and send a NACK to terminate the transmission
  i2c_stop();
  range = (range_highbyte * 256) + range_lowbyte;  //compile the range integer from the two bytes received.
  if(errorlevel){
    return 0;
  }
  else{
    return range;
  }
}



/////////////////////////////////////////
// Function: Change the sensor address //
/////////////////////////////////////////
//Uses the I2C library to change the address of a sensor at a given address
//Collects and reports an error bit where: 1 = there was an error or 0 = there was no error.
//INPUTS: byte oldaddress = the current address of the sensor that we want to change
//INPUTS: byte newddress  = the address that we want to change the sensor to
//OUPUTS: bit  errorlevel = reports if the function was successful in changing the address: 1 = the function had an
//      error, 0 = the function was successful
boolean change_address(byte oldaddress,byte newaddress){
  //note that the new address will only work as an even number (odd numbers will round down)
  boolean errorlevel = 0;
  oldaddress = oldaddress & B11111110;  //Do a bitwise 'and' operation to force the last bit to be zero -- we are writing to the address.
  errorlevel = !i2c_start(oldaddress) | errorlevel; //Start communication at the new address and track error codes
  errorlevel = !i2c_write(170) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(165) | errorlevel;        //Send the unlock code and track the error codes
  errorlevel = !i2c_write(newaddress) | errorlevel; //Send the new address
  i2c_stop();
  return errorlevel;
}

// Function: read gyro values from MPU-6050
void imuRead(){
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
}
void setup() {

  //Awaken the MPU6050
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  
  Serial.begin(115200);
  
  // Awaken the MB1242
  int error = start_sensor(sonar);
  if(error && console){ Serial.println("Failed to start the MB1242.");}
  
  if(console){
    Serial.println("Hold still!");
    Serial.println();
    Serial.print("Calibrating IMU");
  }
  // Calculate offsets for gyro readings
  int i; int samples = 500; tic = millis(); float dttotal=0;
  // and show a nice progress bar of periods
  int progressbarlength = 20; int divisor = samples/progressbarlength;
  for (i = 0; i < samples; i++) {
    if(( (int) i/divisor == (float) i/divisor) && console){
      Serial.print(".");
    }
    imuRead();
    // numerically integrate the gyros to find the offset per second
    toc = millis();
    dt = (toc - tic)*0.001;
    dttotal+= dt;
    tic = toc;
    xoffset+= dt * ((float)GyX / 131);
    yoffset+= dt * ((float)GyY / 131);
    zoffset+= dt * ((float)GyZ / 131);
    delay(d);
  }
  if(console){ Serial.println(""); } // line break
  // convert offsets to degrees per second
  xoffset = xoffset / dttotal;
  yoffset = yoffset / dttotal;
  zoffset = zoffset / dttotal;
  Serial.print("Total calibration time (ms): "); Serial.println(dttotal*1000);
  Serial.print("X axis offset: "); Serial.println(xoffset);
  Serial.print("Y axis offset: "); Serial.println(yoffset);
  Serial.print("Z axis offset: "); Serial.println(zoffset);
  Serial.println("Done calibrating!");
  delay(500);
  Serial.println("Gyroscope in degrees");
  if(console){ Serial.println("Roll\tPitch\tYaw\tRange");}
  else{
    Serial.println("CLEARSHEET");
    Serial.println("LABEL, Roll, Pitch, Yaw, Range");
  }
  tic = millis();
}


void loop() {
  imuRead();
  start_sensor(sonar);
  delay(d);
  range = read_sensor(sonar);
  //range = cos(pitch*PI/180)*read_sensor(sonar);
  x = range*cos(yaw*PI/180);
  y = range*sin(yaw*PI/180);
  if(console){
    Serial.print(roll); Serial.print("\t");
    Serial.print(pitch); Serial.print("\t");
    Serial.print(yaw); Serial.print("\t");
    Serial.println(range);
  }
  else{ // Output to PLX-DAQ
    Serial.println( (String) "DATA," + roll + "," + pitch + "," + yaw + "," + range);
  }
  toc = millis();
  dt = 0.001*(toc - tic);
  tic = toc;
  roll+= (((float)GyY / 131) - yoffset)*dt;
  pitch+= (((float)GyX / 131) - xoffset)*dt;
  yaw+= (((float)GyZ / 131) - zoffset)*dt;
} 
