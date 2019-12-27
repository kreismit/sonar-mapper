# sonar-mapper

This program began as the final project for Cornell University [PHYS 3360](http://courses.cornell.edu/preview_course.php?catoid=36&coid=576564). Its purpose is to generate a map (like a floor plan) of a room using SONAR and a gyroscope.
The PHYS 3360 final project had four components: an Arduino Uno rev2, an MPU6050 6-axis IMU, an HC-SR04 ultrasonic rangefinder (1-D SONAR), and an SD card reader. The device wrote a file to the SD card for each full revolution; the file was a CSV with x- and y-coordinates of its "map" of the room. However, the beam was too wide to capture meaningful data.

The next rebirth of this project was a prototype of the project for Cornell University [MAE 4351, Interdisciplinary Design Concepts](http://courses.cornell.edu/preview_course.php?catoid=36&coid=579669). The basic concept of the prototype was the same, but now the data was output over a serial connection to a screen and the SONAR was replaced by the Maxbotix [MB1242](https://www.maxbotix.com/Ultrasonic_Sensors/MB1242.htm). This is the most current version and thus occupies the _master_ branch.
