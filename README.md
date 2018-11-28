# sonar-mapper
This is the version to use just before the Wednesday, Nov 28 lab. The main change since the last iteration is the addition of an automatic 40-ms delay in every sonar reading; this should help with echoes and interference. It also requires that the spin be slower; to help compensate for this, v3 decreases the length of the ring buffer to increase responsiveness.

Uses libraries:

Wire.h
SD.h
MPU6050_tockn.h (available in Arduino Library Manager)
