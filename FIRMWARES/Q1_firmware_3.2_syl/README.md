# QBoard firmwares

# Install IDE Arduino
sudo apt-get update
sudo apt-get install ubuntu-make
umake ide arduino
sudo usermod -a -G dialout $USER
sudo usermod -a -G tty $USER


- Version 0 is the original version by The Corpora updated so that it could be compiled on an Arduino IDE with version higher than 1.0.0
- Version 1 is the same version but replacing I2C not custom with the standard Wire library
- Version 2 and above are a big evolution of the base :
- the SRFS are replaced by single point LIDAR VL53L1X
- the IMU is replaced by the classic MPU6050 less expensive and more precise
- By the way, the code has been cleaned and simplified

Arduino IDE 1.8.9

Libraries to install on Arduino IDE 1.8.9 via "gérer les librairies":

- VL53L1X by Pololu (complete access to the range data and diagnostic)
- MPU6050 by Electronic Cats (complete access to the functionnalities of the chip)
- the LCD library using wire has been rewritten for LCD03 (included when necessary)

type of card : Arduino/Genuino Mega or Mega 2560
chip : atmega1280
programmer : AVRISP mkll
