# MiniWhegs-Bluetooth

This Arduino code is intended to allow a Mini-Whegs robot to be controlled remotely via bluetooth using the android app: 
"Arduino Joystick Controller" by Andi.Co (https://play.google.com/store/apps/details?id=com.andico.control.joystick&hl=en)

The robot features:
1 Arduino Uno
1 Adafruit MotorShield V2
1 Futaba Servo (pin 9)
1 Gearmotor (M1 port)
1 Battery
1 Sparkfun Bluetooth Mate Silver (pins D2 and D3)

The code is based off of the code on this website by the app developer:
https://sites.google.com/site/bluetoothrccar/home/6-Joystick-Control

However, this code utilizes a different motorshield and does not utilize SoftwareSerial library to allow the bluetooth 
to be operated off of pins D2 and D3 instead of D0 and D1 so my code is modified to accomodate for these features.

Information on the bluetooth mate can be found here:
https://learn.sparkfun.com/tutorials/using-the-bluesmirf?_ga=1.75861205.1168728065.1459968985

Information on the MotorShield can be found here:
https://learn.adafruit.com/adafruit-motor-shield-v2-for-arduino/overview

Catherine Passmore
Biologically-Inspired Robotics Lab
Case Western Reserve University
