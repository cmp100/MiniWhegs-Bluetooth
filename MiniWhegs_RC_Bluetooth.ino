//Based on original code for Bluetooth

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h> 
#include <SoftwareSerial.h> 

int bluetoothTx = 2;  // TX-O pin of bluetooth mate, Arduino D2
int bluetoothRx = 3;  // RX-I pin of bluetooth mate, Arduino D3

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *myMotor = AFMS.getMotor(1);

Servo servo1;


byte commands[4] = {0x00,0x00,0x00,0x00};
byte prevCommands[4] = {0x01,0x01,0x01,0x01};
unsigned long timer0 = 2000;  //Stores the time (in millis since execution started) 
unsigned long timer1 = 0;  //Stores the time when the last sensor reading was sent to the phone
unsigned long timer2 = 0;  //Stores the time when the last command was received from the phone
//14 byte payload that stores the sensor readings
//14 byte payload that stores the sensor readings
byte three[14] = {0xee,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xcc};
float stepSize = 9.04;
//The union allows you to declare a customized data type, in this case it can be either 
//a float or a byte array of size 4. What we need is to store a float which is 4
//bytes long and retrieve each of the 4 bytes separately.
union u_sensor0{
  byte a[4];  
  float b;
}sensor0;
union u_sensor1{
  byte c[4];  
  float d;
}sensor1;
int i = 0;

void setup() {
  Serial.begin(115200);           // set up Serial library at 115200 bps
  Serial.println("Setup");

  bluetooth.begin(115200);

  AFMS.begin();  // create with the default frequency 1.6KHz
  //AFMS.begin(1000);  // OR with a different frequency, say 1KHz
  
  // Attach a servo to pin #9
  servo1.attach(9);
   
  // turn on motor M1
  myMotor->setSpeed(200);
  myMotor->run(RELEASE);


  //Serial.begin(9600);  // Begin the serial monitor at 9600bps

  //bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
  bluetooth.print("$");  // Print three times individually
  bluetooth.print("$");
  bluetooth.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  //bluetooth.begin(9600);  // Start bluetooth serial at 9600
 


//int i;
}

void loop() {
  // put your main code here, to run repeatedly:
if(bluetooth.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.println("bluetooth available");
    Serial.print((char)bluetooth.read());  }
    else {
    Serial.println("bluetooth NOT available");
  }
if(Serial.available()==4){ 
  
   Serial.println("serial available");
    timer2 = millis();  //Store the time when last command was received
    memcpy(prevCommands,commands,4);  //Storing the received commands   
    commands[0] = Serial.read();  //Direction
    commands[1] = Serial.read();  //Speed
    commands[2] = Serial.read();  //Angle
    commands[3] = Serial.read();  //Lights and buttons states
    //Serial.println((char)bluetooth.read());
    /*
     
     Since the last byte yields the servo's angle (between 0-180), it can never be 255. At times, the two
     previous commands pick up incorrect values for the speed and angle. Meaning that they get the direction 
     correct 100% of the time but sometimes get 255 for the speed and 255 for the angle.
     */
    if((commands[2]<=0xb4)&&((commands[0]<=0xf5)&&(commands[0]>=0xf1))){
      //Make sure that the command received involves controlling the car's motors (0xf1,0xf2,0xf3)
      if(commands[0] <= 0xf3){
        if(commands[0] == 0xf1){  //Check if the move forward command was received
          if(prevCommands[0] != 0xf1){  //Change pin state to move forward only if previous state was not move forward
            myMotor->run(FORWARD);
            //myMotor->setSpeed(500);
            Serial.println("Updated direction FWD");
          }  
        }
        else if(commands[0] == 0xf2){  //Check if the move back command was received     
          if(prevCommands[0] != 0xf2){  //Change pin state to move back only if previous state was not move back
            myMotor->run(BACKWARD);
            Serial.println("Updated direction BAK");
          }
        }
        else{  //Check if the stop command was received    
          if(prevCommands[0] != 0xf3){  //Change pin state to stop only if previous state was not stop
            myMotor->run(RELEASE)   ;
            Serial.println("Updated direction STP");
          ;}
        }
        //Change speed only if new speed is not equal to the previous speed
        if(prevCommands[1] != commands[1]){
          myMotor->setSpeed(commands[1]);  
          Serial.println("Updated speed");
        }          
        //Steer front wheels only if the new angle is not equal to the previous angle
        if(prevCommands[2] != commands[2]){
          servo1.write(commands[2]);  
          //Serial.println("Updated angle"); 
        }         
      }
      else if(commands[0] == 0xf5){
        if(prevCommands[0] != 0xf5){
          //Stop everything
          myMotor->run(RELEASE)   ;
          //AFMS.stopped_1W(); 
}
      }
      else{
        //Here you put the code that will control the tilt pan (commands[0] == 0xf4)   
      } 
    }
    else{
      //Resetting the Serial port (clearing the buffer) in case the bytes are not being read in correct order.
      Serial.end();
      Serial.begin(115200);
    }
  }
  else{
    Serial.println("serial NOT available");
    timer0 = millis();  //Get the current time (millis since execution started)
    if((timer0 - timer2)>400){  //Check if it has been 400ms since we received last command
      //More tan 400ms have passed since last command received, car is out of range. Therefore
      //Stop the car and turn lights off
      myMotor->run(RELEASE)   ;  
      //digitalWrite(pinFrontLights,LOW); 
      //digitalWrite(pinBackLights,LOW);
    }
    if((timer0 - timer1)>=477){  //Check if it has been 477ms since sensor reading were sent
      //Calculate the 9V's voltage by multiplying the step size by the step number (analogRead(0)) 
      //This value will be in mV, which is why it's multiplied by 0.001 to convert into Volts.
      sensor0.b = (analogRead(0) * stepSize) * 0.001;  
      //Break the sensor0 float into four bytes for transmission
      three[1] = sensor0.a[0];
      three[2] = sensor0.a[1];
      three[3] = sensor0.a[2];
      three[4] = sensor0.a[3];
      //Get sensor 2's reading
      sensor1.d = analogRead(1);  
      //Break the sensor1 float into four bytes for transmission
      three[5] = sensor1.c[0];
      three[6] = sensor1.c[1];
      three[7] = sensor1.c[2];
      three[8] = sensor1.c[3];
      //Get the remaining reading from the analog inputs
      three[9] = map(analogRead(2),0,1023,0,255);
      three[10] = map(analogRead(3),0,1023,0,255);
      three[11] = map(analogRead(4),0,1023,0,255);
      three[12] = map(analogRead(5),0,1023,0,255); 
      //Send the six sensor readings
      Serial.write(three,14);
      //Store the time when the sensor readings were sent
      timer1 = millis();
    }
    }
}
