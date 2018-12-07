/***************************************************************
* Arduino XM430 Library - Version 1.0
* by Benjamin IOLLER
* Minimalist API for controlling the XM430 servomotors
* 
* Example Program: turn On-OFF the LED, and move the motors from 0 to 1000 every second.
***************************************************************/
#include <XM430.h>
#include <SoftwareSerial.h>

//Software Serial (RX, TX)
SoftwareSerial toRS485(11, 10);

//motorID
byte servo3 = 0x03;
byte servo1 = 0x01;

XM430 gimbal(&toRS485);

void setup() {
  //Start the Serial communication: One for the Nano communication, and the other for the RS485
  Serial.begin(9600);
  gimbal.BeginRS485(57600);
  //Enable Torque, without this function the motors will not move.
  gimbal.TorqueEnable(servo3, ON);
}

void loop() {
  Serial.println("Led on");
  gimbal.LedWrite(servo3, ON);
  gimbal.Goto(servo3, 1000);
  delay(1000);
  
  //Serial.println("Led off");
  gimbal.LedWrite(servo3, OFF);
  gimbal.Goto(servo3, 0);
  delay(1000);

}
