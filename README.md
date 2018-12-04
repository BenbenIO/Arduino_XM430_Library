# Arduino_XM430_Library
A minimal arduino library to control the dynamixel XM series servomotors. 
<br/>The library use the protocol 1 to communicate with the motors, a complete description of this protocol is available [HERE](http://emanual.robotis.com/docs/en/dxl/protocol1/), and the control table for the XM430 motors can be found [HERE](http://emanual.robotis.com/docs/en/dxl/x/xm430-w350/)
<br/>
<br/> This motors use the [RS485 serial communication standard](https://en.wikipedia.org/wiki/RS-485) to communicate, thus we need to generate compatible packet and signal. This required an external hardware in order to convert the TTl signal from the arduino to RS485 of the motor. The chip is the MAX485 and can be found for a very affordable price on [ebay or so](https://www.amazon.co.jp/dp/B014MBRC9Y/ref=asc_df_B014MBRC9Y2543006/?tag=jpgo-22&creative=9315&creativeASIN=B014MBRC9Y&linkCode=df0&hvadid=280311208557&hvpos=1o1&hvnetw=g&hvrand=6514494582976950370&hvpone=&hvptwo=&hvqmt=&hvdev=c&hvdvcmdl=&hvlocint=&hvlocphy=1028824&hvtargid=pla-555087944344).
The following image showes the wiring:
<p align="center">
  <img src="/images/MAX485Wiring.png" width="500">
</p>
<br/> The library generate the packet and then send them though the SoftwareSerial so we are still able to communicate and getting feed back from the arduino.</br>

## Ongoing developpment:
* PID setting
* PID reading value
<br/>__If you want to add any function, please feel free to ask or help :)__

# Code example
```c
#include <XM430.h>
#include <SoftwareSerial.h>

//Software Serial (RX, TX)
SoftwareSerial toRS485(10, 11);

//motorID
byte servo3 = 0x03;
byte servo1 = 0x01;

XM430 gimbal(&toRS485);

void setup() {
  //Start the Serial communication
  Serial.begin(9600);
  gimbal.BeginRS485(57600);
}

void loop() {
  Serial.println("Led on");
  gimbal.LedWrite(servo3, ON);
  gimbal.Goto(servo3, 1000);
  delay(1000);
  
  Serial.println("Led off");
  gimbal.LedWrite(servo3, OFF);
  gimbal.Goto(servo3, 0);
  delay(1000);

}
```
# API Description
* __XM430(SoftwareSerial * ss)__
<br/>Class constructor using the SoftwareSerial pointer

* __void BeginRS485(uint32_t baudrate)__
<br/> BeginRS485: start the SoftwareSerial communication

* __void LedWrite(byte servoID, byte newValue)__
<br/>LedWrite: write the newValue (ON-OFF) at Led address (65->0x41)

* __void TorqueEnable(byte newValue)__
<br /> TorqueEnable: write the newValue(On-OFF) at the torque address (64->0x40)

* __void int32Splitting(uint32_t Position, unsigned char bytes[4])__
<br/>int32Splitting: decompose the uint32_t position into 4 bytes by masking and shiffting

* __void Goto(byte servoID, int position)__
 <br/>Goto: write the 4 byte position at the goalposition address (116->0x74). The protocol use MSB first.
