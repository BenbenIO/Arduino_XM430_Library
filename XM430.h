/***************************************************************
* Arduino XM430 Library - Version 1.0
* by Benjamin IOLLER
* Minimalist API for controlling the XM430 servomotors
* Currently supported: TorqueEnable / Goto
* Ongoing development: SetPID / readPID values / readError
***************************************************************/

#ifndef XM430_H
#define XM430_H

#include "arduino.h"
#include <SoftwareSerial.h>

#define RS485_EN 2
#define TIME_OUT 10
#define BYTE_PID 2
#define ON 0x01
#define OFF 0x00

class XM430
{
public:
    XM430(SoftwareSerial * ss);
    //Class constructor using the SoftwareSerial pointer
    void BeginRS485(uint32_t baudrate);
    //BeginRS485: start the SoftwareSerial communication
    void LedWrite(byte servoID, byte newValue);
    //LedWrite: write the newValue (ON-OFF) at Led address (65->0x41)
    void TorqueEnable(byte servoID, byte newValue);
    //TorqueEnable: write the newValue(On-OFF) at the torque address (64->0x40)
    void int32Splitting(uint32_t Position, unsigned char bytes[4]);
    //int32Splitting: decompose the uint32_t position into 4 bytes by masking and shiffting
    void Goto(byte servoID, int position);
    //Goto: write the 4 byte position at the goalposition address (116->0x74). The protocol use MSB first.

    //Currently working on:
    void SetP(byte servoID, uint16_t P);
    void SetI(byte servoID, uint16_t I);
    void SetD(byte servoID, uint16_t D);
    void SetPID(byte servoID, uint16_t P, uint16_t I, uint16_t D);
    void ReadP(byte servoID);

private:
    int _currentPosition;
    SoftwareSerial * toRS485;
};

#endif
