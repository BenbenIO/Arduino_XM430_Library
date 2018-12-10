#include "arduino.h"
#include "XM430.h"

int XM430::ReadError(void)
{
  int TimeCounter = 0;
  unsigned char IncomingByte;
  int ErrorByte=-1;
  digitalWrite(RS485_EN,LOW);

  while((toRS485->available() < 5) & (TimeCounter < 10))
  {
    TimeCounter++;
    delay(1000);
    Serial.println("waiting data...");
  }

  while (toRS485->available() > 0)
  {
    IncomingByte = toRS485->read();
    if ((IncomingByte == 255) & toRS485->peek() == 255 )
      {
        toRS485->read();
        toRS485->read();
        toRS485->read();
        ErrorByte = toRS485->read();
        return (ErrorByte);
     }
  }
  return (-1);
}

XM430::XM430(SoftwareSerial * ss)
{
  toRS485 = ss;
  pinMode(RS485_EN, OUTPUT);
  Serial.println("motor initialized");
}

void XM430::BeginRS485(uint32_t baudrate)
{
  toRS485->begin(baudrate);
  Serial.println("RS485 serial started");
}

void XM430::LedWrite(byte servoID, byte newValue)
{
  unsigned char instruction = 0x03;
  byte ledAddress = 0x41;
  unsigned char length = 0x04;
  int checksum_ACK;
  byte notchecksum;
  checksum_ACK =  servoID + length + instruction + ledAddress + newValue;
  notchecksum = ~checksum_ACK;

  digitalWrite(RS485_EN,HIGH);
  delay(15);
  toRS485->write(0xFF);
  toRS485->write(0xFF);
  toRS485->write(servoID);
  toRS485->write(length);
  toRS485->write(instruction);
  toRS485->write(ledAddress);
  toRS485->write(newValue);
  toRS485->write(notchecksum);
  delay(15);
  digitalWrite(RS485_EN,LOW);
}

void XM430::TorqueEnable(byte servoID, byte newValue)
{
  unsigned char instruction = 0x03;
  byte torqueAddress = 0x40;
  unsigned char length = 0x04;
  int checksum_ACK;
  byte notchecksum;
  checksum_ACK =  servoID + length + instruction + torqueAddress + newValue;
  notchecksum = (~checksum_ACK);

  digitalWrite(RS485_EN,HIGH);
  delay(15);
  toRS485->write(0xFF);
  toRS485->write(0xFF);
  toRS485->write(servoID);
  toRS485->write(length);
  toRS485->write(instruction);
  toRS485->write(torqueAddress);
  toRS485->write(newValue);
  toRS485->write(notchecksum);
  delay(15);
  digitalWrite(RS485_EN,LOW);
  //readerror:
  //Serial.print("ReadError:");
  //Serial.println(ReadError());
}

void XM430::int32Splitting(uint32_t Position, unsigned char bytes[4])
{
  bytes[0] = (Position & 0xFF000000) >> 24;
  bytes[1] = (Position & 0x00FF0000) >> 16;
  bytes[2] = (Position & 0x0000FF00) >> 8;
  bytes[3] = (Position & 0x000000FF);
}

void XM430::Goto(byte servoID, int position)
{
  unsigned char instruction = 0x03;
  byte goalPositionAddress = 0x74;
  unsigned char length = 0x07;
  unsigned char positionbytes[4];
  int32Splitting(position, positionbytes);
  int checksum_ACK;
  byte notchecksum;
  checksum_ACK =  servoID + length + instruction + goalPositionAddress + positionbytes[3] + positionbytes[2] + positionbytes[1] + positionbytes[0];
  notchecksum = (~checksum_ACK);

  digitalWrite(RS485_EN,HIGH);
  delay(15);
  toRS485->write(0xFF);
  toRS485->write(0xFF);
  toRS485->write(servoID);
  toRS485->write(length);
  toRS485->write(instruction);
  toRS485->write(goalPositionAddress);
  toRS485->write(positionbytes[3]);
  toRS485->write(positionbytes[2]);
  toRS485->write(positionbytes[1]);
  toRS485->write(positionbytes[0]);
  toRS485->write(notchecksum);
  delay(15);
  digitalWrite(RS485_EN,LOW);

}

void XM430::SyncWrite(byte servoID1, int position1, byte servoID2, int position2)
{
  //Write the goal position of two motors in only one packet
  unsigned char instruction = 0x83; //sync write
  byte goalPositionAddress = 0x74;
  unsigned char length = 0x04;   // of data to write
  unsigned char LENGTH = 0xE;  //the full packet
  unsigned char position1bytes[4];
  unsigned char position2bytes[4];
  int32Splitting(position1, position1bytes);
  int32Splitting(position2, position2bytes);
  int checksum_ACK;
  byte notchecksum;
  checksum_ACK =  BROADCAST_ID + LENGTH + instruction + goalPositionAddress + length + servoID1 + position1bytes[3] + position1bytes[2] + position1bytes[1] + position1bytes[0] + servoID2  + position2bytes[3] + position2bytes[2] + position2bytes[1] + position2bytes[0];
  notchecksum = (~checksum_ACK);

  digitalWrite(RS485_EN,HIGH);
  delay(15);
  toRS485->write(0xFF);
  toRS485->write(0xFF);
  toRS485->write(0xFE);
  toRS485->write(LENGTH);
  toRS485->write(instruction);
  toRS485->write(goalPositionAddress);
  toRS485->write(length);
  toRS485->write(servoID1);
  toRS485->write(position1bytes[3]);
  toRS485->write(position1bytes[2]);
  toRS485->write(position1bytes[1]);
  toRS485->write(position1bytes[0]);
  toRS485->write(servoID2);
  toRS485->write(position2bytes[3]);
  toRS485->write(position2bytes[2]);
  toRS485->write(position2bytes[1]);
  toRS485->write(position2bytes[0]);
  toRS485->write(notchecksum);
  delay(15);
  digitalWrite(RS485_EN,LOW);
  Serial.println("Send packet ?");

}

void XM430::SetP(byte servoID, uint16_t P)
{
  unsigned char instruction = 0x03;
  byte Paddress = 0x54;
  unsigned char pbytes[2];
  pbytes[0] = P >> 8;
  pbytes[1] = P;

  unsigned char length = 0x05;
  int checksum_ACK;
  byte notchecksum;
  checksum_ACK =  servoID + length + instruction + Paddress + pbytes[0] + pbytes[1];
  notchecksum = ~checksum_ACK;

  digitalWrite(RS485_EN,HIGH);
  delay(15);
  toRS485->write(0xFF);
  toRS485->write(0xFF);
  toRS485->write(servoID);
  toRS485->write(length);
  toRS485->write(instruction);
  toRS485->write(Paddress);
  toRS485->write(pbytes[0]);
  toRS485->write(pbytes[1]);
  toRS485->write(notchecksum);
  delay(15);
  digitalWrite(RS485_EN,LOW);

}

void XM430::ReadP(byte servoID)
{
  unsigned char instruction = 0x02;  //read
  byte Paddress = 0x54;       //P address (84)
  unsigned char length = 0x04; //Length = number of Parameters + 2
  int checksum_ACK;
  byte notchecksum;
  byte toread = 0x02;
  checksum_ACK =  servoID + length + instruction + Paddress + toread;
  notchecksum = ~checksum_ACK;

  // Packet Generation:
  digitalWrite(RS485_EN,HIGH);
  delay(15);
  toRS485->write(0xFF);
  toRS485->write(0xFF);
  toRS485->write(servoID);
  toRS485->write(length);
  toRS485->write(instruction);
  toRS485->write(Paddress);
  toRS485->write(toread);
  toRS485->write(notchecksum);
  delay(15);
  digitalWrite(RS485_EN,LOW);

  //Try to get the result:
  unsigned char Incoming_Byte;
  unsigned char P_Low_Byte;
  unsigned char P_High_Byte;
  unsigned char Error_Byte;
  int P_Long_Byte = -1;
  int Time_Counter = 0;
  digitalWrite(RS485_EN,LOW);

  while((toRS485->available() < 8) & (Time_Counter < TIME_OUT))
    {
      Time_Counter++;
      delay(1000);
      Serial.println("waiting for data...");
      Serial.println(toRS485->available());
    }

  while(toRS485->available() > 0)
    {
      Incoming_Byte = toRS485->read();
	  if((Incoming_Byte == 255) & (toRS485->peek() == 255))
      {
        toRS485->read();                            // Second Start bit
        toRS485->read();                            // servoID
        toRS485->read();                            // Length
        if((Error_Byte = toRS485->read()) != 0)
        {
          Serial.println("Error...");
        }
        P_Low_Byte = toRS485->read();
        Serial.print(P_Low_Byte);
        P_High_Byte = toRS485->read();
        Serial.print(P_High_Byte);
        P_Long_Byte = P_High_Byte << 8;
        P_Long_Byte = P_Long_Byte + P_Low_Byte;
	  }
    }
  Serial.print("Read Value for P :");
  Serial.println(P_Long_Byte);
}

void XM430::SetI(byte servoID, uint16_t I)
{
  unsigned char instruction = 0x03;
  byte Iaddress = 0x52;       //I address (82)
  unsigned char ibytes[2];
  ibytes[0] = I >> 8;
  ibytes[1] = I;

  unsigned char length = 0x05;
  int checksum_ACK;
  byte notchecksum;
  checksum_ACK =  servoID + length + instruction + Iaddress + ibytes[0] + ibytes[1];
  notchecksum = ~checksum_ACK;

  digitalWrite(RS485_EN,HIGH);
  delay(15);
  toRS485->write(0xFF);
  toRS485->write(0xFF);
  toRS485->write(servoID);
  toRS485->write(length);
  toRS485->write(instruction);
  toRS485->write(Iaddress);
  toRS485->write(ibytes[0]);
  toRS485->write(ibytes[1]);
  toRS485->write(notchecksum);
  delay(15);
  digitalWrite(RS485_EN,LOW);
}

void XM430::SetD(byte servoID, uint16_t D)
{
  unsigned char instruction = 0x03;  //write
  byte Daddress = 0x50;       //D address (80)
  unsigned char dbytes[2];
  dbytes[0] = D >> 8;
  dbytes[1] = D;
  unsigned char length = 0x05;
  int checksum_ACK;
  byte notchecksum;
  checksum_ACK =  servoID + length + instruction + Daddress + dbytes[0] + dbytes[1];
  notchecksum = ~checksum_ACK;

  // Packet Generation:
  digitalWrite(RS485_EN,HIGH);
  delay(15);
  toRS485->write(0xFF);
  toRS485->write(0xFF);
  toRS485->write(servoID);
  toRS485->write(length);
  toRS485->write(instruction);
  toRS485->write(Daddress);
  toRS485->write(dbytes[0]);
  toRS485->write(dbytes[1]);
  toRS485->write(notchecksum);
  delay(15);
  digitalWrite(RS485_EN,LOW);
}

void XM430::SetPID(byte servoID, uint16_t P, uint16_t I, uint16_t D)
{
  Serial.print("Setting PID for motor: ");
  Serial.println(servoID, DEC);
  SetP(servoID, P);
  SetI(servoID, I);
  SetD(servoID, D);
}
