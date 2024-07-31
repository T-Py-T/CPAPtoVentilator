#include <Wire.h>

#define SENSIRION_ADDR 0x40  // address 64

/*  read flow  for Sensirion 3200
    init continuous flow reading for Sensirion 3200
    https://14core.com/wp-content/uploads/2017/12/Sensirion-Mass-Flow-Meters-SFM3000-I2C-Functional-Description.pdf
*/

void init_flow_sensor(){
  // init continuous flow reading for Sensirion 3200
  // https://14core.com/wp-content/uploads/2017/12/Sensirion-Mass-Flow-Meters-SFM3000-I2C-Functional-Description.pdf
  Wire.beginTransmission(SENSIRION_ADDR); // transmit to device
  Wire.write(0x10);        // request reading
  Wire.write(0x00);        // request serial
  Wire.endTransmission();    // stop transmitting
}

int read_flow(){  
  int math = 0;
  // read flow  for Sensirion 3200
  // https://14core.com/wp-content/uploads/2017/12/Sensirion-Mass-Flow-Meters-SFM3000-I2C-Functional-Description.pdf
  Wire.requestFrom(SENSIRION_ADDR, 3); // (Arg1 Address, Arg2 # of Bytes)
  byte msb = 0;
  byte lsb = 0;
  byte crc = 0;
  while(Wire.available()){
    msb = Wire.read();
    lsb = Wire.read();
    crc = Wire.read();  //TODO use CRC
  }
  math = (msb<<8) + lsb; //msb*2^8) + lsb
  math -= 32768; // offset value 
  math /= 115; // Rate
  return abs(math);
}
