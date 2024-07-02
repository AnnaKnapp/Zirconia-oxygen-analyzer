// the ADC communicates using SPI, so include the library:
#include <SPI.h>
#include "Adafruit_AD569x.h"

#include <SPI.h>
#include "wiring_private.h" // pinPeripheral() function
//mosi on 16, miso on 7, sck on 17  

Adafruit_AD569x ad5693;
SPIClass mySPI (&sercom1, 7, 17, 16, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

const byte RREG = 0x20;     // ADS1262/3 read reg command
const byte WREG = 0x40;   // ADS1262/3 write reg command
const byte POWER_ADDRESS = 0x01;

const byte readcommand = 0x12;

// pins used for the connection with the sensor
// the other you need are controlled by the SPI library):
const int dataReadyPin = 3;
const int startpin = 4;
const int pwdnpin = 5;

const int dataReadyPin_2 = 26;
const int startpin_2 = 28;
const int pwdnpin_2 = 30;

byte dataToSend[8];

int i = 0;

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMicros = 0;        // will store last time LED was updated

// constants won't change:
const long interval = 2000;
const int dac =  A0;
const PROGMEM unsigned int sine[] = {0x7F58,0x80EF,0x8286,0x841D,0x85B4,0x874A,0x88E1,
0x8A76,0x8C0C,0x8DA1,0x8F35,0x90C8,0x925B,0x93ED,
0x957F,0x970F,0x989F,0x9A2D,0x9BBA,0x9D47,0x9ED2,
0xA05B,0xA1E4,0xA36B,0xA4F0,0xA674,0xA7F7,0xA977,
0xAAF6,0xAC74,0xADEF,0xAF69,0xB0E1,0xB256,0xB3CA,
0xB53B,0xB6AB,0xB818,0xB982,0xBAEB,0xBC51,0xBDB4,
0xBF15,0xC074,0xC1D0,0xC329,0xC47F,0xC5D3,0xC724,
0xC871,0xC9BC,0xCB04,0xCC49,0xCD8B,0xCEC9,0xD005,
0xD13D,0xD271,0xD3A3,0xD4D1,0xD5FB,0xD722,0xD846,
0xD966,0xDA82,0xDB9B,0xDCAF,0xDDC0,0xDECE,0xDFD7,
0xE0DD,0xE1DE,0xE2DC,0xE3D5,0xE4CB,0xE5BC,0xE6A9,
0xE792,0xE877,0xE958,0xEA34,0xEB0C,0xEBE0,0xECAF,
0xED7A,0xEE40,0xEF02,0xEFC0,0xF078,0xF12D,0xF1DC,
0xF287,0xF32E,0xF3D0,0xF46D,0xF505,0xF598,0xF627,
0xF6B1,0xF736,0xF7B6,0xF832,0xF8A8,0xF91A,0xF986,
0xF9EE,0xFA51,0xFAAF,0xFB07,0xFB5B,0xFBAA,0xFBF4,
0xFC39,0xFC78,0xFCB3,0xFCE9,0xFD19,0xFD44,0xFD6B,
0xFD8C,0xFDA8,0xFDBF,0xFDD1,0xFDDE,0xFDE5,0xFDE8,
0xFDE5,0xFDDE,0xFDD1,0xFDBF,0xFDA8,0xFD8C,0xFD6B,
0xFD44,0xFD19,0xFCE9,0xFCB3,0xFC78,0xFC39,0xFBF4,
0xFBAA,0xFB5B,0xFB07,0xFAAF,0xFA51,0xF9EE,0xF986,
0xF91A,0xF8A8,0xF832,0xF7B6,0xF736,0xF6B1,0xF627,
0xF598,0xF505,0xF46D,0xF3D0,0xF32E,0xF287,0xF1DC,
0xF12D,0xF078,0xEFC0,0xEF02,0xEE40,0xED7A,0xECAF,
0xEBE0,0xEB0C,0xEA34,0xE958,0xE877,0xE792,0xE6A9,
0xE5BC,0xE4CB,0xE3D5,0xE2DC,0xE1DE,0xE0DD,0xDFD7,
0xDECE,0xDDC0,0xDCAF,0xDB9B,0xDA82,0xD966,0xD846,
0xD722,0xD5FB,0xD4D1,0xD3A3,0xD271,0xD13D,0xD005,
0xCEC9,0xCD8B,0xCC49,0xCB04,0xC9BC,0xC871,0xC724,
0xC5D3,0xC47F,0xC329,0xC1D0,0xC074,0xBF15,0xBDB4,
0xBC51,0xBAEB,0xB982,0xB818,0xB6AB,0xB53B,0xB3CA,
0xB256,0xB0E1,0xAF69,0xADEF,0xAC74,0xAAF6,0xA977,
0xA7F7,0xA674,0xA4F0,0xA36B,0xA1E4,0xA05B,0x9ED2,
0x9D47,0x9BBA,0x9A2D,0x989F,0x970F,0x957F,0x93ED,
0x925B,0x90C8,0x8F35,0x8DA1,0x8C0C,0x8A76,0x88E1,
0x874A,0x85B4,0x841D,0x8286,0x80EF,0x7F58,0x7DC1,
0x7C2A,0x7A93,0x78FC,0x7766,0x75CF,0x743A,0x72A4,
0x710F,0x6F7B,0x6DE8,0x6C55,0x6AC3,0x6931,0x67A1,
0x6611,0x6483,0x62F6,0x6169,0x5FDE,0x5E55,0x5CCC,
0x5B45,0x59C0,0x583C,0x56B9,0x5539,0x53BA,0x523C,
0x50C1,0x4F47,0x4DCF,0x4C5A,0x4AE6,0x4975,0x4805,
0x4698,0x452E,0x43C5,0x425F,0x40FC,0x3F9B,0x3E3C,
0x3CE0,0x3B87,0x3A31,0x38DD,0x378C,0x363F,0x34F4,
0x33AC,0x3267,0x3125,0x2FE7,0x2EAB,0x2D73,0x2C3F,
0x2B0D,0x29DF,0x28B5,0x278E,0x266A,0x254A,0x242E,
0x2315,0x2201,0x20F0,0x1FE2,0x1ED9,0x1DD3,0x1CD2,
0x1BD4,0x1ADB,0x19E5,0x18F4,0x1807,0x171E,0x1639,
0x1558,0x147C,0x13A4,0x12D0,0x1201,0x1136,0x1070,
0x0FAE,0x0EF0,0x0E38,0x0D83,0x0CD4,0x0C29,0x0B82,
0x0AE0,0x0A43,0x09AB,0x0918,0x0889,0x07FF,0x077A,
0x06FA,0x067E,0x0608,0x0596,0x052A,0x04C2,0x045F,
0x0401,0x03A9,0x0355,0x0306,0x02BC,0x0277,0x0238,
0x01FD,0x01C7,0x0197,0x016C,0x0145,0x0124,0x0108,
0x00F1,0x00DF,0x00D2,0x00CB,0x00C8,0x00CB,0x00D2,
0x00DF,0x00F1,0x0108,0x0124,0x0145,0x016C,0x0197,
0x01C7,0x01FD,0x0238,0x0277,0x02BC,0x0306,0x0355,
0x03A9,0x0401,0x045F,0x04C2,0x052A,0x0596,0x0608,
0x067E,0x06FA,0x077A,0x07FF,0x0889,0x0918,0x09AB,
0x0A43,0x0AE0,0x0B82,0x0C29,0x0CD4,0x0D83,0x0E38,
0x0EF0,0x0FAE,0x1070,0x1136,0x1201,0x12D0,0x13A4,
0x147C,0x1558,0x1639,0x171E,0x1807,0x18F4,0x19E5,
0x1ADB,0x1BD4,0x1CD2,0x1DD3,0x1ED9,0x1FE2,0x20F0,
0x2201,0x2315,0x242E,0x254A,0x266A,0x278E,0x28B5,
0x29DF,0x2B0D,0x2C3F,0x2D73,0x2EAB,0x2FE7,0x3125,
0x3267,0x33AC,0x34F4,0x363F,0x378C,0x38DD,0x3A31,
0x3B87,0x3CE0,0x3E3C,0x3F9B,0x40FC,0x425F,0x43C5,
0x452E,0x4698,0x4805,0x4975,0x4AE6,0x4C5A,0x4DCF,
0x4F47,0x50C1,0x523C,0x53BA,0x5539,0x56B9,0x583C,
0x59C0,0x5B45,0x5CCC,0x5E55,0x5FDE,0x6169,0x62F6,
0x6483,0x6611,0x67A1,0x6931,0x6AC3,0x6C55,0x6DE8,
0x6F7B,0x710F,0x72A4,0x743A,0x75CF,0x7766,0x78FC,
0x7A93,0x7C2A,0x7DC1};


void setup() {
  Serial.begin(1000000);

  // start the SPI library:
  SPI.begin();
  SPI.setDataMode(SPI_MODE1);
  SPI.setClockDivider(SPI_CLOCK_DIV32);

  mySPI.begin();
  pinPeripheral(7, PIO_SERCOM);
  pinPeripheral(16, PIO_SERCOM);
  pinPeripheral(17, PIO_SERCOM);

  mySPI.setDataMode(SPI_MODE1);
  mySPI.setClockDivider(SPI_CLOCK_DIV32);


  if (ad5693.begin(0x4C, &Wire)) { // If A0 jumper is set high, use 0x4E
    Serial.println("AD5693 initialization successful!");
  } else {
    Serial.println("Failed to initialize AD5693. Please check your connections.");
    while (1) delay(10); // Halt
  }

  ad5693.reset();
  
  if (ad5693.setMode(NORMAL_MODE, true, false)) {
    Serial.println("AD5693 configured");
  } else {
    Serial.println("Failed to configure AD5693.");
    while (1) delay(10); // Halt
  }

  // initalize the  data ready and chip select pins:
  pinMode(dataReadyPin, INPUT_PULLUP);
  pinMode(startpin, OUTPUT);
  pinMode(pwdnpin, OUTPUT);

  pinMode(dataReadyPin_2, INPUT_PULLUP);
  pinMode(startpin_2, OUTPUT);
  pinMode(pwdnpin_2, OUTPUT);

  delay(3000);

  digitalWrite(pwdnpin, LOW);
  digitalWrite(pwdnpin_2, LOW);
  delay(500);
  digitalWrite(pwdnpin, HIGH);
  digitalWrite(pwdnpin_2, HIGH);
  digitalWrite(startpin, LOW);
  digitalWrite(startpin_2, LOW);
  delay(1000);
  Serial.println("writing");
               //pwr - vbias, int - chksum, mode0, mode1 - filt, mode2 -DR+PGA, inmux
  writeAdcFirst6Regs(0b00000011, 0b00000101, 0x00, 0b01100000, 0b00111100, 0b00001010);
               //pwr - vbias, int - chksum, mode0, mode1 - filt, mode2 -DR+PGA, inmux
  writeAdc_2_First6Regs(0b00000011, 0b00000101, 0x00, 0b01100000, 0b00001100, 0b00001010);
  delay(100);
  Serial.println("reading");
  readAdcFirst6Regs();
  readAdc_2_First6Regs();
  delay(100);
  digitalWrite(startpin, HIGH);
  digitalWrite(startpin_2, HIGH);
  delay(1000);
  
}


void loop() {
  unsigned long currentMicros = micros();
  if (currentMicros - previousMicros >= interval) {
    // save the last time
    previousMicros = currentMicros;
    
    // if the LED is off turn it on and vice-versa:
    //analogWrite(dac, sine[i]);
    ad5693.writeUpdateDAC(sine[i]);
    readBothADCsByCmdChk(i);
    i= i+1;
    if (i>499) {
      i=0;
    }
  }


}

void readADC1ByCmdChk(int index) {

  byte readcommand = 0x12;
  byte adcData[7] = {readcommand, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  SPI.transfer(adcData, 7); 
  //Serial.println(adcData[1]);
  //Serial.println(adcData[2]);
  //Serial.println(adcData[3]);
  //Serial.println(adcData[4]);
  //Serial.println(adcData[5]);
  //Serial.println(adcData[6]);
  //Serial.println((adcData[2]+adcData[3]+adcData[4]+adcData[5]+0x9B)&255);
  if (adcData[6] != ((adcData[2]+adcData[3]+adcData[4]+adcData[5]+0x9B)&255)) {
    Serial.println("checksum failed");
    } else {
      byte bigbyte = index >> 8;
      byte lilbyte = index;
      byte tosend[6] = {adcData[2], adcData[2], adcData[2], adcData[2], bigbyte, lilbyte};
      //long rawdata = (adcData[2])<<24 | int(adcData[3])<<16 | int(adcData[4])<<8 | int(adcData[5]);
      //Serial.println(rawdata);
      //float convFactor = 2.5/2147483648;
      //float convData = rawdata * convFactor;
      //byte * convBytes = (byte *) &convData;
      Serial.write(tosend, 6);
      //byte * idx = (byte *) &index;
      //Serial.write(idx, 2);
      //Serial.println(convData);
      }
  }


void readBothADCsByCmdChk(int index) {
  byte adc1Data[7] = {readcommand, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  SPI.transfer(adc1Data, 7);
  byte adc2Data[7] = {readcommand, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  mySPI.transfer(adc2Data, 7);
  //Serial.println(adcData[1]);
  //Serial.println(adcData[2]);
  //Serial.println(adcData[3]);
  //Serial.println(adcData[4]);
  //Serial.println(adcData[5]);
  //Serial.println(adcData[6]);
  //Serial.println((adcData[2]+adcData[3]+adcData[4]+adcData[5]+0x9B)&255);
  if (adc1Data[6] != ((adc1Data[2]+adc1Data[3]+adc1Data[4]+adc1Data[5]+0x9B)&255) & adc2Data[6] != ((adc2Data[2]+adc2Data[3]+adc2Data[4]+adc2Data[5]+0x9B)&255)) {
    Serial.println("checksum failed");
    } else {
      byte bigbyte = index >> 8;
      byte lilbyte = index;
      byte tosend[10] = {adc1Data[2], adc1Data[3], adc1Data[4], adc1Data[5], adc2Data[2], adc2Data[3], adc2Data[4], adc2Data[5], bigbyte, lilbyte};
      //long rawdata = (adcData[2])<<24 | int(adcData[3])<<16 | int(adcData[4])<<8 | int(adcData[5]);
      //Serial.println(rawdata);
      //float convFactor = 2.5/2147483648;
      //float convData = rawdata * convFactor;
      //byte * convBytes = (byte *) &convData;
      Serial.write(tosend, 10);
      //byte * idx = (byte *) &index;
      //Serial.write(idx, 2);
      //Serial.println(convData);
      }
  }


void writeAdc_2_First6Regs(byte power, byte interface, byte mode0, byte mode1, byte mode2, byte inpmux) {

  byte opcode = WREG | POWER_ADDRESS;
  byte numberToSend = 0x05;
  byte dataToSend[8] = {opcode, numberToSend, power, interface, mode0, mode1, mode2, inpmux};

  mySPI.transfer(dataToSend, 8); //Send register location
}

void writeAdcFirst6Regs(byte power, byte interface, byte mode0, byte mode1, byte mode2, byte inpmux) {

  byte opcode = WREG | POWER_ADDRESS;
  byte numberToSend = 0x05;
  byte dataToSend[8] = {opcode, numberToSend, power, interface, mode0, mode1, mode2, inpmux};

  SPI.transfer(dataToSend, 8); //Send register location
}


void readAdc_2_First6Regs() {

  byte opcode = RREG | POWER_ADDRESS;
  byte numberToSend = 0x05;
  byte dataToSend[8] = {opcode, numberToSend, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  mySPI.transfer(dataToSend, 8); //Send register location
  Serial.println(dataToSend[2]);
  Serial.println(dataToSend[3]);
  Serial.println(dataToSend[4]);
  Serial.println(dataToSend[5]);
  Serial.println(dataToSend[6]);
  Serial.println(dataToSend[7]);
}

void readAdcFirst6Regs() {

  byte opcode = RREG | POWER_ADDRESS;
  byte numberToSend = 0x05;
  byte dataToSend[8] = {opcode, numberToSend, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

  SPI.transfer(dataToSend, 8); //Send register location
  Serial.println(dataToSend[2]);
  Serial.println(dataToSend[3]);
  Serial.println(dataToSend[4]);
  Serial.println(dataToSend[5]);
  Serial.println(dataToSend[6]);
  Serial.println(dataToSend[7]);
}
