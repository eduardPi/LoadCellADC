#include <Arduino.h>
#include <WiFi.h>
#include "FS.h"
#include <SPI.h>
// ADS1242 24bit ADC works

// Pin Definitions
const int DRDY_PIN = 19;      // Data Ready / Data Out pin
const int PWR_DOWN_PIN = 13;  // Power-Down pin


// Global variables
volatile bool dataReady = false;  // Data Ready flag
byte adcData[3];
int adcValue=0;
int Wtare=0;
// 69, 
float Windex=1;
float Weight=0;
String serial_read;



  int LC_DataRead()
  {
      int DataResult=0;
digitalWrite(PWR_DOWN_PIN, LOW);
SPI.transfer(0x01);
  for (byte i=0; i<3; i++)
  {
    adcData[i]=SPI.transfer(0x00);
  }
  digitalWrite(PWR_DOWN_PIN, HIGH);
  DataResult=adcData[0]<<16;
  DataResult=adcData[1]<<8;
  DataResult=adcData[2];
  Serial.print("Read data DEC: ");
  Serial.println(DataResult);
  Serial.print("Read data HEX: ");
  Serial.println(DataResult,HEX);
  return DataResult;
  }
  void Selfcal()
  {
    digitalWrite(PWR_DOWN_PIN, LOW);
    SPI.transfer(0xF0);
    digitalWrite(PWR_DOWN_PIN, HIGH);
  }

void readADCData() {
  // Read 24 bits of data
  
  digitalWrite(PWR_DOWN_PIN, LOW);
  SPI.transfer(0x01);
   delay(0.1);
  for (int i = 0; i < 3; i++) {
    adcData[i] = SPI.transfer(0x00);
  }
   digitalWrite(PWR_DOWN_PIN, HIGH);
    adcValue = (adcData[0] << 16) | (adcData[1] << 8) | adcData[2];
    adcValue = adcValue-Wtare;
    // Serial.println(adcValue);
    Weight=adcValue/Windex;
    //  Serial.println(Weight);
    // Serial.print("ADC Value: ");
    // Serial.print("Data in HEX: ");
    // Serial.println(adcValue, HEX);
    // Serial.println(String("Data in DEC: ") +  adcValue);
    Serial.println(String("Weight: ") +  Weight + String(" gr"));
    
  
}


void WriteReg(byte RegIndex, byte command)
{
byte WriteCommand=0b01010000;
WriteCommand= WriteCommand | RegIndex;
byte Regcount=0x0;
Serial.println(WriteCommand,BIN);
digitalWrite(PWR_DOWN_PIN, LOW);
SPI.transfer(WriteCommand);
SPI.transfer(Regcount);
SPI.transfer(command);
digitalWrite(PWR_DOWN_PIN, HIGH);


}

void ReadReg(byte RegIndex)
{
byte RegReadqty=0;
byte Regread[2]; 
byte ReadCom=0x10;
byte Command=ReadCom | RegIndex;
digitalWrite(PWR_DOWN_PIN, LOW);
SPI.transfer(Command);
SPI.transfer(RegReadqty);
 delay(0.1);
  for (int i = 0; i < 1; i++) {
    adcData[i] = SPI.transfer(0x00);
  }
  digitalWrite(PWR_DOWN_PIN, HIGH);
   adcValue = (adcData[0] << 8) | adcData[1] ;
    // adcValue = (adcData[0] << 8) | adcData[1] ;


    Serial.print("Data in HEX: ");
    Serial.println(adcValue, HEX);
    Serial.println(String("Data in DEC: ") +  adcValue);
    // return adcValue;

}


void TareInit(){
//int TareCells[5];
Wtare=0;
for (byte i; i<5; i++){
 readADCData();
 Wtare=adcValue;
// Wtare+=adcValue;
//  TareCells[i]=adcValue;
}
//Wtare=Wtare/5;
Serial.println(String("Wtare: ") + Wtare);

//Wtare=(adcValue[0]+adcValue[1]+adcValue[2]+adcValue[3]+adcValue[4])/5);
}
void Calibration() {
  Serial.println("Calibration started");
  digitalWrite(PWR_DOWN_PIN, HIGH);
  for (int i = 0; i < 3; i++) {
    SPI.transfer(0x00);
  }
  pinMode(18, OUTPUT);
  for (byte i; i < 2; i++)
  {
    digitalWrite(18, HIGH);
    digitalWrite(18, LOW);
  }
  delay(1000);
  digitalWrite(PWR_DOWN_PIN, LOW);
  SPI.begin();
  Serial.println("Calibration finished");
}




void setup() {
  Serial.begin(115200);

  pinMode(DRDY_PIN, INPUT_PULLUP);
  pinMode(PWR_DOWN_PIN, OUTPUT);


 
  SPI.begin();

  // Set SPI settings for ADS1232

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE1));

  // Ensure the ADS1232 is not in power-down mode initially
  digitalWrite(PWR_DOWN_PIN, LOW);  // Set power-down pin high to activate

  // No need to attach an interrupt to DRDY, as it's also the DOUT pin
  WriteReg(0x00,0x06); // Gain
  // WriteReg(0x02,0x40); 
  WriteReg(0x02,0x44); 
}

void loop() {

// WriteReg(0x00,0x06);
  // ReadReg(0x00);
//  readADCData();
readADCData();
delay(500);
//////////////////////////////////////////////////////////////////////
  if (Serial.available() > 0) {
    delay (100);  // wait to message arrive

    // read all the available characters
    while (Serial.available() > 0) {
      serial_read = Serial.readStringUntil('\n');

    }
    serial_read.toLowerCase();
    if (serial_read == "cal") {

      Calibration();

    } else if (serial_read == "tare") {
      TareInit();
    } else if (serial_read == "selfcal") {
    Selfcal();
    Serial.println("Self calibration complete");
    } else if (serial_read.substring(0, 5) == "wndx$") {
      String Wndx=serial_read.substring(5);
      Windex=Wndx.toInt();
      Serial.println(String ("Windex set to: ") + Wndx );
    }

  }
}