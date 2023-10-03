#include <Arduino.h>
#include <WiFi.h>
#include "FS.h"
#include <SPI.h>
// ADS1232 24bit ADC works

// Pin Definitions
#define IO_exp_nCS 5
#define S2_0 16    //GPIO 16  on board
#define S2_1 4    //GPIO 4    on board
#define S2_2 0    //GPIO 0    on board
#define S2_3 2    //GPIO 2    on board

const int DRDY_PIN = 19;      // Data Ready / Data Out pin
const int PWR_DOWN_PIN = 13;  // Power-Down pin (Matrix board- 22) (MSR2- 13)


// Global variables
volatile bool dataReady = false;  // Data Ready flag
byte adcData[3];
int adcValue=0;
int Wtare=0;
int counter=0;
long meanSum=0;
float Windex=-17.8;
float Weight=0;
float PrevWeight=0;
byte EX_PORT0 = B00110001;
byte EX_PORT1 = B11100000;
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

byte sensor_state_optimized( byte optimized_state)
{
  switch (optimized_state)
  {
    case 0:
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      //      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      //      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;

    case 15:
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, HIGH);

  }
  return optimized_state;
}

    void Selfcal()
  {
    digitalWrite(PWR_DOWN_PIN, LOW);
    SPI.transfer(0xF0);
    digitalWrite(PWR_DOWN_PIN, HIGH);
  }

    void SelfOffsetCal()
  {
    digitalWrite(PWR_DOWN_PIN, LOW);
    SPI.transfer(0xF1);
    digitalWrite(PWR_DOWN_PIN, HIGH);
  }
  void SelfGainCal()
  {
    digitalWrite(PWR_DOWN_PIN, LOW);
    SPI.transfer(0xF2);
    digitalWrite(PWR_DOWN_PIN, HIGH);
  }
  
    void OffsetCal()
  {
    digitalWrite(PWR_DOWN_PIN, LOW);
    SPI.transfer(0xF3);
    digitalWrite(PWR_DOWN_PIN, HIGH);
  }
  void GainCal()
  {
    digitalWrite(PWR_DOWN_PIN, LOW);
    SPI.transfer(0xF4);
    digitalWrite(PWR_DOWN_PIN, HIGH);
  }

void ADCReset()
{
    digitalWrite(PWR_DOWN_PIN, LOW);
    SPI.transfer(0xFD);
    digitalWrite(PWR_DOWN_PIN, HIGH);
}

void readADCData() {
  // Read 24 bits of data
  
  digitalWrite(PWR_DOWN_PIN, HIGH);
  sensor_state_optimized(15);
  // delay(500);
while (digitalRead(DRDY_PIN)!=LOW)
{

}
// if (digitalRead(DRDY_PIN)==LOW)
// {
  for (int i = 0; i < 3; i++) {
    adcData[i] = SPI.transfer(0x00);
  }
   sensor_state_optimized(0);
   digitalWrite(PWR_DOWN_PIN, LOW);
    adcValue = (adcData[0] << 16) | (adcData[1] << 8) | adcData[2];
    adcValue = adcValue-Wtare;
//    Serial.println(adcValue);
    Weight=adcValue/Windex;
    PrevWeight=Weight;
    //  Serial.println(Weight);
    // Serial.print("ADC Value: ");
    // Serial.print("Data in HEX: ");
    // Serial.println(adcValue, HEX);
    // Serial.println(String("Data in DEC: ") +  adcValue);
    Serial.println(String("Weight: ") +  Weight + String(" gr"));
    // counter++;
    // meanSum=(Weight+meanSum);
    // if (counter==30){
    //   meanSum=meanSum/counter;
    //   counter=0;
    //   // Serial.println("*****************");
    //   // Serial.println(meanSum);
    //   // Serial.println("*****************");

    // }
    // sensor_state_optimized(0);
    // delay(100);
    
// }
  
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

int SPI_IO_expander_send(byte address, byte command)
{
  unsigned int bit_command = 0;
  bit_command = (bit_command | address) << 9;
  //Serial.print("not shifted: ");
  //Serial.println(bit_command,BIN);
  //bit_command=bit_command<<9;
  bit_command = bit_command | command;

  digitalWrite(IO_exp_nCS, LOW);
  SPI.transfer16(bit_command);
  digitalWrite(IO_exp_nCS, HIGH);
  /*
    Serial.print("Bit command: ");
    Serial.print(bit_command, HEX);
    Serial.print("  ");
    Serial.println(bit_command, BIN);
    return  bit_command;
  */
  return  bit_command;
}

int IO_exp_read(byte reg)
{
  byte reg_rcv = 0;
  unsigned int read_reg = 0b1000000000000000;
  unsigned int command = reg << 9;
  command = read_reg | command;
  // Serial.print("Register read :");
  // Serial.print(command, HEX);
  // Serial.print(" ");
  // Serial.println(command, BIN);

  digitalWrite(IO_exp_nCS, LOW);            // set the SS pin to LOW
  reg_rcv = SPI.transfer16(command);
  digitalWrite(IO_exp_nCS, HIGH);
  if (reg == 0)
  {
    Serial.print("I/O reg0: ");
  } else if (reg == 1)
  {
    Serial.print("I/O reg1: ");
  }

  Serial.println(reg_rcv, BIN);
  return reg_rcv;
}

void IO_reg_init()
{

  SPI_IO_expander_send(0x06, 0x00);
  SPI_IO_expander_send(0x02, EX_PORT0);
  SPI_IO_expander_send(0x07, 0x00);
  SPI_IO_expander_send(0x03, EX_PORT1);
}

void A2D2_sampling()
{
  byte Regconf = B11011111;

  EX_PORT1 = EX_PORT1 & Regconf;
  SPI_IO_expander_send(0x03, EX_PORT1);

  int A2D_VAL = SPI.transfer16(0);

  Regconf = B00100000;
  EX_PORT1 = EX_PORT1 | Regconf;
  SPI_IO_expander_send(0x03, EX_PORT1);
 Serial.println(String("ADC board: ") + A2D_VAL);

}


void setup() {
  Serial.begin(115200);

  pinMode(S2_0, OUTPUT);
  digitalWrite(S2_0, LOW);
  pinMode(S2_1, OUTPUT);
  digitalWrite(S2_1, LOW);
  pinMode(S2_2, OUTPUT);
  digitalWrite(S2_2, LOW);
  pinMode(S2_3, OUTPUT);
  digitalWrite(S2_3, LOW);

pinMode(IO_exp_nCS, OUTPUT);
digitalWrite(IO_exp_nCS, HIGH);
//  pinMode(DRDY_PIN, INPUT_PULLUP);
  pinMode(PWR_DOWN_PIN, OUTPUT);

digitalWrite(PWR_DOWN_PIN, HIGH);
delay(0.2);
digitalWrite(PWR_DOWN_PIN, LOW);
delay(0.2);
digitalWrite(PWR_DOWN_PIN, HIGH);
 
  SPI.begin();

  // Set SPI settings for ADS1232

  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

  // Ensure the ADS1232 is not in power-down mode initially
  digitalWrite(PWR_DOWN_PIN, LOW);  // Set power-down pin high to activate

  // No need to attach an interrupt to DRDY, as it's also the DOUT pin
  // WriteReg(0x00,0x06); // Gain
  // WriteReg(0x02,0x40); 
  // WriteReg(0x02,0x44); 
  IO_reg_init();
 sensor_state_optimized(0);
}


void loop() {
 
// WriteReg(0x00,0x07);
// ReadReg(0x00);
//  ReadReg(0x0A);
A2D2_sampling();
readADCData();
// delay(100);
//   IO_exp_read(0x00);
//   delay(100);
//   IO_exp_read(0x01);
//   delay(100);
//delay(300);
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
    } else if (serial_read == "selfoffcal") {
    SelfOffsetCal();
    Serial.println("Self offset calibration complete");
    } else if (serial_read == "selfgcal") {
    SelfGainCal();
    Serial.println("Self gain calibration complete");
    } else if (serial_read == "offsetcal") {
    OffsetCal();
    Serial.println("Offset calibration complete");
    } else if (serial_read == "gcal") {
    GainCal();
    Serial.println("Gain calibration complete");
    } else if (serial_read == "reset") {
      ADCReset();
      Serial.println("ADC reset complete");
    } else if (serial_read.substring(0, 5) == "wndx$") {
      String Wndx=serial_read.substring(5);
      Windex=Wndx.toFloat();
      Serial.println(String ("Windex set to: ") + Wndx );
    }

  }
}