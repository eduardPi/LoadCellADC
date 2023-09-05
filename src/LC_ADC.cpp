#include <SPI.h>
// ADS1232 24bit ADC works
// Pin Definitions
const int DRDY_PIN = 19;      // Data Ready / Data Out pin
const int PWR_DOWN_PIN = 22;  // Power-Down pin

String serial_read;


// Global variables
volatile bool dataReady = false;  // Data Ready flag
byte adcData[3];
int adcValue=0;
int Wtare=0;
float Windex=76;
float Weight=0;

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

}

void loop() {
  digitalWrite(PWR_DOWN_PIN, HIGH);
  delay(100);
  if (digitalRead(DRDY_PIN) == LOW) {
    // Data is ready, read ADC data
    readADCData();
    digitalWrite(PWR_DOWN_PIN, LOW);
    // Process and print the received data

  }

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
    }

  }
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
void readADCData() {
  // Read 24 bits of data
  for (int i = 0; i < 3; i++) {
    adcData[i] = SPI.transfer(0x00);
  }
    adcValue = (adcData[0] << 16) | (adcData[1] << 8) | adcData[2];
    adcValue = adcValue-Wtare;
    Weight=adcValue/Windex;
    Serial.print("ADC Value: ");
    Serial.print("Data in HEX: ");
    Serial.println(adcValue, HEX);
    Serial.println(String("Data in DEC: ") +  adcValue);
    Serial.println(String("Weight: ") +  Weight + String(" gr"));
    
  
}