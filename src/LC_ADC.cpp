/*
  Ver 8.1.1.a1_LC
  31/05/23
  Updates:
  MAX11203_




*/
#include <Arduino.h>
#include <PubSubClient.h>
//#include <AsyncMqttClient.h>
#include <HTTPClient.h>
#include <WiFi.h>
//#include <Shifter.h>
#include <ESPmDNS.h>
#include <Update.h>
//#include <WiFiClient.h>
#include <WebServer.h>
#include <EEPROM.h>
//#include <NTPClient.h>
#include <WiFiUdp.h>
#include <SPIFFS.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "time.h"
#include <SPI.h>
#include <ArduinoJson.h>


String FW_Version = "v8.1.1.a1_LC";

#define DisplayFreeHeap false
#define SPI_clk 1000000  // SPI communication clock speed in HZ
//#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
long uS_TO_S_FACTOR = 1000000;
//#define TIME_TO_SLEEP  5        /* Time ESP32 will go to sleep (in seconds) */
long TIME_TO_SLEEP = 0;
RTC_DATA_ATTR int bootCount = 0;

#define EEPROM_SIZE 500  // define the number of bytes you want to access
/* EEPROM mapping
  0-25 Board settings
  25
  60-79 ssid name
  80-99 ssid password
  100-149 local time web update
  150-169 ssid factory name
  170-189 ssid factory password
  200-229 UDP transmitting IP
  230-234 UDP transmitting port
  235-241 Scan time interval (milliseconds)
  242-245 Pixel Value Detection + Pixel Value Detection factory
  246-249 Sampling offset + Sampling offset factory
  291-    Dynamic Compensation Toggle
  292-293 Current sampling compare to Buffer Threshold  (A,R Add to string)
  294- Tx state
  295-296 Change detection + Change detection fctry
  297- : Pixel Mark State
  298- Sampling offset factory
  299- first FW flash bit set
  300-399 Primary config URL
  400-499 Secondary config URL

*/
#define Analog_read 34
#define WiFi_Button 35
#define OE2 22

//#define WiFi_LED 17
//#define COM_LED 21

//*****************MQTT settings******************

const char *mqtt_broker = "broker.emqx.io";
const char *topic = "Matrix_test";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);
//************************************************


#define IO_exp_nCS 5
#define A2D_nCS 21

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Define Analog sensor input  >>>>>>>>>>>>>>>>>>>>>>>>>>>
#define S0_0 15    //GPIO 15  on board
#define S0_1 32    //GPIO 32  on board
#define S0_2 33    //GPIO 33  on board
#define S0_3 25    //GPIO 25  on board


#define S1_0 26    //GPIO 26  on board
#define S1_1 27    //GPIO 27  on board
#define S1_2 14    //GPIO 14  on board
#define S1_3 12    //GPIO 12  on board

#define S2_0 16    //GPIO 16  on board
#define S2_1 4    //GPIO 4    on board
#define S2_2 0    //GPIO 0    on board
#define S2_3 2    //GPIO 2    on board
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
/*
  //<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Define Power switch output >>>>>>>>>>>>>>>>>>>>>>>>>>>>
  #define Shift_Data 5      //SER_IN  -SER
  #define Shift_Clk 19      //CLOCK   - SRCLK
  #define Shift_Latch 18    //Latch CLOCK   - RCLK
  #define shift_clear 15   // clear   - nSRCLR
*/


//***************Experimental outputs for N-ch Mosfet control********************************
//#define FET_Shift_Data 22      //SER_IN //SER2
//#define FET_Shift_Clk 22      //CLOCK  // SRCLK2
//#define FET_shift_clear 21   // clear  // nSRCLR2
//#define FET_shift_OE 17  // Output Enable  //nOE


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
const char* host = "esp32";
char ssid_reconnect[25] = {};
char password_reconnect[20] = {};
char ssid_reconnect_fctr[20] = {};
char password_reconnect_fctr[20] = {};
char UDP_TX_URL[30] = {};
char UDP_TX_Port[5] = {};
char FirstConfigURL[100] = {};
char SecondConfigURL[100] = {};

const char* ssid = 0;
const char* password = 0;


//----------------AP Host name Network Name and password--------------------------------
const char* host_name = "MatrixRFID";
const char* host_password = "123123";
//------------------------------------------------------------------------------------------
int addr = 0;
int Full_mtrx_clmn;  // Full matrix sign
int Full_mtrx_raw;   // Full matrix sign
int Column;      //   X Axis
int Raw;         //   Y Axis

int Column_start;  // X offset scanning
int Raw_start;      // Y offset scanning
int Raw_start_offset;      // Y offset
int Column_start_offset;// X offset

int Filter_scan;    // Put inside string only greater value than Filter_scan, else put 0
int Filter_tx;      // Send matrix only if value greater than value

// int loop_couter=0;

//////////////////   Buffer  //////////////////////////
int buffer_board[120][45];
byte buffer_board_flag[120][45];
// int PreprocessMesh[120][45];
int CompensationDelta[120][45];
int CompensatedMesh[120][45];
byte XActiveMeshFlag[120][45];
// int PreCompensatedMesh[120][45];
// int PreCompensatedMeshLast[120][45];

byte XActiveMeshFlagCount[45];
int XActiveMeshValueDelta[45];
int BufferEventTH = 0;
int Sample_raw = 1;
//int offset[5400];
int offset_counter = 0;
int Sampled_mesh = 0;
int Sampled_mesh_prev = 0;
int sampling = 0;
int sampling_offset = 0;




///////////////////////////////////////////////////////

byte scan_mode_http_counter = 0;

int sensor = 0;

int raw_buffer = 0;
int Rate_counter = 0;
int Sample_counter = 0;
int transmission_ID = 0;
int test = 0;
int x_offset = 0;
int scan_mode;
int pair = 0;
int UniqueID = 0;
int Test_y = 0;
int Test_x = 0;


byte IO_EX_REG_VAL = 0b00110100;
byte EX_PORT0 = B10110001;
byte EX_PORT1 = B11100000;

unsigned int A2D_VAL = 0;
unsigned long TotalPixel = 0;


unsigned int Scan_Time_Delay_usec = 5;
int SamplingValueSumPrev = 0;
int SamplingValueSum = 0;
int SamplingValueDelta = 0;
int DynSamplingDelta = 0;
unsigned long servConfigIntervalParamenter = 5000;
unsigned long servConfigIntervalParamenter2 = 5000; // Interval for keep alive Ping request

unsigned long sample_time = 0;
unsigned long connection_lost = 0;
unsigned long sample_mesh_Tx_duration = 0;
unsigned long ServConfigPingInterval = 0;
unsigned long ServConfigPingInterval2 = 0;
unsigned long getIDfaulseTimer = 0;


bool Item = false;   // Shelf-false (0)    foot board-true (1)
bool scan_state = false;
bool net_id = false;
bool HTTP_flag = false;
bool Tx_flag = false;
bool smple_rate_flag = false;
bool sample_1 = false;
bool sample_2 = false;
bool scan_pair_state = false;
bool string_show = false;
bool scan_show = false;
bool header_rec = false;
bool clear_scan = false;
bool offset_flag = false;
bool offset_reset_flag = false;
bool offtst = false;
bool web_last_config = false;
bool connection_abort_cmnd = false;
bool connection_lost_flag = false;
bool Testing_Mode = false;
bool Testing_scan = false;
bool auto_sample = false;
bool FET_flag = false;
bool samp_log = false;
bool log_state = false;
bool TxChangeFlag = false;
bool SrvChangeFlag = false;
bool udpAddresFlag = false;
bool udpPortFlag = false;
bool updActiveFlag = false;
bool wifiFlag = false;
bool PrimeServPostFlag = false;
bool SecondServPostFlag = false;
bool PixelMarkState = true;
bool serverfeedback = false;
bool bufconst = false;
bool TxAlways = false;
bool compensationFlag = false;
bool dynprint = false;
bool dyncomp = false;
bool dyncompFlag=false;
bool getID=false;

String con_abort = "abort";
String serial_read;
String request;
String Message;
String Message_UDP ;
String Scanshw;
String Header;
String Header2;
String Sampling_buffer;
String Sampling_buffer_Comp;
String ScnShwMessage;
String Web_column;
String Web_raw;
String web_update_date;
String passwrd_stars;
String passwrd_stars_fctr;
String ID_name;

byte past_connection = 0;
byte UDP_matrix_counter = 0;
byte ChangeDetectTH = 0;
int ValueDetectTH = 0;
int ValueDetectTHfctr = 0;
byte sampling_Top = 0;
byte Sampling_Top_steady = 0;
byte Scan_count = 0;


//<<<<<<<<<<<<<<<<<<<<<< Define UDP server >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

uint8_t UDP_send_buffer[1500];
uint8_t UDP_send_buffer_voaltile = 0;


//const char * udpAddress = "52.91.135.65";
//const char * udpAddress = "3.230.94.5"; // Amazon Dashboard server
const char * udpAddress = "34.125.251.53";  // Guy new server testing IP
//const char * udpAddress = "192.168.252.7";  //Idan Local
//const char * udpAddress = "34.125.197.97";  // Guy new server testing IP
//const char * udpAddress = "34.69.157.253";  // Guy new server testing IP
//const int udpPort = 2223;
unsigned int udpPort = 2223;
//create UDP instance
WiFiUDP udp;
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< NTP server time zone +2UTC >>>>>>>>>>>>>>>>>>>>>>>>>>>>
//const char* ntpServer = "pool.ntp.org";
//const int   daylightOffset_sec = 7200;
//const long  gmtOffset_sec = 7200;   // For UTC +2.0 2*60*60=7200




//<<<<<<<<<<<<<<<<<<<<<< Define NTP Client to get time >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

//WiFiUDP ntpUDP;
//NTPClient timeClient(udp);

//String formattedDate;
//String dayStamp;
//String timeStamp;

const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 7200;
const int   daylightOffset_sec = 3600;

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<MQTT functions>>>>>>>>>>>>>>>>>>>>>>>>>>>>


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<Get local time NTP server >>>>>>>>>>>>>>>>>>>>>>>>>

void printLocalTime()
{
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return;
  }
  if (!web_last_config)
  {
    Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  }


  if (web_last_config)
  {
    char timeStringBuff[50];
    strftime(timeStringBuff, sizeof(timeStringBuff), "%A, %B %d %Y %H:%M:%S", &timeinfo);
    //print like "const char*"
    //  Serial.println(timeStringBuff);
    //
    //  //Optional: Construct String object
    //  String asString(timeStringBuff);

    addr = 100;
    for (int i = 0; i < 50; i++) {
      EEPROM.write(addr + i, timeStringBuff[i]);
      EEPROM.commit();
    }


    int char_counter = 0;
    bool char_flag = LOW;
    web_update_date = "";
    for (int i = 100; i < 150; i++) {
      byte readValue = EEPROM.read(i);
      if (char_flag)
      {
        char_counter++;
      }

      if (char(readValue) == ':') {
        char_flag = HIGH;
      }

      web_update_date += char(readValue);

      if (char_counter == 5)
      {
        break;
      }
    }
    Serial.println(web_update_date);

    //  Serial.println(timeStringBuff);
    //   //Optional: Construct String object
    //  String asString(timeStringBuff);
    //   Serial.println(asString);
    //   Serial.println(asString.length());
  }

  // Serial.println(&timeinfo);
}
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< NTP EPOCH time >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
// Variable to save current epoch time
unsigned long timestamp;

// Function that gets current epoch time
unsigned long GetEpochTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return (0);
  }
  time(&now);
  return now;
}


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Login Page >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

const char* loginIndex =

  "<!DOCTYPE html>"
  "<html>"
  "<head>"
  "<style>"
  "h2 { color: white;"  // color Cean
  "     text-indent: 40%;"
  "     font-size:33px;"
  "     margin-left: 30px; } "
  " p { color: #CCFF33; " //color Yellow/green
  "       text-indent: 42%; "
  "      font-size:20px;"
  "    margin-left: 0px; }    "

  "@import url(https://fonts.googleapis.com/css?family=Open+Sans);"
  ".btn-large "
  "{ "
  "padding: 9px 14px; "
  "font-size: 20px; "
  "line-height: normal;"
  "-webkit-border-radius: 5px; "
  "-moz-border-radius: 5px; "
  "border-radius: 5px; "
  "}"

  ".btn:hover "
  "{ "
  "color: #333333; "
  "text-decoration: none; "
  "background-color: #e6e6e6; "
  "background-position: 0 -15px; "
  "-webkit-transition: background-position 0.1s linear; "
  "-moz-transition: background-position 0.1s linear; "
  "-ms-transition: background-position 0.1s linear;"
  "-o-transition: background-position 0.1s linear; "
  "transition: background-position 0.1s linear;"
  "}"

  ".btn-primary, .btn-primary:hover"
  "{"
  "text-shadow: 0 -1px 0 rgba(0, 0, 0, 0.25); color: #ffffff; }"
  ".btn-primary.active { color: rgba(255, 255, 255, 0.75); }"
  ".btn-primary { background-color: #4a77d4; "
  "background-image: -moz-linear-gradient(top, #6eb6de, #4a77d4); "
  "background-image: -ms-linear-gradient(top, #6eb6de, #4a77d4); "
  "background-image: -webkit-gradient(linear, 0 0, 0 100%, from(#6eb6de), to(#4a77d4));"
  "background-image: -webkit-linear-gradient(top, #6eb6de, #4a77d4); "
  "background-image: -o-linear-gradient(top, #6eb6de, #4a77d4); "
  "background-image: linear-gradient(top, #6eb6de, #4a77d4); "
  "background-repeat: repeat-x; "
  "filter: progid:dximagetransform.microsoft.gradient(startColorstr=#6eb6de, endColorstr=#4a77d4, GradientType=0);  "
  "border: 1px solid #3762bc; text-shadow: 1px 1px 1px rgba(0,0,0,0.4); "
  "box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.2), 0 1px 2px rgba(0, 0, 0, 0.5); "
  "}"

  ".btn-primary:hover, .btn-primary:active, .btn-primary.active, .btn-primary.disabled, .btn-primary[disabled] "

  "{ "
  "filter: none; "
  "background-color: #4a77d4; "
  "}"
  ".btn-block "
  "{ "
  "width: 100%; display:block; "
  "}"

  "* { -webkit-box-sizing:border-box; -moz-box-sizing:border-box; -ms-box-sizing:border-box; -o-box-sizing:border-box; box-sizing:border-box; }"

  "html { width: 100%; height:100%; overflow:hidden; }"


  "body { "
  "    width: 100%;"
  "    height:100%;"
  "    font-family: 'Open Sans', sans-serif;"
  "    background: #092756;"
  "    background: -moz-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 10%,rgba(138,114,76,0) 40%),-moz-linear-gradient(top,  rgba(47, 47, 39, 1) 0%, rgba(38, 53, 63,.2) 100%), -moz-linear-gradient(-45deg,  #0a6a72 0%, #092756 100%);"
  "    background: -webkit-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 50%,rgba(138,114,76,0) 40%), -webkit-linear-gradient(top,  rgba(47, 47, 39, 1) 0%,rgba(38, 53, 63,.2) 100%), -webkit-linear-gradient(-45deg,  #0a6a72 0%,#092756 100%);"
  "    background: -o-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 10%,rgba(138,114,76,0) 40%), -o-linear-gradient(top,  rgba(47, 47, 39, 1) 0%,rgba(38, 53, 63,.2) 100%), -o-linear-gradient(-45deg,  #0a6a72 0%,#092756 100%);"
  "    background: -ms-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 10%,rgba(138,114,76,0) 40%), -ms-linear-gradient(top,  rgba(47, 47, 39, 1) 0%,rgba(38, 53, 63,.2) 100%), -ms-linear-gradient(-45deg,  #0a6a72 0%,#092756 100%);"
  "    background: -webkit-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 10%,rgba(138,114,76,0) 40%), linear-gradient(to bottom,  rgba(47, 47, 39, 1) 0%,rgba(38, 53, 63,.2) 100%), linear-gradient(-45deg,  #0a6a72 0%,#092756 100%);"
  "    filter: progid:DXImageTransform.Microsoft.gradient( startColorstr='#3E1D6D', endColorstr='#092756',GradientType=1 );"
  "}"

  ".login { "
  "    position: absolute;"
  "    top: 50%;"
  "    left: 50%;"
  "    margin: -150px 0 0 -150px;"
  "    width:300px;"
  "    height:300px;"
  "}"
  ".login h1 { color: #fff; text-shadow: 0 0 10px rgba(0,0,0,0.3); letter-spacing:1px; text-align:center; }"

  "input { "
  "    width: 100%; "
  "    margin-bottom: 10px; "
  "    background: rgba(0,0,0,0.3);"
  "    border: none;"
  "    outline: none;"
  "    padding: 10px;"
  "    font-size: 13px;"
  "    color: #fff;"
  "    text-shadow: 1px 1px 1px rgba(0,0,0,0.3);"
  "    border: 1px solid rgba(0,0,0,0.3);"
  "    border-radius: 4px;"
  "    box-shadow: inset 0 -5px 45px rgba(100,100,100,0.2), 0 1px 1px rgba(255,255,255,0.2);"
  "    -webkit-transition: box-shadow .5s ease;"
  "    -moz-transition: box-shadow .5s ease;"
  "    -o-transition: box-shadow .5s ease;"
  "    -ms-transition: box-shadow .5s ease;"
  "    transition: box-shadow .5s ease;"
  "}"
  "input:focus { box-shadow: inset 0 -5px 45px rgba(100,100,100,0.4), 0 1px 1px rgba(255,255,255,0.2); }"

  "</style>"
  "</head>"
  "<body>"
  "<h2>ESP32 Board Uploader</h2> "
  "<p>Matrix powered by Eduard D.</p> "
  "<hr>"
  "<div class=\"login\">"
  "   <h1>Login</h1>"
  "   <form>"
  "       <input type=\"text\" name=\"userid\" placeholder=\"Username\" required=\"required\" />"
  "       <input type=\"password\" name=\"psw\" placeholder=\"Password\" required=\"required\" />"
  "       <button type=\"submit\" onclick='check(this.form)' class=\"btn btn-primary btn-block btn-large\">Let me in</button>"
  "   </form>"
  "</div>"


  "<script>"

  " function check(form)"
  " {"
  " if(form.userid.value=='Matrix' && form.psw.value=='ENG123')"
  " {"
  // " window.open('/serverIndex',\"_self\");"
  "window.open('/serverIndex')"
  "}"
  " else"
  "{"

  "   alert('Error Password or Username')/*displays error message*/"
  "  }"
  " }"

  "</script>"

  "</body>"
  "</html>";



//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Web Server Page >>>>>>>>>>>>>>>>>>>>>>>>>>>>

String MatrixConfig (int SendColumn, int SendRaw , int SendFullColumn, int SendFullRaw, int wifipwr, int RawOffset, int ColumnOffset, int Scfltr, int Httpfltr, int SysID, int Scan_mode, String FW_ver, String Web_config_update, String net_name, String net_pswrd, String Tramsmitting_URL, String Tramsmitting_PORT, unsigned int ScanDelay_usec, bool Fet_state, int samp_offset, bool Smpl_log, String SysIDglobal, bool tx_state, String MAC, byte changeDetectTH, String Web1stConfigServer, String Web2ndConfigServer, byte HighSteadyPxl, int HighSteadyValue, int MeshEventTH, int ValueDetectTH, int ValueDetectTH_, bool PixState, bool DynamicComp)
{

String DynamicCompensation;
  if (DynamicComp)
  {
    DynamicCompensation = "checked";
  } else {
    DynamicCompensation = "";
  }

  String Fet_state_tx;
  if (Fet_state)
  {
    Fet_state_tx = "checked";
  } else {
    Fet_state_tx = "";
  }

  String Sampling_log;
  if (Smpl_log)
  {
    Sampling_log = "checked";
  } else {
    Sampling_log = "";
  }
  String TxState;
  if (tx_state)
  {
    TxState = "checked";
  } else {
    TxState = "";
  }

  String Item_web;
  if (Item)
  {
    Item_web = "Foot board";
  } else {
    Item_web = "Shelf board";
  }

  String PixelState;
  if (PixState)
  {
    PixelState = "checked";
  } else {
    PixelState = "";
  }

  String serverIndex =
    "<!DOCTYPE html>"
    "  <html>"
    "  <head>"
    "  <style>"
    "  h2 { color: white;"  // color Cean
    "      text-indent: 0%;"
    "     font-size:33px;"
    //    "    margin-left: 100px;}"
    "    text-align: center;}"

    "   p { color: #CCFF33; "//color Yellow/green
    "        text-indent: 0%;"
    "       font-size:20px;"
    //  "    margin-left: 20px;}"
    //"    margin-right: 50px;}"
    // "}"
    "     text-align: center;}"
    "p2 { color: white;"
    "font-size:20px;}"
    "label {"
    "  color: #b9e028;"
    "  padding: 8px;"
    "  font-family: Arial;"
    "}"
    "  @import url(https://fonts.googleapis.com/css?family=Open+Sans);"

    "  .btn-large"
    "  {"
    "  padding: 9px 14px;"
    "  font-size: 20px;"
    "  line-height: normal;"
    "  -webkit-border-radius: 5px;"
    "  -moz-border-radius: 5px;"
    "  border-radius: 5px;"
    "  }"

    "  .btn:hover"
    "  {"
    "  color: #333333;"
    "  text-decoration: none;"
    "  background-color: #e6e6e6;"
    "  background-position: 0 -15px;"
    "  -webkit-transition: background-position 0.1s linear;"
    "  -moz-transition: background-position 0.1s linear;"
    "  -ms-transition: background-position 0.1s linear;"
    "  -o-transition: background-position 0.1s linear;"
    "  transition: background-position 0.1s linear;"
    "  }"

    "  .btn-primary, .btn-primary:hover"
    "  {"
    "  text-shadow: 0 -1px 0 rgba(0, 0, 0, 0.25); color: #ffffff; }"
    "  .btn-primary.active { color: rgba(255, 255, 255, 0.75); }"
    "  .btn-primary { background-color: #4a77d4;"
    "  background-image: -moz-linear-gradient(top, #6eb6de, #4a77d4);"
    "  background-image: -ms-linear-gradient(top, #6eb6de, #4a77d4);"
    "  background-image: -webkit-gradient(linear, 0 0, 0 100%, from(#6eb6de), to(#4a77d4));"
    "  background-image: -webkit-linear-gradient(top, #6eb6de, #4a77d4);"
    "  background-image: -o-linear-gradient(top, #6eb6de, #4a77d4);"
    "  background-image: linear-gradient(top, #6eb6de, #4a77d4);"
    "  background-repeat: repeat-x;"
    "  filter: progid:dximagetransform.microsoft.gradient(startColorstr=#6eb6de, endColorstr=#4a77d4, GradientType=0);"
    "  border: 1px solid #3762bc; text-shadow: 1px 1px 1px rgba(0,0,0,0.4);"
    "  box-shadow: inset 0 1px 0 rgba(255, 255, 255, 0.2), 0 1px 2px rgba(0, 0, 0, 0.5);"
    "  }"

    "  .btn-primary:hover, .btn-primary:active, .btn-primary.active, .btn-primary.disabled, .btn-primary[disabled]"

    "  {"
    "  filter: none;"
    "  background-color: #4a77d4;"
    "  }"
    "  .btn-block"
    "  {"
    "  width: 100%;"
    "  display:block;"
    "  }"

    "  * { -webkit-box-sizing:border-box; -moz-box-sizing:border-box; -ms-box-sizing:border-box; -o-box-sizing:border-box; box-sizing:border-box; }"

    "  html { width: 100%; height:100%;}"  // overflow:hidden; }"       html scroll hide


    "  body {"
    "      width: 100%;"
    "      height:100%;"
    "     font-family: 'Open Sans', sans-serif;"
    "     background: #092756;"
    "     background: -moz-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 10%,rgba(138,114,76,0) 40%),-moz-linear-gradient(top,  rgba(47, 47, 39, 1) 0%, rgba(38, 53, 63,.2) 100%), -moz-linear-gradient(-45deg,  #0a6a72 0%, #092756 100%);"
    "     background: -webkit-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 50%,rgba(138,114,76,0) 40%), -webkit-linear-gradient(top,  rgba(47, 47, 39, 1) 0%,rgba(38, 53, 63,.2) 100%), -webkit-linear-gradient(-45deg,  #0a6a72 0%,#092756 100%);"
    "     background: -o-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 10%,rgba(138,114,76,0) 40%), -o-linear-gradient(top,  rgba(47, 47, 39, 1) 0%,rgba(38, 53, 63,.2) 100%), -o-linear-gradient(-45deg,  #0a6a72 0%,#092756 100%);"
    "     background: -ms-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 10%,rgba(138,114,76,0) 40%), -ms-linear-gradient(top,  rgba(47, 47, 39, 1) 0%,rgba(38, 53, 63,.2) 100%), -ms-linear-gradient(-45deg,  #0a6a72 0%,#092756 100%);"
    "      background: -webkit-radial-gradient(0% 100%, ellipse cover, rgba(38, 53, 63,.2) 10%,rgba(138,114,76,0) 40%), linear-gradient(to bottom,  rgba(47, 47, 39, 1) 0%,rgba(38, 53, 63,.2) 100%), linear-gradient(-45deg,  #0a6a72 0%,#092756 100%);"
    "      filter: progid:DXImageTransform.Microsoft.gradient( startColorstr='#3E1D6D', endColorstr='#092756',GradientType=1 );"
    "  }"



    "  input {"
    "     width: 100%;"
    "     margin-bottom: 5px;"
    "     background: rgba(0,0,0,0.3);"
    "     border: none;"
    "     outline: none;"
    "    padding: 10px;"
    "    font-size: 13px;"
    "    color: #fff;"
    "    text-shadow: 1px 1px 1px rgba(0,0,0,0.3);"
    "    border: 1px solid rgba(0,0,0,0.3);"
    "    border-radius: 4px;"
    "      box-shadow: inset 0 -5px 45px rgba(100,100,100,0.2), 0 1px 1px rgba(255,255,255,0.2);"
    "     -webkit-transition: box-shadow .5s ease;"
    "     -moz-transition: box-shadow .5s ease;"
    "      -o-transition: box-shadow .5s ease;"
    "      -ms-transition: box-shadow .5s ease;"
    "      transition: box-shadow .5s ease;"
    "  }"
    "  input:focus { box-shadow: inset 0 -5px 45px rgba(100,100,100,0.4), 0 1px 1px rgba(255,255,255,0.2); }"


    "  div {"
    "  font-size: 30px;"
    "  color: RGB(0, 204, 204,1);"
    "  margin-left: 600px;"
    "  text-align: center;"
    "  }"

    "  .pickfile {"
    "      position: absolute;"
    "      top: 50%;"
    "      left: 50%;"
    "      margin: -150px 0 0 -150px;"
    "      width:300px;"
    "      height:300px;"
    "  }"

    "   .switch {"
    "  position: relative;"
    "  display: inline-block;"
    "  width: 55px;"
    "  height: 30px;"
    "}"

    ".switch input { "
    "  opacity: 0;"
    "  width: 0;"
    "  height: 0;"
    "}"

    ".slider {"
    "  position: absolute;"
    "  cursor: pointer;"
    "  top: 0;"
    "  left: 0;"
    " right: 0;"
    " bottom: 0;"
    " background-color: #646464;"
    " -webkit-transition: .4s;"
    "  transition: .4s;"
    "} "
    ".slider:before {"
    "  position: absolute;"
    " content: \"\";"
    "  height: 22px;"
    "  width: 22px;"
    "  left: 4px;"
    "  bottom: 4px;"
    "  background-color: #b9e028;"
    "  -webkit-transition: .4s;"
    "  transition: .4s;"
    "}"

    "input:checked + .slider {"
    "  background-color: #2196F3;"
    "}"

    "input:focus + .slider {"
    "  box-shadow: 0 0 1px #2196F3;"
    "}"

    "input:checked + .slider:before {"
    "  -webkit-transform: translateX(26px);"
    "  -ms-transform: translateX(26px);"
    "  transform: translateX(26px);"
    "}"

    /* Rounded sliders */
    ".slider.round {"
    "  border-radius: 10px;"
    "}"

    ".slider.round:before {"
    "  border-radius: 50%;"
    "}"

    "   </style>"
    "   </head>"

    "  <body>"

    "  <h2>MatrixID Board Configuration </h2>"
    "<p2 style=\"font-size: 15px;\"> FW version: " + FW_ver + "</p2>"
    "<p2 style=\"font-size: 15px;padding-left: 70%;\"> FW Type: " + Item_web + "</p2>"
    "  <hr>"
    "  <br>"
    "  <p>Pick a .bin file and upload </p>"
    "  <br>"
    //    "  <br>"

    "  <script src = 'https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'> </script>"
    "  <form method = 'POST' action = '#' enctype = 'multipart/form-data' id = 'upload_form' class = 'pickfile'>"
    "  <input type = 'file' name = 'update'>"
    "  <button type = 'submit' class = \"btn btn-primary btn-block btn-large\">Upload file</button>"
    "   </form>"
    "   <div id='prg'  >progress: 0%</div>"

    "   <script>"
    "    $('form').submit(function(e){"
    "    e.preventDefault();"
    "    var form = $('#upload_form')[0];"
    "    var data = new FormData(form);"
    "     $.ajax({"
    "    url: '/update',"
    "    type: 'POST',"
    "    data: data,"
    "    contentType: false,"
    "    processData:false,"
    "    xhr: function() {"
    "    var xhr = new window.XMLHttpRequest();"
    "    xhr.upload.addEventListener('progress', function(evt) {"
    "    if (evt.lengthComputable) {"
    "    var per = evt.loaded / evt.total;"
    "    $('#prg').html('progress: ' + Math.round(per*100) + '%');"
    "    }"
    "    }, false);"
    "    return xhr;"
    "    },"
    "    success:function(d, s) {"
    "    console.log('success!')"
    "    },"
    "    error: function (a, b, c) {"
    "    }"
    "    });"
    "    });"
    "    </script>"
    "   <br><br><br><br><br><br><br><br>"
    "    <p2> Matrix Setup/</p2><p2 style=\"color:#db0f2b;\">Factory settings</p2>"
    "    <p2 style=\"padding-left: 65%;font-size: 15px;font-weight: bold;\">Last update: </p2><p2 style=\"font-size: 15px;\">" + Web_config_update + "</p2>"
    "   <hr>"
    "     <form action=\"/action_page\">"
    "<label for = \"raw\" style=\"padding-right: 30px;\">Sampling Matrix H (SR):</label>  <input type=\"number\" id=\"raw\" name=\"raw\" value=\"" + String(SendRaw) + "\"  min=\"0\" max=\"255\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"rawfctr\" name=\"rawfctr\" value=\"" + String(EEPROM.read(41)) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;color:#db0f2b;\">"

    "<label for=\"column\" style=\"padding-left: 78px;padding-right: 35px;\">Sampling Matrix W (M):</label><input type=\"number\"  id=\"column\" name=\"column\" value=\"" + String(SendColumn) + "\"min=\"0\" max=\"255\" style=\"height: 20px; width:70px;\"><input type=\"number\"  id=\"columnfctr\" name=\"columnfctr\" value=\"" + String(EEPROM.read(40)) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px; color:#db0f2b;\">"
    //    "<br>"
    "<label for=\"ScanMode\" style=\"padding-left: 383px; padding-right: 27px;\">Scan Mode:</label>"
    "<label for=\"SingleScan\" style=\"padding-left: 130px; color:white;\">Single</label>"
    "<label class=\"switch\"> <input type=\"checkbox\"" + Fet_state_tx + " id=\"ScanState\" name=\"ScanState\"><span class=\"slider round\"></span></label>"
    "<label for=\"DoubleScan\" style=\"color:white;\">Double</label>"

    //    "<label for=\"fraw\" style=\"padding-right: 72px;\">Full Matrix H (SR):</label>  <input type=\"number\" id=\"fraw\" name=\"fraw\" value=\"" + String(SendFullRaw) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"frawfctr\" name=\"frawfctr\" value=\"" + String(EEPROM.read(44)) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;color:#db0f2b; "padding-right: 72px;\">"

    //    "<label for=\"fcolumn\" style=\"padding-left: 78px;padding-right: 70px;\" >Full Matrix W (M):</label><input type=\"number\"  id=\"fcolumn\" name=\"fcolumn\" value=\"" + String(SendFullColumn) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;\"><input type=\"number\"  id=\"fcolumnfctr\" name=\"fcolumnfctr\" value=\"" + String(EEPROM.read(43)) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    "<br>"
    // "<label for=\"roffset\" style=\"padding-right: 64px;\" >Raw starts on (SR):</label>  <input type=\"number\" id=\"roffset\" name=\"roffset\" value=\"" + String(RawOffset) + "\" min=\"0\" max=\"45\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"roffsetfctr\" name=\"roffsetfctr\" value=\"" + String(EEPROM.read(42)) + "\" min=\"0\" max=\"45\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    //"<br>"
    // "<br>"
    "<label for=\"samp_offset\"style=\"padding-right: 90px;\" >Sampling Offset:</label><input type=\"number\"  id=\"samp_offset\" name=\"samp_offset\" value=\"" + String(samp_offset) + "\" min=\"0\" max=\"4000\" style=\"height: 20px; width:70px;\"><input type=\"number\"  id=\"samp_offsetfctr\" name=\"samp_offsetfctr\" value=\"" + String(EEPROM.read(298)) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    "<label for=\"log10\" style=\"padding-left: 800px; padding-right: 30px;\">Sampling Logarithmic:</label>"
    "<label for=\"LogOff\" style=\"padding-left: 80px; color:white;\">Off</label>"
    "<label class=\"switch\"> <input type=\"checkbox\"" + Sampling_log + " id=\"Scan_log\" name=\"Scan_log\"><span class=\"slider round\"></span></label>"
    "<label for=\"LogOn\" style=\"color:white;\">On</label>"
    "<br>"
    //    "<label for=\"coffset\"style=\"padding-right: 118px;\" >X_Offset(M):</label><input type=\"number\"  id=\"coffset\" name=\"coffset\" value=\"" + String(ColumnOffset) + "\" min=\"0\" max=\"80\" style=\"height: 20px; width:70px;\"><input type=\"number\"  id=\"coffsetfctr\" name=\"coffsetfctr\" value=\"" + String(EEPROM.read(45)) + "\" min=\"0\" max=\"80\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    //    "<br>"
    "<label for=\"httpfilter\" style=\"padding-right: 34px;\">Tx HTTP Filter above of:</label><input type=\"number\" id=\"httpfilter\" name=\"httpfilter\" value=\"" + String(Httpfltr) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"httpfilterfctr\" name=\"httpfilterfctr\" value=\"" + String((EEPROM.read(47) * 50)) + "\" min=\"0\" max=\"4095\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    "<label for=\"Tx_state\" style=\"padding-left: 800px; padding-right: 30px;\">Tx State:</label>"
    "<label for=\"Off\" style=\"padding-left: 174px; color:white;\">Off</label>"
    "<label class=\"switch\"> <input type=\"checkbox\"" + TxState + " id=\"TxState\" name=\"TxState\"><span class=\"slider round\"></span></label>"
    "<label for=\"On\" style=\"color:white;\">On</label>"
    "<br>"
    "<label for=\"scanfilter\" style=\"padding-right: 76px;\">Scan Filter below:</label>  <input type=\"number\" id=\"scanfilter\" name=\"scanfilter\"value=\"" + String(Scfltr) + "\" min=\"0\" max=\"1500\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"scanfilterfctr\" name=\"scanfilterfctr\" value=\"" + String((EEPROM.read(46) * 50)) + "\"  min=\"0\" max=\"1500\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    "<label for=\"PixelMarkState\" style=\"padding-left: 800px; padding-right: 30px;\">Pixel Mark State A/R:</label>"
    "<label for=\"PixelOff\" style=\"padding-left: 86px; color:white;\">Off</label>"
    "<label class=\"switch\"> <input type=\"checkbox\"" + PixelState + " id=\"PixelMarkState\" name=\"PixelMarkState\"><span class=\"slider round\"></span></label>"
    "<label for=\"PixelOn\" style=\"color:white;\">On</label>"
    "<br>"
    "<label for=\"MeshEventTH\" style=\"padding-right: 24px;\">Mesh Event Mark A/R TH:</label><input type=\"number\" id=\"MeshEventTH\" name=\"MeshEventTH\" value=\"" + String(MeshEventTH) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"MeshEventTHfctr\" name=\"MeshEventTHfctr\" value=\"" + String(EEPROM.read(292)) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    "<label for=\"DynamicCompensate\" style=\"padding-left: 800px; padding-right: 30px;\">Dynamic Compensation:</label>"
    "<label for=\"DynamicCompOff\" style=\"padding-left: 62px; color:white;\">Off</label>"
    "<label class=\"switch\"> <input type=\"checkbox\"" + DynamicCompensation + " id=\"DynamicCompensation\" name=\"DynamicCompensation\"><span class=\"slider round\"></span></label>"
    "<label for=\"DynamicCompOn\" style=\"color:white;\">On</label>"
    "<br>"
    "<label for=\"ValueDetectionTH\" style=\"padding-right: 67px;\">Value Detection TH:</label><input type=\"number\" id=\"ValueDetectionTH\" name=\"ValueDetectionTH\" value=\"" + String(ValueDetectTH) + "\" min=\"0\" max=\"5000\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"ValueDetectionTHfctr\" name=\"ValueDetectionTHfctr\" value=\"" + String(ValueDetectTH_) + "\" min=\"0\" max=\"5000\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    "<br>"
    "<label for=\"ChangeDetectionTH\" style=\"padding-right: 72px;\">Pixel Detection TH:</label><input type=\"number\" id=\"ChangeDetectionTH\" name=\"ChangeDetectionTH\" value=\"" + String(changeDetectTH) + "\" min=\"0\" max=\"50\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"ChangeDetectionTHfctr\" name=\"ChangeDetectionTHfctr\" value=\"" + String(EEPROM.read(296)) + "\" min=\"0\" max=\"50\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    "<label for=\"HighSteadyValue\" style=\"padding-left: 83px;padding-right: 20px;\">Max Steady Pxl / Value:</label><label for=\"\" style=\"color:white;\">" + String(HighSteadyPxl) + " / " + String(HighSteadyValue) + "</label>"
    "<br>"
    "<label for=\"systemid\" style=\"padding-right: 70px;\">System Local ID #:</label>  <input type=\"number\" id=\"systemid\" name=\"systemid\" value=\"" + String(SysID) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"systemidfctr\" name=\"systemidfctr\" value=\"" + String(EEPROM.read(49)) + "\" min=\"0\" max=\"255\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    //    "<br>"
    "<label for=\"systemIDglobal\" style=\"padding-left: 83px;padding-right: 60px;\">System ID Global:</label><label for=\"SysID\" style=\"color:white;\">" + String(ID_name) + "</label>"
    "<br>"
    "<label for=\"wifipwr\" style=\"padding-right:97px;\">Wifi RF Power:</label> <input type=\"number\" id=\"wifipwr\" name=\"wifipwr\" value=\"" + String(wifipwr) + "\" min=\"0\" max=\"11\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"wifipwrfctr\" name=\"wifipwrfctr\" value=\"" + String(EEPROM.read(50)) + "\" min=\"0\" max=\"11\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    "<label for=\"MAC_ADDRESS\" style=\"padding-left: 83px;padding-right: 90px;\">System MAC:</label><label for=\"MAC\" style=\"color:white;\">" + MAC + "</label>"

    "<br>"
    //"<br>"
    //    "<label for=\"Scanmode\" style=\"padding-right:120px;\">Scan Mode:</label> <input type=\"number\" id=\"Scanmode\" name=\"Scanmode\" value=\"" + String(Scan_mode) + "\" min=\"0\" max=\"1\" style=\"height: 20px; width:70px;\"><input type=\"number\" id=\"Scanmodefctr\" name=\"Scanmodefctr\" value=\"" + String(EEPROM.read(51)) + "\" min=\"0\" max=\"1\" style=\"height: 20px; width:70px;color:#db0f2b;\">"
    //    "<br>"
    "<label for=\"ScanTimeInterval\" style=\"padding-right:68px;\">Scan interval usec:</label> <input type=\"number\" id=\"ScanTimeInterval\" name=\"ScanTimeInterval\" value=\"" + String(ScanDelay_usec) + "\" min=\"0\" max=\"5000\" style=\"height: 20px; width:70px;\">"
    "<br>"

    "<label for=\"netname\" style=\"padding-right: 94px;\">Network Name:</label><input type=\"text\" id=\"netname\" name=\"netname\" value=\"" + String(net_name) + "\"style=\"height: 20px; width:150px;\">"
    //    "<label for=\"netnamefctr\" style=\"padding-left: 78px; padding-right: 33px;\">Factory Network Name:</label><input type=\"text\" id=\"netnamefctr\" name=\"netnamefctr\" value=\"" + String(net_name_fctr) + "\"style=\"height: 20px; width:150px;color:#db0f2b;\">"
    "<label for=\"TransmittingURL\" style=\"padding-left: 77px; padding-right: 33px;\">UDP Tx Address:</label><input type=\"text\" id=\"TransmittingURL\" name=\"TransmittingURL\" value=\"" + Tramsmitting_URL + "\"style=\"height: 20px; width:150px;\">"    // change UDP socket address for transmitting
    "<br>"

    "<label for=\"netpswrd\"style=\"padding-right: 66px;\">Network Password:</label><input type=\"text\" id=\"netpswrd\" name=\"netpswrd\" value=\"" + String(net_pswrd) + "\"style=\"height: 20px; width:150px;\">"
    //    "<label for=\"netpswrdfctr\"style=\"padding-left: 78px; padding-right: 5px;\">Factory Network Password:</label><input type=\"text\" id=\"netpswrdfctr\" name=\"netpswrdfctr\" value=\"" + String(net_pswrd_fctr) + "\"style=\"height: 20px; width:150px;color:#db0f2b;\">"
    "<label for=\"TransmittingPort\"style=\"padding-left: 77px; padding-right: 62px;\">UDP Tx Port:</label><input type=\"text\" id=\"TransmittingPort\" name=\"TransmittingPort\" value=\"" + String(Tramsmitting_PORT) + "\"style=\"height: 20px; width:150px;\">" // change UDP socket port for transmitting
    "<br>"
    "<label for=\"PrimaryServerConfig\"style=\"padding-right: 75px;\">1st Config Server:</label><input type=\"text\" id=\"PrimaryServerConfig\" name=\"PrimaryServerConfig\" value=\"" + String(Web1stConfigServer) + "\"style=\"height: 20px; width:530px;\">"
    "<br>"
    "<label for=\"SecondaryServerConfig\"style=\"padding-right: 70px;\">2nd Config Server:</label><input type=\"text\" id=\"SecondaryServerConfig\" name=\"SecondaryServerConfig\" value=\"" + String(Web2ndConfigServer) + "\"style=\"height: 20px; width:530px;\">"

    "<button type = \"submit\" class = \"btn btn-primary btn-block btn-large\" style=\"margin-left: 47%;color:white; height: 70px; width:100px;\">Config Matrix</button>"

    "</form>"
    "  </body>"
    "  </html>";

  return serverIndex;
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<< Server IP parameters define >>>>>>>>>>>>>>>>>>>>>>>>>>
//WiFiServer server(80);
WebServer server(80);

IPAddress local_IP(192, 168, 1, 100);
IPAddress gateway(192, 168, 0, 2);
IPAddress subnet(255, 255, 255, 0);


//<<<<<<<<<<<<<<<<<<<<<<<<<<<< Client Static IP parameters define >>>>>>>>>>>>>>>>>>>
//WiFiClient client;


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                        PROGRAMM FUNCTIONS                                                                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Serial communication >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

byte sensor_state_optimized( byte optimized_state)
{
  switch (optimized_state)
  {
    case 0:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, LOW); digitalWrite(S0_1, LOW); digitalWrite(S0_0, LOW);
      //      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      //      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 1:
      digitalWrite(S0_0, HIGH);

      break;
    case 2:
      digitalWrite(S0_1, HIGH); digitalWrite(S0_0, LOW);
      break;
    case 3:
      digitalWrite(S0_0, HIGH);
      break;
    case 4:
      digitalWrite(S0_2, HIGH); digitalWrite(S0_1, LOW); digitalWrite(S0_0, LOW);
      break;
    case 5:
      digitalWrite(S0_0, HIGH);
      break;
    case 6:
      digitalWrite(S0_1, HIGH); digitalWrite(S0_0, LOW);
      break;
    case 7:
      digitalWrite(S0_0, HIGH);
      break;
    case 8:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, LOW); digitalWrite(S0_1, LOW); digitalWrite(S0_0, LOW);
      break;
    case 9:
      digitalWrite(S0_0, HIGH);
      break;
    case 10:
      digitalWrite(S0_1, HIGH); digitalWrite(S0_0, LOW);
      break;
    case 11:
      digitalWrite(S0_0, HIGH);
      break;
    case 12:
      digitalWrite(S0_2, HIGH); digitalWrite(S0_1, LOW); digitalWrite(S0_0, LOW);
      break;
    case 13:
      digitalWrite(S0_0, HIGH);
      break;
    case 14:
      digitalWrite(S0_1, HIGH); digitalWrite(S0_0, LOW);
      break;
    case 15:
      digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      break;
    case 16:
      digitalWrite(S1_0, HIGH);
      break;
    case 17:
      digitalWrite(S1_1, HIGH); digitalWrite(S1_0, LOW);
      break;
    case 18:
      digitalWrite(S1_0, HIGH);
      break;
    case 19:
      digitalWrite(S1_2, HIGH); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      break;
    case 20:
      digitalWrite(S1_0, HIGH);
      break;
    case 21:
      digitalWrite(S1_1, HIGH); digitalWrite(S1_0, LOW);
      break;
    case 22:
      digitalWrite(S1_0, HIGH);
      break;
    case 23:
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      break;
    case 24:
      digitalWrite(S1_0, HIGH);
      break;
    case 25:
      digitalWrite(S1_1, HIGH); digitalWrite(S1_0, LOW);
      break;
    case 26:
      digitalWrite(S1_0, HIGH);
      break;
    case 27:
      digitalWrite(S1_2, HIGH); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      break;
    case 28:
      digitalWrite(S1_0, HIGH);
      break;
    case 29:
      digitalWrite(S1_1, HIGH); digitalWrite(S1_0, LOW);
      break;
    case 30:
      digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 31:
      digitalWrite(S2_0, HIGH);
      break;
    case 32:
      digitalWrite(S2_1, HIGH); digitalWrite(S2_0, LOW);
      break;
    case 33:
      digitalWrite(S2_0, HIGH);
      break;
    case 34:
      digitalWrite(S2_2, HIGH); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 35:
      digitalWrite(S2_0, HIGH);
      break;
    case 36:
      digitalWrite(S2_1, HIGH); digitalWrite(S2_0, LOW);
      break;
    case 37:
      digitalWrite(S2_0, HIGH);
      break;
    case 38:
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 39:
      digitalWrite(S2_0, HIGH);
      break;
    case 40:
      digitalWrite(S2_1, HIGH); digitalWrite(S2_0, LOW);
      break;
    case 41:
      digitalWrite(S2_0, HIGH);
      break;
    case 42:
      digitalWrite(S2_2, HIGH); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 43:
      digitalWrite(S2_0, HIGH);
      break;
    case 44:
      digitalWrite(S2_1, HIGH); digitalWrite(S2_0, LOW);
      break;

  }
  return optimized_state;
}

byte sensor_state( byte state)
{
  switch (state)
  {
    case 0:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, LOW); digitalWrite(S0_1, LOW); digitalWrite(S0_0, LOW);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 1:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, LOW); digitalWrite(S0_1, LOW); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 2:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, LOW); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, LOW);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 3:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, LOW); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 4:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, LOW); digitalWrite(S0_0, LOW);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 5:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, LOW); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 6:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, LOW);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 7:
      digitalWrite(S0_3, LOW); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 8:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, LOW); digitalWrite(S0_1, LOW); digitalWrite(S0_0, LOW);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 9:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, LOW); digitalWrite(S0_1, LOW); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 10:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, LOW); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, LOW);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 11:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, LOW); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 12:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, LOW); digitalWrite(S0_0, LOW);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 13:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, LOW); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 14:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, LOW);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 15:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 16:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 17:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 18:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, LOW); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 19:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 20:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, LOW); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 21:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 22:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, LOW); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 23:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 24:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, LOW); digitalWrite(S1_1, LOW); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 25:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, LOW); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 26:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, LOW); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 27:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, LOW); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 28:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, LOW); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 29:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, LOW);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 30:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 31:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, HIGH);
      break;
    case 32:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, LOW);
      break;
    case 33:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, LOW); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, HIGH);
      break;
    case 34:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 35:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, LOW); digitalWrite(S2_0, HIGH);
      break;
    case 36:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, LOW);
      break;
    case 37:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, LOW); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, HIGH);
      break;
    case 38:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 39:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, LOW); digitalWrite(S2_1, LOW); digitalWrite(S2_0, HIGH);
      break;
    case 40:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, LOW); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, LOW);
      break;
    case 41:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, LOW); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, HIGH);
      break;
    case 42:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, LOW); digitalWrite(S2_0, LOW);
      break;
    case 43:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, LOW); digitalWrite(S2_0, HIGH);
      break;
    case 44:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, LOW);
      break;
    case 45:
      digitalWrite(S0_3, HIGH); digitalWrite(S0_2, HIGH); digitalWrite(S0_1, HIGH); digitalWrite(S0_0, HIGH);
      digitalWrite(S1_3, HIGH); digitalWrite(S1_2, HIGH); digitalWrite(S1_1, HIGH); digitalWrite(S1_0, HIGH);
      digitalWrite(S2_3, HIGH); digitalWrite(S2_2, HIGH); digitalWrite(S2_1, HIGH); digitalWrite(S2_0, HIGH);
      break;
  }
  return state;
}

void Column_switch()
{
  sensor++;
  if (sensor == Column)
  {
    sensor = 0;
  }
}

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< SPI functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


int LC_send(byte Register_ADC, byte command)
{
  unsigned int bit_command = 0;
  byte Regconf = B11000000;
  // Serial.println(Register_ADC,BIN);
  Register_ADC=Register_ADC<<1;
  Register_ADC= Register_ADC | Regconf;
  // Serial.println(Register_ADC,BIN);

  bit_command = (bit_command | Register_ADC) << 8;

  bit_command = bit_command | command;
  // Serial.println(bit_command,BIN);
  digitalWrite(OE2, LOW);
  SPI.transfer16(bit_command);
  digitalWrite(OE2, HIGH); 
  return  bit_command;

}

int SPI_LC_send(byte address, byte command)
{
  unsigned int bit_command = 0;
  byte Regconf = B11000000;
  bit_command = (bit_command | address) << 8;

  bit_command = bit_command | command;
  // Serial.println(bit_command,HEX);
  digitalWrite(OE2, LOW);
  SPI.transfer16(bit_command);
  digitalWrite(OE2, HIGH); 

  return  bit_command;

}

int LC_dataRead()
{
  unsigned int bit_command = 0;
  byte bit_command2 = B11001001;
  // Register_ADC=Register_ADC<<1;
  // Register_ADC= Register_ADC | Regconf;
  // Serial.println(Register_ADC,BIN);

  // bit_command = (bit_command | Regconf) << 8;
  // bit_command = bit_command | command;
  Serial.println(bit_command,BIN);
  digitalWrite(OE2, LOW);
  SPI.transfer(bit_command2);
  bit_command=SPI.transfer16(bit_command);
  digitalWrite(OE2, HIGH); 
  return  bit_command;

}

int LC_regRead(byte Register_ADC)
{
  int RegRead=0;
  unsigned int bit_command = 0;
  byte Regconf = 0b11000001;
  Register_ADC=Register_ADC<<1;
  Register_ADC=Regconf | Register_ADC;
  bit_command = (bit_command | Register_ADC) << 8;
  
  // bit_command = bit_command | Regconf;
  Serial.println(bit_command, BIN);
  digitalWrite(OE2, LOW);
  RegRead=SPI.transfer16(bit_command);
  digitalWrite(OE2, HIGH); 
  return RegRead;
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
  Serial.print("Register read :");
  Serial.print(command, HEX);
  Serial.print(" ");
  Serial.println(command, BIN);

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

void LEDPWR(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
   
    Regconf = B00010000;
    EX_PORT0 = EX_PORT0 | Regconf;
  } else if (!state)
  {
    
    Regconf = B11101111;
    EX_PORT0 = EX_PORT0 & Regconf;
  }

  SPI_IO_expander_send(0x02, EX_PORT0);
}

void LEDR(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    // Regconf = B00000001;
    Regconf = B00000010;
    EX_PORT1 = EX_PORT1 | Regconf;
  } else if (!state)
  {
    //Regconf = B11111110;
    Regconf = B11111101;
    EX_PORT1 = EX_PORT1 & Regconf;
  }

  SPI_IO_expander_send(0x03, EX_PORT1);
}

void LEDG(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    //Regconf = B00000010;
    Regconf = B00000001;
    EX_PORT1 = EX_PORT1 | Regconf;
  } else if (!state)
  {
    //Regconf = B11111101;
    Regconf = B11111110;
    EX_PORT1 = EX_PORT1 & Regconf;
  }

  SPI_IO_expander_send(0x03, EX_PORT1);
}


void IO_reg_init()
{

  SPI_IO_expander_send(0x06, 0x00);
  SPI_IO_expander_send(0x02, EX_PORT0);
  SPI_IO_expander_send(0x07, 0x00);
  SPI_IO_expander_send(0x03, EX_PORT1);
}

// unsigned int A2D_sampling()
// {
//   digitalWrite(A2D_nCS, LOW);
//   //delay(0.5);
//   //   AtoD_PinSet(LOW);
//   A2D_VAL = SPI.transfer16(0);
//   digitalWrite(A2D_nCS, HIGH);
//   return A2D_VAL;
// }

int A2D1_sampling()
{
  byte Regconf = B01111111;

  EX_PORT1 = EX_PORT1 & Regconf;
  SPI_IO_expander_send(0x03, EX_PORT1);

  A2D_VAL = SPI.transfer16(0);

  Regconf = B10000000;
  EX_PORT1 = EX_PORT1 | Regconf;
  SPI_IO_expander_send(0x03, EX_PORT1);

  return A2D_VAL;
}

int A2D2_sampling()
{
  byte Regconf = B11011111;

  EX_PORT1 = EX_PORT1 & Regconf;
  SPI_IO_expander_send(0x03, EX_PORT1);

  A2D_VAL = SPI.transfer16(0);

  Regconf = B00100000;
  EX_PORT1 = EX_PORT1 | Regconf;
  SPI_IO_expander_send(0x03, EX_PORT1);

  return A2D_VAL;
}

int A2D3_sampling()
{
  byte Regconf = B10111111;

  EX_PORT1 = EX_PORT1 & Regconf;
  SPI_IO_expander_send(0x03, EX_PORT1);

  A2D_VAL = SPI.transfer16(0);

  Regconf = B01000000;
  EX_PORT1 = EX_PORT1 | Regconf;
  SPI_IO_expander_send(0x03, EX_PORT1);

  return A2D_VAL;
}

void IOregRead()
{
  IO_exp_read(0x00);
  IO_exp_read(0x01);
}



void SR_nOE(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00000001;
    EX_PORT0 = EX_PORT0 | Regconf;
  } else if (!state)
  {
    Regconf = B11111110;
    EX_PORT0 = EX_PORT0 & Regconf;
  }

  SPI_IO_expander_send(0x02, EX_PORT0);
}

void SR_CLK(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00000010;
    EX_PORT0 = EX_PORT0 | Regconf;
  } else if (!state)
  {
    Regconf = B11111101;
    EX_PORT0 = EX_PORT0 & Regconf;
  }

  SPI_IO_expander_send(0x02, EX_PORT0);
}

void SR_CLK2(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00001000;
    EX_PORT1 = EX_PORT1 | Regconf;
  } else if (!state)
  {
    Regconf = B11110111;
    EX_PORT1 = EX_PORT1 & Regconf;
  }

  SPI_IO_expander_send(0x03, EX_PORT1);
}


void SR_LATCH(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00001000;
    EX_PORT0 = EX_PORT0 | Regconf;
  } else if (!state)
  {
    Regconf = B11110111;
    EX_PORT0 = EX_PORT0 & Regconf;
  }

  SPI_IO_expander_send(0x02, EX_PORT0);
}

void SR_LATCH2(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00010000;
    EX_PORT1 = EX_PORT1 | Regconf;
  } else if (!state)
  {
    Regconf = B11101111;
    EX_PORT1 = EX_PORT1 & Regconf;
  }

  SPI_IO_expander_send(0x03, EX_PORT1);
}

void SER(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00000100;
    EX_PORT0 = EX_PORT0 | Regconf;
  } else if (!state)
  {
    Regconf = B11111011;
    EX_PORT0 = EX_PORT0 & Regconf;
  }

  SPI_IO_expander_send(0x02, EX_PORT0);
}

void SER2(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00000100;
    EX_PORT1 = EX_PORT1 | Regconf;
  } else if (!state)
  {
    Regconf = B11111011;
    EX_PORT1 = EX_PORT1 & Regconf;
  }
  SPI_IO_expander_send(0x03, EX_PORT1);
}


void nSRCLR2(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00010000;
    EX_PORT0 = EX_PORT0 | Regconf;
  } else if (!state)
  {
    Regconf = B11101111;
    EX_PORT0 = EX_PORT0 & Regconf;
  }

  SPI_IO_expander_send(0x02, EX_PORT0);
}


void FET_switch_init()
{
SER2(true);

for (int i=0; i=Raw; i++)
{
  SR_CLK2(HIGH);
  SR_CLK2(LOW);
}
  SR_LATCH2(HIGH);
  SR_LATCH2(LOW);

}

void FET_Switch(int RawCount)
{
if (RawCount==0)
{
  SER2(false);
} else {SER2(true);}

  SR_CLK2(HIGH);
  SR_CLK2(LOW);
  SR_LATCH2(HIGH);
  SR_LATCH2(LOW);
}



void nSRCLR(bool state)
{
  byte Regconf = B00000000;
  if (state)
  {
    Regconf = B00100000;
    EX_PORT0 = EX_PORT0 | Regconf;
  } else if (!state)
  {
    Regconf = B11011111;
    EX_PORT0 = EX_PORT0 & Regconf;
  }

  SPI_IO_expander_send(0x02, EX_PORT0);
}


void Y_Axis_SR_set2high(int Raw_reset)
{
  for (int y = 0; y < Raw_reset; y++)
  {
    SER(HIGH);
    SR_CLK(HIGH);
    SR_CLK(LOW);
    SR_LATCH(HIGH);
    SR_LATCH(LOW);
  }
}

//void FET_SR_control_shift(int Smpl_Trace)
//{
//  //   bool hi=LOW;
//  //  Smpl_Trace = 45 - (Smpl_Trace);
//  if (Smpl_Trace == 0)
//  {
//    //      Serial.println(Smpl_Trace);
//    digitalWrite(FET_Shift_Data, LOW);
//    //   hi=LOW;
//  } else {
//    //   hi=HIGH;
//    digitalWrite(FET_Shift_Data, HIGH);
//  }
//  //Serial.println(hi);
//  digitalWrite(FET_Shift_Clk, HIGH);
//  digitalWrite(FET_Shift_Clk, LOW);
//  SR_LATCH(HIGH);
//  digitalWrite(FET_Shift_Clk, HIGH);
//  digitalWrite(FET_Shift_Clk, LOW);
//  SR_LATCH(LOW);
//  //  digitalWrite(Shift_Latch, HIGH);
//  //  digitalWrite(Shift_Latch, LOW);
//
//
//  //  delay(0.5);
//}
//void FET_SR_control_HIGH_set()
//{
//  digitalWrite(FET_shift_OE, HIGH);
//  nSRCLR2(false);
//  nSRCLR2(true);
//  // digitalWrite(FET_shift_clear, LOW);
//  // digitalWrite(FET_shift_clear, HIGH);
//  SR_LATCH(true);
//  SR_LATCH(false);
//  //  digitalWrite(Shift_Latch, HIGH);
//  //  digitalWrite(Shift_Latch, LOW);
//  digitalWrite(FET_Shift_Data, HIGH);
//
//  for (int i = 0; i < 45; i++)
//  {
//    digitalWrite(FET_Shift_Clk, HIGH);
//    digitalWrite(FET_Shift_Clk, LOW);
//  }
//  SR_LATCH(true);
//  SR_LATCH(false);
//  // digitalWrite(Shift_Latch, HIGH);
//  // digitalWrite(Shift_Latch, LOW);
//
//}


//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< Wireless functions >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


//*************************************************************************************************************************************
//--------------------------------------------------------Configuration request from server"-------------------------------------------
//*************************************************************************************************************************************


void wifi_power_set(int wifi_power)
{
  switch (wifi_power)
  {
    case 0:
      WiFi.setTxPower(WIFI_POWER_MINUS_1dBm);// -4
      break;
    case 1:
      WiFi.setTxPower(WIFI_POWER_2dBm);  // 8
      break;
    case 2:
      WiFi.setTxPower(WIFI_POWER_5dBm);// 20
      break;
    case 3:
      WiFi.setTxPower(WIFI_POWER_7dBm);// 28
      break;
    case 4:
      WiFi.setTxPower(WIFI_POWER_8_5dBm); //34
      break;
    case 5:
      WiFi.setTxPower(WIFI_POWER_11dBm);  //44
      break;
    case 6:
      WiFi.setTxPower(WIFI_POWER_13dBm);  //52
      break;
    case 7:
      WiFi.setTxPower(WIFI_POWER_15dBm);  //60
      break;
    case 8:
      WiFi.setTxPower(WIFI_POWER_17dBm);  //68
      break;
    case 9:
      WiFi.setTxPower(WIFI_POWER_18_5dBm);  //74
      break;
    case 10:
      WiFi.setTxPower(WIFI_POWER_19dBm);  //76
      break;
    case 11:
      WiFi.setTxPower(WIFI_POWER_19_5dBm);  //78
      break;

  }
}
void ServConfigPing()
{

  String response;
  String PrimerConfURL;
  String SecondConfURL;
  String UDP_address;
  String UDP_port;
  String WebWifiPower;
  int WifiPower = 0;
  String UDP_active_toggle;

  String Config_URL = FirstConfigURL + String(ID_name.c_str()) + "/config";
  // String Config_URL = "https://wa-bl-server.herokuapp.com/api/sensor/" + String(ID_name.c_str()) + "/config";
  String Config_ACK_URL = "-ack";
  String WebDetectionTH;
  bool config_response = false;

  StaticJsonDocument<200> doc;

  HTTPClient http;
  http.begin(Config_URL);
  //Serial.println(Config_URL);

  int httpResponseCode = http.GET();
  if (httpResponseCode > 0)
  {
    response = http.getString();
    Serial.println(response );

    char charBuf[response.length() + 1];
    response.toCharArray(charBuf, response.length() + 1);

    if (serverfeedback) {
      Serial.println(httpResponseCode);
      Serial.println(charBuf);
    }

    //DynamicJsonDocument doc(2048);
    //deserializeJson(doc,charBuf);
    DeserializationError error = deserializeJson(doc, charBuf);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }


        if ((doc["config"]["DynComp"].as<String>()) == "true") {
        config_response = true;
        dyncomp=true;
        EEPROM.write(291, dyncomp);
        EEPROM.commit();
        Serial.println (String("Dynamic Compensation set to: ") + dyncomp);

    }
        if ((doc["config"]["DynComp"].as<String>()) == "true") {
        config_response = false;
        dyncomp=false;
        EEPROM.write(291, dyncomp);
        EEPROM.commit();
        Serial.println (String("Dynamic Compensation set to: ") + dyncomp);

    }

    if ((UDP_address = doc["config"]["udpAddress"].as<String>()) != "null") {
      config_response = true;
      UDP_address.toCharArray(UDP_TX_URL, 30);  // convert string to char array
      addr = 200;
      for (int y = addr; y < addr + 30; y++)
      {
        if (UDP_TX_URL[y - addr] != EEPROM.read(y))
        {
          for (int i = 0; i < 30; i++) {
            EEPROM.write(addr + i, UDP_TX_URL[i]);
            EEPROM.commit();

          }
          udpAddress = UDP_TX_URL;
          Serial.println (String("UDP Tx Address set to: ") + udpAddress);
          break;
        }
      }
    }


    if ((UDP_port = doc["config"]["udpPort"].as<String>()) != "null") {
      config_response = true;
      UDP_port.toCharArray(UDP_TX_Port, 5);
      addr = 230;
      for (int y = addr; y < addr + 5; y++)
      {
        if (UDP_TX_Port[y - addr] != EEPROM.read(y))
        {
          for (int i = 0; i < 5; i++) {
            EEPROM.write(addr + i, UDP_TX_Port[i]);
            EEPROM.commit();

            //     Serial.println(udpPortLocation);
            //     Serial.println(response.substring((udpPortLocation + 10), response.indexOf("\"", udpPortLocation + 10)));
          }

          udpPort = UDP_port.toInt();
          Serial.println (String("UDP Tx Port set to: ") + udpPort);

          break;
        }
      }
    }
    if ((doc["config"]["sendSignal"].as<String>()) == "true") {
      config_response = true;
      Tx_flag = true;
      TxChangeFlag = true;
    }

    if ((doc["config"]["TxAlways"].as<String>()) == "true") {
      config_response = true;
      TxAlways = true;

    }
    if ((doc["config"]["TxAlways"].as<String>()) == "false") {
      config_response = true;
      TxAlways = false;

    }
    if ((WebWifiPower = doc["config"]["wifiPower"].as<String>()) != "null") {
      config_response = true;
      WifiPower = WebWifiPower.toInt();
      if (WifiPower >= 0 && WifiPower <= 12 && WifiPower != EEPROM.read(23)) {
        if (WifiPower != EEPROM.read(23))
        {
          if (WifiPower >= 0 && WifiPower <= 12 ) {
            wifi_power_set(WifiPower);
            EEPROM.write(23, WifiPower);
            EEPROM.commit();
            Serial.println (String("Wifi power set to: ") + EEPROM.read(23));
          }
        }

      }
    }

    if ((WebDetectionTH = doc["config"]["detectionTh"].as<String>()) != "null") {
      config_response = true;
      ChangeDetectTH = WebDetectionTH.toInt();
      if (ChangeDetectTH != EEPROM.read(295)) {
        EEPROM.write(295, ChangeDetectTH);
        EEPROM.commit();
        ChangeDetectTH = EEPROM.read(295);
        Serial.println (String("Detection threshold set to: ") + ChangeDetectTH);
      }
    }

    if ((UDP_active_toggle = doc["config"]["udpActive"].as<String>()) != "null") {
      config_response = true;
      Serial.println (String("UDP Tx Active was to: ") + HTTP_flag);
      HTTP_flag = UDP_active_toggle.toInt();
      Serial.println (String("UDP Tx Active was to: ") + HTTP_flag);
      if (HTTP_flag != EEPROM.read(294)) {
        EEPROM.write(294, HTTP_flag);
        EEPROM.commit();
        HTTP_flag = EEPROM.read(294);
        Serial.println (String("UDP Tx Active set to: ") + HTTP_flag);
      }
    }

    if ((PrimerConfURL = doc["config"]["primaryHttpUrl"].as<String>()) != "null") {
      config_response = true;
      PrimerConfURL.toCharArray(FirstConfigURL, 100);  // convert string to char array
      addr = 300;
      for (int y = addr; y < addr + 100; y++)
      {
        if (FirstConfigURL[y - addr] != EEPROM.read(y))
        {
          for (int i = 0; i < 100; i++) {
            EEPROM.write(addr + i, FirstConfigURL[i]);
            EEPROM.commit();

          }
          Serial.println (String("Primer server config Address set to: ") + PrimerConfURL);
          break;
        }
      }
    }

    if ((SecondConfURL = doc["config"]["secondaryHttpUrl"].as<String>()) != "null") {
      config_response = true;
      SecondConfURL.toCharArray(SecondConfigURL, 100);  // convert string to char array
      addr = 400;
      for (int y = addr; y < addr + 100; y++)
      {
        if (SecondConfigURL[y - addr] != EEPROM.read(y))
        {
          for (int i = 0; i < 100; i++) {
            EEPROM.write(addr + i, SecondConfigURL[i]);
            EEPROM.commit();
          }
          Serial.println (String("Secondary server config Address set to: ") + SecondConfigURL);
          break;
        }
      }
    }


    //    UDP_active_toggle = doc["config"]["udpActive"].as<bool>();
    //    PrimerConfigURL = doc["config"]["primaryHttpUrl"].as<String>();
    //    SecondConfigURL = doc["config"]["secondaryHttpUrl"].as<String>();
    //    detectionTH = doc["config"]["detectionTh"].as<int>();


    //    Serial.println(UDP_address);
    //    Serial.println(UDP_port);
    //    Serial.println(WifiPower);
    //    Serial.println(UDP_active_toggle);
    //    Serial.println(PrimerConfigURL);
    //    Serial.println(detectionTH);

    if (config_response)
    {
      Config_URL += Config_ACK_URL;
      HTTPClient http;
      //Serial.println(Config_URL);
      http.begin(Config_URL);
      httpResponseCode = http.POST("");
      Serial.println(httpResponseCode);
      response = http.getString();
      config_response = false;
    }

  }

  ServConfigPingInterval2 = millis();
}



void ServConfigPost(bool UdpAddressPost, bool UdpPortPost, bool UdpActivePost, bool WifiPowerPost, bool ChDetectTH_Post, bool PrimeServPost, bool SecondServPost, bool DynCompPost) {

  if (UdpAddressPost || UdpPortPost || UdpActivePost || WifiPowerPost || ChDetectTH_Post || PrimeServPost || SecondServPost || DynCompPost) {


    String ServerPostMessage;
    StaticJsonDocument<200> doc;

    if (DynCompPost) {
      doc["servUpdate"]["DynComp"] = String(dyncomp);
      dyncompFlag=false; 
    }

    if (UdpAddressPost) {
      doc["servUpdate"]["udpAddress"] = String(udpAddress);
      udpAddresFlag = false;
    }

    if (UdpPortPost) {
      doc["servUpdate"]["udpPort"] = String(udpPort);
      udpPortFlag = false;
    }

    if (UdpActivePost) {
      doc["servUpdate"]["udpActive"] = HTTP_flag;
      updActiveFlag = false;
    }

    if (WifiPowerPost) {
      doc["servUpdate"]["wifiPower"] = String(EEPROM.read(23));
      wifiFlag = false;
    }

    if (ChDetectTH_Post) {
      doc["servUpdate"]["detectionTh"] = String(ChangeDetectTH);
      SrvChangeFlag = false;
    }

    if (PrimeServPost) {
      doc["servUpdate"]["primaryHttpUrl"] = String(FirstConfigURL);
      PrimeServPostFlag = false;
    }

    if (SecondServPost) {
      doc["servUpdate"]["secondaryHttpUrl"] = String(SecondConfigURL);
      SecondServPostFlag = false;
    }

    serializeJson(doc, ServerPostMessage);
    //  Serial.println( serializeJsonPretty(doc, ServerPostMessage));
    Serial.println(ServerPostMessage);


    WiFiClient client;
    HTTPClient http;
    String PostURL = String(FirstConfigURL) + String(ID_name.c_str()) + String( "/config-ack");
    http.begin(PostURL);
    //http.begin("https://webhook.site/14d50c94-dc58-42f9-891d-5603d0b41669");
    http.addHeader("Content-Type", "application/json");
    //Serial.println(PostURL);
    int httpResponseCode = http.POST(ServerPostMessage);
    // int httpResponseCode = http.POST(PostURL);
    if (httpResponseCode > 0) {
      String response = http.getString();
      // Serial.println(httpResponseCode);
      // Serial.println(response);
    }
  }
}

void Tx_state_mem(bool TX_flag_mem) {

  EEPROM.write(294, TX_flag_mem);
  EEPROM.commit();
  Serial.println(" TX state set to: " + String(TX_flag_mem));


}



void handleForm() {
  String web_column = server.arg("column");
  String web_column_fctr = server.arg("columnfctr");
  String web_raw = server.arg("raw");
  String web_raw_fctr = server.arg("rawfctr");
  String web_full_column = server.arg("fcolumn");
  String web_full_column_fctr = server.arg("fcolumnfctr");
  String web_full_raw = server.arg("fraw");
  String web_full_raw_fctr = server.arg("frawfctr");
  String web_raw_offset = server.arg("roffset");
  String web_raw_offset_fctr = server.arg("roffsetfctr");
  String web_column_offset = server.arg("coffset");
  String web_column_offset_fctr = server.arg("coffsetfctr");
  String web_http_fltr = server.arg("httpfilter");
  String web_http_fltr_fctr = server.arg("httpfilterfctr");
  String web_scan_fltr = server.arg("scanfilter");
  String web_scan_fltr_fctr = server.arg("scanfilterfctr");
  String web_ChangeDetectTH = server.arg("ChangeDetectionTH");
  String web_ChangeDetectTH_fctr = server.arg("ChangeDetectionTHfctr");
  String web_system_ID = server.arg("systemid");
  String web_system_ID_fctr = server.arg("systemidfctr");
  String web_wifi_pwr = server.arg("wifipwr");
  String web_wifi_pwr_fctr = server.arg("wifipwrfctr");
  String web_scanmode = server.arg("Scanmode");
  String web_scanmode_fctr = server.arg("Scanmodefctr");
  String web_net_name = server.arg("netname");
  String web_net_pswrd = server.arg("netpswrd");
  String web_net_name_fctr = server.arg("netnamefctr");
  String web_net_pswrd_fctr = server.arg("netpswrdfctr");
  String web_TX_URL = server.arg("TransmittingURL");
  String web_TX_PORT = server.arg("TransmittingPort");
  String web_Scan_Time_interval = server.arg("ScanTimeInterval");
  String web_ScanState = server.arg("ScanState");
  String web_samp_offset = server.arg("samp_offset");
  String web_samp_offsetfctr = server.arg("samp_offsetfctr");
  String web_Scan_log = server.arg("Scan_log");
  String web_TxState = server.arg("TxState");
  String web_PrimaryServerConfig = server.arg("PrimaryServerConfig");
  String web_SecondaryServerConfig = server.arg("SecondaryServerConfig");
  String web_MeshEventTH = server.arg("MeshEventTH");
  String web_ValueDetectionTH = server.arg("ValueDetectionTH");
  String web_ValueDetectionTHfctr = server.arg("ValueDetectionTHfctr");
  String web_PixelMarkState = server.arg("PixelMarkState");
  String web_DynamicCompensation=server.arg("DynamicCompensation");


if ((web_DynamicCompensation == "on") && !dyncomp)
{
  dyncomp=true;
    EEPROM.write(291, dyncomp);
    EEPROM.commit();
    dyncomp = EEPROM.read(291);
    Serial.println (String("Dynamic Compensation set to: ") + dyncomp);
    web_last_config = HIGH;
    dyncompFlag=true;
    
  
} else if ((web_DynamicCompensation != "on") && dyncomp) 
{
  dyncomp=false;
    EEPROM.write(291, dyncomp);
    EEPROM.commit();
    dyncomp = EEPROM.read(291);
    Serial.println (String("Dynamic Compensation set to: ") + dyncomp);
    web_last_config = HIGH;
    dyncompFlag=true;
}


  if ((web_PixelMarkState == "on") && !PixelMarkState)
  {
    PixelMarkState = true;
    EEPROM.write(250, PixelMarkState);
    EEPROM.commit();
    PixelMarkState = EEPROM.read(250);
    Serial.println (String("Pixel Mark A/R enable set to: ") + PixelMarkState);
    web_last_config = HIGH;
  } else if ((web_PixelMarkState != "on") && PixelMarkState) {
    PixelMarkState = false;
    EEPROM.write(250, PixelMarkState);
    EEPROM.commit();
    PixelMarkState = EEPROM.read(250);
    web_last_config = HIGH;
    Serial.println (String("Pixel Mark A/R enable set to: ") + PixelMarkState);
  }



  if (web_ValueDetectionTH.toInt() != ValueDetectTH) {
    ValueDetectTH = web_ValueDetectionTH.toInt();

    EEPROM.write(242, ValueDetectTH >> 8);
    EEPROM.commit();
    EEPROM.write(243, ValueDetectTH);
    EEPROM.commit();

    ValueDetectTH = (EEPROM.read(242) << 8) + EEPROM.read(243);
    Serial.println (String("Item detect value set to: ") + ValueDetectTH);
    web_last_config = HIGH;
  }

  if (web_ValueDetectionTHfctr.toInt() != ValueDetectTHfctr) {
    ValueDetectTHfctr = web_ValueDetectionTHfctr.toInt();
    EEPROM.write(244, ValueDetectTHfctr >> 8);
    EEPROM.commit();
    EEPROM.write(245, ValueDetectTHfctr);
    EEPROM.commit();
    ValueDetectTHfctr = (EEPROM.read(244) << 8) + EEPROM.read(245);
    Serial.println (String("Item detect value factory set to: ") + ValueDetectTHfctr);
    web_last_config = HIGH;
  }
  if (web_MeshEventTH.toInt() != BufferEventTH) {
    EEPROM.write(293, web_MeshEventTH.toInt());
    EEPROM.commit();
    BufferEventTH = EEPROM.read(293);
    Serial.println (String("Mesh buffer event TH set to: ") + BufferEventTH);
    web_last_config = HIGH;
  }
  Scan_Time_Delay_usec = web_Scan_Time_interval.toInt();
  char Time_interval_usec[6] = {};
  web_Scan_Time_interval.toCharArray(Time_interval_usec, 6);  // convert string to char array
  addr = 235;
  for (int y = addr; y < addr + 6; y++)
  {
    if (Time_interval_usec[y - addr] != EEPROM.read(y))
    {
      for (int i = 0; i < 6; i++) {
        EEPROM.write(addr + i, Time_interval_usec[i]);
        EEPROM.commit();

      }
      Serial.println (String("Scan time interval set to: ") + Scan_Time_Delay_usec);
      web_last_config = HIGH;
      break;
    }
  }

  if (web_samp_offset.toInt() != sampling_offset) {

    sampling_offset = web_samp_offset.toInt();
    EEPROM.write(246, sampling_offset >> 8);
    EEPROM.commit();
    EEPROM.write(247, sampling_offset);
    EEPROM.commit();

    sampling_offset = (EEPROM.read(246) << 8) + EEPROM.read(247);
    Serial.println (String("Sampling offset set to: ") + sampling_offset);
    web_last_config = HIGH;
  }

  if (web_samp_offsetfctr.toInt() != EEPROM.read(298)) {

    int sampling_offset_fctr = web_samp_offsetfctr.toInt();
    EEPROM.write(248, sampling_offset_fctr >> 8);
    EEPROM.commit();
    EEPROM.write(249, sampling_offset_fctr);
    EEPROM.commit();
    sampling_offset_fctr = (EEPROM.read(248) << 8) + EEPROM.read(249);
    Serial.println (String("Sampling factory offset set to: ") + sampling_offset_fctr);
    web_last_config = HIGH;
  }



  web_TX_URL.toCharArray(UDP_TX_URL, 30);  // convert string to char array
  addr = 200;
  for (int y = addr; y < addr + 30; y++)
  {
    if (UDP_TX_URL[y - addr] != EEPROM.read(y))
    {
      for (int i = 0; i < 30; i++) {
        EEPROM.write(addr + i, UDP_TX_URL[i]);
        EEPROM.commit();

      }
      udpAddress = UDP_TX_URL;
      Serial.println (String("UDP Tx Address set to: ") + udpAddress);
      web_last_config = HIGH;
      udpAddresFlag = true;
      break;
    }
  }

  web_TX_PORT.toCharArray(UDP_TX_Port, 5);  // convert string to char array
  //     Serial.println(web_TX_PORT);

  addr = 230;
  for (int y = addr; y < addr + 5; y++)
  {
    if (UDP_TX_Port[y - addr] != EEPROM.read(y))
    {

      for (int i = 0; i < 5; i++) {
        EEPROM.write(addr + i, UDP_TX_Port[i]);
        EEPROM.commit();

      }

      unsigned int UDP_port = web_TX_PORT.toInt();
      udpPort = UDP_port;
      Serial.println (String("UDP Tx Port set to: ") + udpPort);
      web_last_config = HIGH;
      udpPortFlag = true;
      break;
    }
  }


  web_net_name.toCharArray(ssid_reconnect, 25);  // convert string to char array

  addr = 55;
  for (int y = addr; y < addr + 25; y++)
  {
    if (ssid_reconnect[y - addr] != EEPROM.read(y))
    {
      for (int i = 0; i < 25; i++) {
        EEPROM.write(addr + i, ssid_reconnect[i]);
        EEPROM.commit();
      }
      Serial.println (String("SSID name set to: ") + ssid_reconnect);
      web_last_config = HIGH;
      break;
    }
  }

  web_net_pswrd.toCharArray(password_reconnect, 20); // convert string to char array

  addr = 80;
  for (int y = addr; y < addr + 20; y++)
  {
    if (password_reconnect[y - addr] != EEPROM.read(y))
    {
      for (int i = 0; i < 20; i++) {
        EEPROM.write(addr + i, password_reconnect[i]);
        EEPROM.commit();
      }
      Serial.println (String("SSID password set to: ") + password_reconnect);
      web_last_config = HIGH;
      break;
    }
  }
  /*
    web_net_name_fctr.toCharArray(ssid_reconnect_fctr, 20);  // convert string to char array

    addr = 150;
    for (int y = addr; y < addr + 20; y++)
    {
      if (ssid_reconnect_fctr[y - addr] != EEPROM.read(y))
      {
        for (int i = 0; i < 20; i++) {
          EEPROM.write(addr + i, ssid_reconnect_fctr[i]);
          EEPROM.commit();
        }
        Serial.println (String("Factory SSID name set to: ") + ssid_reconnect_fctr);
        web_last_config = HIGH;
        break;
      }
    }
  */
  /*
    web_net_pswrd_fctr.toCharArray(password_reconnect_fctr, 20); // convert string to char array

    addr = 170;
    for (int y = addr; y < addr + 20; y++)
    {
      if (password_reconnect_fctr[y - addr] != EEPROM.read(y))
      {
        for (int i = 0; i < 20; i++) {
          EEPROM.write(addr + i, password_reconnect_fctr[i]);
          EEPROM.commit();
        }
        Serial.println (String("Factory SSID password set to: ") + password_reconnect_fctr);
        web_last_config = HIGH;
        break;
      }
    }
  */
  if (web_scanmode.toInt() != EEPROM.read(12)) {
    EEPROM.write(12, web_scanmode.toInt());
    EEPROM.commit();
    scan_mode = EEPROM.read(12);
    Serial.println(String("Scan mode set to: ") + scan_mode);
    web_last_config = HIGH;
  }


  if (web_scanmode_fctr.toInt() != EEPROM.read(51)) {
    EEPROM.write(51, web_scanmode_fctr.toInt());
    EEPROM.commit();
    Serial.println (String("Scan mode factory set to: ") + EEPROM.read(51));
    web_last_config = HIGH;
  }

  if (web_wifi_pwr.toInt() != EEPROM.read(23)) {
    int power_read = web_wifi_pwr.toInt();
    if (power_read >= 0 && power_read <= 12) {
      wifi_power_set(power_read);
      EEPROM.write(23, power_read);
      EEPROM.commit();
      Serial.println (String("Wifi power set to: ") + EEPROM.read(23));
      web_last_config = HIGH;
      wifiFlag = true;
    }
  }

  if (web_wifi_pwr_fctr.toInt() != EEPROM.read(50)) {
    EEPROM.write(50, web_wifi_pwr_fctr.toInt());
    EEPROM.commit();
    Serial.println (String("Wifi power factory set to: ") + EEPROM.read(50));
    web_last_config = HIGH;
  }


  if (web_system_ID.toInt() != UniqueID) {

    EEPROM.write(22, web_system_ID.toInt());
    EEPROM.commit();
    UniqueID = EEPROM.read(22);
    Serial.println (String("System ID set to: ") + EEPROM.read(22));
    web_last_config = HIGH;
  }

  if (web_system_ID_fctr.toInt() != EEPROM.read(49)) {

    EEPROM.write(49, web_system_ID_fctr.toInt());
    EEPROM.commit();
    Serial.println (String("System ID factory set to: ") + EEPROM.read(49));
    web_last_config = HIGH;
  }

  if (web_ChangeDetectTH.toInt() != ChangeDetectTH ) {

    EEPROM.write(295, web_ChangeDetectTH.toInt());
    EEPROM.commit();
    ChangeDetectTH = EEPROM.read(295);
    Serial.println (String("Change Detect Threshold set to: ") + EEPROM.read(295));
    web_last_config = HIGH;
    SrvChangeFlag = true;

  }

  if (web_ChangeDetectTH_fctr.toInt() != EEPROM.read(296)) {

    EEPROM.write(296, web_ChangeDetectTH_fctr.toInt());
    EEPROM.commit();
    Serial.println (String("Change Detect Threshold factory set to: ") + EEPROM.read(296));
  }

  if (web_scan_fltr.toInt() != Filter_scan ) {

    int fltrsc = web_scan_fltr .toInt();
    if (fltrsc > Filter_tx) {
      fltrsc = Filter_tx;
      Serial.println();
      Serial.println("Scan filter is above Transmit filter and will be equilized");
    }
    if (fltrsc <= 1500 && fltrsc >= 0) {
      fltrsc /= 50;
    } else {
      fltrsc = 0;
    }
    EEPROM.write(9, fltrsc);
    EEPROM.commit();
    Filter_scan = (EEPROM.read(9) * 50);
    Serial.println (String("Scan Filter set to: ") + (EEPROM.read(9) * 50));
    web_last_config = HIGH;
  }

  if ((web_scan_fltr_fctr.toInt() / 50) != EEPROM.read(46)) {
    int fltrsc2 = web_scan_fltr_fctr.toInt();
    if (fltrsc2 > Filter_tx) {
      fltrsc2 = Filter_tx;
      Serial.println();
      Serial.println("Scan filter is above Transmit filter and will be equilized");
    }
    if (fltrsc2 <= 1500 && fltrsc2 >= 0) {
      fltrsc2 /= 50;
    } else {
      fltrsc2 = 0;
    }
    EEPROM.write(46, fltrsc2);
    EEPROM.commit();
    Serial.println (String("Scan Filter factory set to: ") + (EEPROM.read(46) * 50));
    web_last_config = HIGH;
  }

  if (web_http_fltr.toInt() != Filter_tx ) {
    int fltrtx = web_http_fltr.toInt();
    if (fltrtx < Filter_scan) {
      fltrtx = Filter_scan;
    }
    if (fltrtx >= 0 && fltrtx < 4095)
    {
      fltrtx /= 50;
      EEPROM.write(10, fltrtx);
      EEPROM.commit();
      Filter_tx = (EEPROM.read(10) * 50);
      Serial.println (String("Http Filter set to: ") + (EEPROM.read(10) * 50));
    }

    web_last_config = HIGH;
  }

  if ((web_http_fltr_fctr.toInt() / 50) != EEPROM.read(47)) {
    int fltrtx2 = web_http_fltr_fctr.toInt();
    if (fltrtx2 < Filter_scan) {
      fltrtx2 = Filter_scan;
    }
    if (fltrtx2 >= 0 && fltrtx2 < 4095)
    {
      fltrtx2 /= 50;
      EEPROM.write(47, fltrtx2);
      EEPROM.commit();
      Serial.println (String("Http Filter factory set to: ") + (EEPROM.read(47) * 50));

    }
    web_last_config = HIGH;
  }
  if (web_raw_offset.toInt() != Raw_start ) {

    EEPROM.write(4, web_raw_offset.toInt());
    EEPROM.commit();
    Raw_start = EEPROM.read(4);
    Raw_start_offset = Raw_start;
    Serial.println (String("Raw offset set to: ") + EEPROM.read(4));
    web_last_config = HIGH;
  }

  if (web_raw_offset_fctr.toInt() != EEPROM.read(42)) {
    EEPROM.write(42, web_raw_offset_fctr.toInt());
    EEPROM.commit();
    Serial.println (String("Raw offset factory set to: ") + EEPROM.read(42));
    web_last_config = HIGH;
  }


  if (web_column_offset.toInt() != Column_start ) {

    EEPROM.write(5, web_column_offset.toInt());
    EEPROM.commit();
    Column_start = EEPROM.read(5);
    Column_start_offset = Column_start;
    x_offset = Column_start;
    Serial.println (String("X offset factory set to: ") + EEPROM.read(5));
    web_last_config = HIGH;
  }
  if (web_column_offset_fctr.toInt() != EEPROM.read(45)) {
    EEPROM.write(45, web_column_offset_fctr.toInt() );
    EEPROM.commit();
    Serial.println (String("X offset factory set to: ") + EEPROM.read(45));
    web_last_config = HIGH;
  }


  if (web_column.toInt() != Column) {
    EEPROM.write(25, web_column.toInt());
    EEPROM.commit();
    Column = EEPROM.read(25);
    Serial.println (String("Column set to: ") + EEPROM.read(25));
    web_last_config = HIGH;
  }

  if (web_column_fctr.toInt() != EEPROM.read(40)) {
    EEPROM.write(40, web_column_fctr.toInt() );
    EEPROM.commit();
    Serial.println (String("Column factory set to: ") + EEPROM.read(40));
    web_last_config = HIGH;
  }

  if (web_raw.toInt() != Raw) {
    EEPROM.write(24, web_raw.toInt());
    EEPROM.commit();
    Raw = EEPROM.read(24);
    Y_Axis_SR_set2high(Raw);
    Serial.println (String("Raw set to: ") + EEPROM.read(24));
    web_last_config = HIGH;
  }

  if (web_raw_fctr.toInt() != EEPROM.read(41)) {
    EEPROM.write(41, web_raw_fctr.toInt() );
    EEPROM.commit();
    Serial.println (String("Raw factory set to: ") + EEPROM.read(41));
    web_last_config = HIGH;
  }

  if (web_full_column.toInt() != Full_mtrx_clmn) {
    EEPROM.write(6, web_full_column.toInt());
    EEPROM.commit();
    Full_mtrx_clmn = EEPROM.read(6);
    Serial.println (String("Full matrix column factory set to: ") + EEPROM.read(6));
    web_last_config = HIGH;
  }
  if (web_full_column_fctr.toInt() != EEPROM.read(43)) {
    EEPROM.write(43, web_full_column_fctr.toInt());
    EEPROM.commit();
    Serial.println (String("Full matrix column factory set to: ") + EEPROM.read(43));
    web_last_config = HIGH;
  }

  if (web_full_raw.toInt() != Full_mtrx_raw) {
    EEPROM.write(21, web_full_raw.toInt());
    EEPROM.commit();
    Full_mtrx_raw = EEPROM.read(21);
    Serial.println (String("Full matrix raw set to: ") + EEPROM.read(21));
    web_last_config = HIGH;
  }
  if (web_full_raw_fctr.toInt() != EEPROM.read(44)) {
    EEPROM.write(44, web_full_raw_fctr.toInt());
    EEPROM.commit();
    Serial.println (String("Full matrix raw factory set to: ") + EEPROM.read(44));
    web_last_config = HIGH;
  }
  if ((web_ScanState == "on") && !FET_flag)
  {
    FET_flag = true;
    Serial.println("filter FET set: HIGH");
    web_last_config = HIGH;
  } else if ((web_ScanState == "on") && FET_flag) {
    FET_flag = false;
    Serial.println("filter FET set: LOW");
    web_last_config = HIGH;
  }


  if ((web_Scan_log == "on") && !log_state )
  {
    log_state = true;
    Serial.println("Logarithmic State set to: HIGH");
    web_last_config = HIGH;
  } else if ((web_Scan_log == "on") && log_state) {
    log_state = false;
    Serial.println("Logarithmic State set to: LOW");
    web_last_config = HIGH;
  }

  if ((web_TxState == "on") && !HTTP_flag)
  {
    HTTP_flag = true;
    Serial.println("HTTP flag set to: HIGH");
    updActiveFlag = true;
    web_last_config = HIGH;
  } else if ((web_TxState != "on") && HTTP_flag) {
    updActiveFlag = true;
    HTTP_flag = false;
    web_last_config = HIGH;
    Serial.println("HTTP flag set to: LOW");
  }

  web_PrimaryServerConfig.toCharArray(FirstConfigURL, 100);
  addr = 300;
  for (int y = addr; y < addr + 100; y++)
  {
    if (FirstConfigURL[y - addr] != EEPROM.read(y))
    {
      for (int i = 0; i < 100; i++) {
        EEPROM.write(addr + i, FirstConfigURL[i]);
        EEPROM.commit();

      }
      PrimeServPostFlag = true;
      web_last_config = HIGH;
      Serial.println (String("Primer server config Address set to: ") + FirstConfigURL);
      break;
    }
  }

  web_SecondaryServerConfig.toCharArray(SecondConfigURL, 100);
  addr = 400;
  for (int y = addr; y < addr + 100; y++)
  {
    if (SecondConfigURL[y - addr] != EEPROM.read(y))
    {
      for (int i = 0; i < 100; i++) {
        EEPROM.write(addr + i, SecondConfigURL[i]);
        EEPROM.commit();
      }
      SecondServPostFlag = true;
      web_last_config = HIGH;
      Serial.println (String("Secondary server config Address set to: ") + SecondConfigURL);
      break;
    }
  }


  if (web_last_config)
  {
    ServConfigPost(udpAddresFlag, udpPortFlag, updActiveFlag, wifiFlag, SrvChangeFlag, PrimeServPostFlag, SecondServPostFlag,dyncompFlag);

    printLocalTime();

  }

}


//*************************************************************************************************************************************
//--------------------------------------------------------Function Scanning network----------------------------------------------------
//*************************************************************************************************************************************

void network_scan() {
  int net = 0;
  int encrpt = 0;
  String encrpt_type;
  if (scan_state == true) {

    // net=WiFi.scanNetworks(false,true);  //boolean WiFi.scanNetworks(Assynchronous scan,Hiden networks)
    net = WiFi.scanNetworks(); // Only Visible network scan
    Serial.println("------------------------------");
    Serial.println(net + String(" Available network(s)"));
    Serial.println("------------------------------");

    delay(200);
    for (int i = 0; i < net; i++) {

      int net_name = i + 1;
      encrpt = WiFi.encryptionType(i);

      switch (encrpt) {

        // WiFi.encryptionType(i) return index (5/2/4/7/8)

        // 5 : ENC_TYPE_WEP - WEP
        // 2 : ENC_TYPE_TKIP - WPA / PSK
        // 4 : ENC_TYPE_CCMP - WPA2 / PSK
        // 7 : ENC_TYPE_NONE - open network
        // 8 : ENC_TYPE_AUTO - WPA / WPA2 / PSK

        case 5:
          encrpt_type = "WEP";
          break;
        case 2:
          encrpt_type = "WPA - PSK";
          break;
        case 4:
          encrpt_type = "WPA2 - PSK";
          break;
        case 7:
          encrpt_type = "OPEN";
          break;
        case 8:
          encrpt_type = "WPA - WPA2 - PSK";
          break;
      }



      Serial.println(net_name + String(". ") + WiFi.SSID(i) + String("                             Ch.") + WiFi.channel(i) + String("  (") + WiFi.RSSI(i) + String(" dBm)   ") + encrpt_type /* + String("  MAC addrs: ") + WiFi.BSSIDstr(i) */ );

      // Serial.printf("%d: %s, Ch:%d (%ddBm) %s\n", i + 1, WiFi.SSID(i).c_str(), WiFi.channel(i), WiFi.RSSI(i), WiFi.encryptionType(i) == ENC_TYPE_NONE ? "open" : "\n"); Serial.println();

      delay(50);
    }


    net_id = false; //  if requred after scanning to connect the network (enter SSID and password) set net_id=true and uncomment network_connect() in the main loop()
    scan_state = false;
  }
  WiFi.scanDelete();   // Delete the last scan result from memory
}

//*************************************************************************************************************************************
//--------------------------------------------------------Function Connection Abort--------------------------------------------------
//*************************************************************************************************************************************

void connection_abort() {


  //disconnect any previous connections
  WiFi.disconnect();
  delay(500);
  HTTP_flag = false;
  LEDG(LOW);
  // digitalWrite(WiFi_LED, LOW);
  Serial.println ("");
  Serial.println ("Connection aborted");

}



//*************************************************************************************************************************************
//--------------------------------------------------------Function HTTP Server set-----------------------------------------------------
//*************************************************************************************************************************************

void HTTP_server_set()
{
  // Start the server
  server.begin();
  Serial.println();
  Serial.println("Server started");



  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://esp32.local
    Serial.println("Error setting up MDNS responder!");
    //  while (1) {
    delay(1000);
    //  }
  }
  Serial.println("mDNS responder started");
  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });



  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");

    server.send(200, "text/html", MatrixConfig(Column, Raw, Full_mtrx_clmn, Full_mtrx_raw, EEPROM.read(23), Raw_start_offset , x_offset, Filter_scan, Filter_tx, UniqueID, scan_mode, FW_Version, web_update_date, ssid_reconnect, password_reconnect/*passwrd_stars*/, UDP_TX_URL, UDP_TX_Port, Scan_Time_Delay_usec , FET_flag, sampling_offset, samp_log, ID_name, HTTP_flag, WiFi.macAddress(), ChangeDetectTH, String(FirstConfigURL), SecondConfigURL, Sampling_Top_steady, SamplingValueDelta, BufferEventTH, ValueDetectTH, ValueDetectTHfctr, PixelMarkState,dyncomp));

  });


  server.on("/action_page", HTTP_GET, []() {
    handleForm();

    server.sendHeader("Connection", "close");

    server.send(200, "text/html", MatrixConfig(Column, Raw, Full_mtrx_clmn, Full_mtrx_raw, EEPROM.read(23), Raw_start_offset , x_offset, Filter_scan, Filter_tx, UniqueID, scan_mode, FW_Version, web_update_date, ssid_reconnect, password_reconnect, UDP_TX_URL, UDP_TX_Port, Scan_Time_Delay_usec, FET_flag, sampling_offset, samp_log, ID_name, HTTP_flag, WiFi.macAddress(), ChangeDetectTH, FirstConfigURL, SecondConfigURL, Sampling_Top_steady, SamplingValueDelta, BufferEventTH, ValueDetectTH, ValueDetectTHfctr, PixelMarkState,dyncomp));
  });
  //  server.on("/action_page", handleForm);



  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });
  server.begin();
}

//*************************************************************************************************************************************
//--------------------------------------------------------Function Network connection--------------------------------------------------
//*************************************************************************************************************************************

void network_connect() {

  char ssid_[25] = {};
  char password_[25] = {};
  int net = 0;
  bool led_state = false;

  Serial.println();
  Serial.println("Please enter the network...");
  Serial.println();
  Serial.println ("Enter a network name:");

  //---------------------------------------Enter network SSID----------------------------------------------

  if (net_id == true) {
    while (net_id == true) {
      if (Serial.available() > 0) {
        delay (100);  // wait to message arrive

        // read all the available characters
        while (Serial.available() > 0) {

          serial_read = Serial.readStringUntil('\n');

        }
        serial_read.toCharArray(ssid_, 25);  // convert string to char array
        serial_read.toCharArray(ssid_reconnect, 25);  // convert string to char array
        ssid = ssid_;

        addr = 55;
        for (int i = 0; i < 25; i++) {
          EEPROM.write(addr + i, ssid_reconnect[i]);
          EEPROM.commit();
        }


        net_id = false;
      }

    }
    String Wifi_ssid;

    net = WiFi.scanNetworks();
    for (int i = 0; i < net; i++) {
      Wifi_ssid = WiFi.SSID(i);


      if (Wifi_ssid == ssid_) {  // check if entered network SSID is a proper network name from the list

        net_id = true;
        WiFi.scanDelete();

      }
    }
    if (net_id == true) {

      Serial.println(String("Network SSID:") + ssid);
      Serial.println ("Enter a password:");

    } else {

      Serial.println("Not available network");
      Serial.println();
      connection_abort();
    }
  }
  //----------------------------------------Enter network password-------------------------------------------
  if (net_id == true) {
    while (net_id == true) {
      if (Serial.available() > 0) {
        delay (100);  // wait to message arrive
        // read all the available characters
        while (Serial.available() > 0) {

          serial_read = Serial.readStringUntil('\n');


        }
        serial_read.toCharArray(password_, 20); // convert string to char array
        serial_read.toCharArray(password_reconnect, 20); // convert string to char array
        net_id = false;
      }
      addr = 80;

      for (int i = 0; i < 20; i++) {
        EEPROM.write(addr + i, password_reconnect[i]);
        EEPROM.commit();
      }


      password = password_;


    }
    net_id = true;
    Serial.print(String("Password:") + password); //Serial.println(password);



    if (  net_id == true) {


      // ---------------------------------------------Connect to WiFi network--------------------------------------


      if (WiFi.isConnected() ) { // check if device is connected to AP

        WiFi.disconnect(); // Sets currently configured SSID and password to null values and disconnects the station from an access point

      }
      Serial.println();
      Serial.println();
      Serial.println(String("Connecting to the network: ") + ssid);


      WiFi.begin(ssid, password);
      int con_attempt = 0;
      while (WiFi.status() != WL_CONNECTED)
      {
        delay(500);
        Serial.print(".");
        led_state = !led_state;
        LEDG(led_state);
        con_attempt++;

        if (WiFi.status() == WL_CONNECT_FAILED || con_attempt == 80 ) {
          Serial.println();
          Serial.println("Failed to connect");
          connection_abort();
          break;
        }
        if (Serial.available() > 0) {

          while (Serial.available() > 0) {


            serial_read = Serial.readStringUntil('\n');
            serial_read.toLowerCase();

          }
          if (serial_read == con_abort) {
            connection_abort();

            break;
          }

        }
      }

      if (WiFi.status() == WL_CONNECTED) {

        WiFi.setAutoReconnect(true); // Set whether module will attempt to reconnect to an access point in case it is disconnected.

        /*
          If parameter autoReconnect is set to true, then module will try to reestablish lost connection to the AP. If set to false then module will stay disconnected.
           Note: running setAutoReconnect(true) when module is already disconnected will not make it reconnect to the access point. Instead reconnect() should be used.
        */
        Serial.println();
        Serial.println("WiFi connected");

        past_connection++;
        LEDG(HIGH);
        //       digitalWrite(WiFi_LED, HIGH);

        HTTP_server_set();

        connection_lost_flag = false;
        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        printLocalTime();

        //        timeClient.begin();
        //        delay(1000);
        //        while (!timeClient.update()) {
        //          timeClient.forceUpdate();
        //
        //          Serial.println( "NTP servere updated");
        //          formattedDate = timeClient.getFormattedDate();wddw
        //          Serial.println(formattedDate);
        //
        //          // Extract date
        //          int splitT = formattedDate.indexOf("T");
        //          dayStamp = formattedDate.substring(0, splitT);
        //          Serial.print("DATE: ");
        //          Serial.println(dayStamp);
        //          // Extract time
        //          timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
        //          Serial.print("HOUR: ");
        //          Serial.println(timeStamp);
        //        }


        //  Status();

        net_id == false;
      }

    }
  }
}


//*************************************************************************************************************************************
//--------------------------------------------------------Function Status--------------------------------------------------------------
//*************************************************************************************************************************************

void Status() {

  Serial.println(String("Firmware Ver: ") + FW_Version);
  Serial.println();
  Serial.println(String("Scan mode: ") + EEPROM.read(12));
  Serial.println(String("Full matrix H:") + Full_mtrx_raw + " W:" + Full_mtrx_clmn );
  Serial.println(String("Sampling Matrix H:") + Raw + " W:" + Column);
  Serial.println(String("Raw starts on: ") + Raw_start);
  Serial.println(String("X_Offset: ") + x_offset);
  Serial.println(String("Scan Filter values below of: ") + Filter_scan);
  Serial.println(String("Transmitting HTTP Filter is above of: ") + Filter_tx);
  Serial.println(String("Max transmission ID: ") + Sample_raw);
  Serial.println(String("Local System ID: ") + UniqueID);
  Serial.println(String("Global System ID: ") + ID_name);
  Serial.println(String("Sampling interval: ") + Scan_Time_Delay_usec);
  Serial.println(String("Testing mode status: ") + Testing_Mode + " Y:" + Test_y + " X:" + Test_y);
  Serial.println(String("Tx trigger pixel amount TH: ") + ChangeDetectTH);
  Serial.println(String("Tx trigger board value TH: ") + ValueDetectTH);
  Serial.println(String("Buffer comparation pixel mark A/R TH: ") + SamplingValueDelta);


  Serial.println();
  //    Serial.println(WiFi.hostname() + String(" module network parameters"));
  Serial.println("-----------------------------------------------------------------");
  // print SSID name,local IP allotted ,Subnet Mask, Gateway IP,MAC address & signal strength
  // Serial.println();
  Serial.print("Connection status: ");
  if (WiFi.isConnected()) {
    Serial.println(String("Connected to <") + WiFi.SSID() + ">");
  } else {
    Serial.println("Not connected to network");
  }
  Serial.println(String("Wifi output power: ") + WiFi.getTxPower());
  Serial.println(String("Wifi power level: ") + EEPROM.read(23));

  Serial.println(String("AP Connection SSID         : ") + WiFi.SSID());

  Serial.print("AP Connection BSSID        : "); Serial.println(WiFi.BSSIDstr()); // AP MAC address

  Serial.print("IP allotted to module      : "); Serial.println( WiFi.localIP());

  Serial.print("Subnet mask                : "); Serial.println(WiFi.subnetMask());

  Serial.print("Gateway IP                 : "); Serial.println(WiFi.gatewayIP());

  Serial.print("UDP Tx Address:            : "); Serial.println(udpAddress);

  Serial.print("UDP Tx Port:               : "); Serial.println(udpPort);

  Serial.println(String("ESP MAC address            : ") + WiFi.macAddress());

  Serial.println(String("Signal strength            : (") + WiFi.RSSI() + String(" dBm)"));

  //Serial.println(); Serial.println();

  //  Serial.println(WiFi.hostname() + String(" module DNS parameters"));
  Serial.println("-----------------------------------------------------------------");

  /* With the input parameter dns_no we can specify which Domain Name Server's IP we need. This parameter is zero based and allowed values are none, 0 or 1. If no parameter is provided, then IP of DNS #1 is returned. */
  Serial.print("DNS #1 IP                  :"); Serial.println(WiFi.dnsIP(0) );
  Serial.print("DNS #2 IP                  :"); Serial.println(WiFi.dnsIP(1) );


  // Serial.println(WiFi.status());   // Print ESP8266 connection status
  // Serial.println(WL_CONNECTED);
  //Serial.println();
  //Serial.println();
  //  Serial.println(WiFi.hostname().c_str() + String(" module parameters"));
  Serial.println("-----------------------------------------------------------------");
  //  Serial.println(String("Host name                  :") + WiFi.hostname());
  Serial.println();
  WiFi.printDiag(Serial);  // ESP32 Diagnostic
  //  Serial.println(String ("Coverage of") + WiFi.getPhyMode() + String(" protocols is present")); // check which coverage protocol is present (802.11a,802.11b,802.11g,802.11n,802.11ac)
  Serial.println();
  Serial.println();

  //  Serial.println (String("Number of clients connected to ") + host_name + String(": ") + WiFi.softAPgetStationNum());   // number of clients connected to ESP8266
  // Serial.println(String("Remote IP: ")+ Client.remoteIP());


}

//*************************************************************************************************************************************
//--------------------------------------------------------Function "Reconnect to previous connected Wifi AP"---------------------------
//*************************************************************************************************************************************

void reconnect() {
  bool led_state = false;
  // WiFi.reconnect();  // Reconnect to network with last connection to the AP (should be recorded SSID and password in the memory) if use WiFi.disconnect() function the autoreconnect will not work since WiFi.disconnect() will null the SSID and password parameters
  ssid = ssid_reconnect;
  password = password_reconnect;
  if (past_connection > 0) {
    //
    WiFi.disconnect(); // disconnect from connected network (set ssid and password to zero)
    delay(200);
    WiFi.begin(ssid, password);
    Serial.println();
    Serial.print("Reconnecting");
    //    WiFi.reconnect();
    int con_attempt = 0;


    /* Function WiFi.status() returns one of the following connection statuses:
        -----------------------------------------------------------
      3 -WL_CONNECTED after successful connection is established
      1 -WL_NO_SSID_AVAILin case configured SSID cannot be reached
      4 -WL_CONNECT_FAILED if password is incorrect
      0 -WL_IDLE_STATUS when Wi-Fi is in process of changing between statuses
      6 -WL_DISCONNECTED if module is not configured in station mode
    */
    while (WiFi.status() != WL_CONNECTED)
    {
      delay(500);
      Serial.print(".");
      led_state = !led_state;
      LEDG(led_state);
      con_attempt++;

      if (Serial.available() > 0) {
        delay (100);
        while (Serial.available() > 0) {

          serial_read = Serial.readStringUntil('\n'); // fill string with received information
        }
        serial_read.toLowerCase();  // Lower case the input information
        if (serial_read == con_abort) {
          connection_abort();
          connection_abort_cmnd = true;
          break;
        }
      }

      if (WiFi.status() == WL_CONNECT_FAILED || con_attempt == 80 ) {
        Serial.println();
        Serial.println("Failed to connect");
        connection_abort();
        break;
      }

      if (WiFi.status() == WL_CONNECTED) {
        Serial.println (String("Reconnected to: ") + ssid_reconnect);
        HTTP_server_set();
        //       digitalWrite(WiFi_LED, HIGH);
        LEDG(HIGH);
        connection_lost_flag = false;
        HTTP_flag = true;


        configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
        printLocalTime();

        //        timeClient.begin();
        //        delay(1000);
        //        while (!timeClient.update()) {
        //          timeClient.forceUpdate();
        //          Serial.println( "NTP servere updated");
        //          formattedDate = timeClient.getFormattedDate();
        //          Serial.println(formattedDate);
        //
        //          // Extract date
        //          int splitT = formattedDate.indexOf("T");
        //          dayStamp = formattedDate.substring(0, splitT);
        //          Serial.print("DATE: ");
        //          Serial.println(dayStamp);
        //          // Extract time
        //          timeStamp = formattedDate.substring(splitT + 1, formattedDate.length() - 1);
        //          Serial.print("HOUR: ");
        //          Serial.println(timeStamp);
        //        }
      }
    }
  } else ( Serial.println("There was no connection before"));


}


void factory_reset()
{
  Serial.println("Setting Factory Reset");
  delay(100);
  int column_fctr = EEPROM.read(40);
  delay(100);
  EEPROM.write(25, column_fctr );
  delay(100);
  EEPROM.commit();
  delay(100);
  Column = EEPROM.read(25);     //  Scan X Axis
  int raw_fctr = EEPROM.read(41);
  delay(100);
  EEPROM.write(24, raw_fctr );
  delay(100);
  EEPROM.commit();
  delay(100);
  int raw_offset_fctr = EEPROM.read(42);
  delay(100);
  EEPROM.write(4, raw_offset_fctr);
  delay(100);
  EEPROM.commit();
  Raw_start = EEPROM.read(4);
  Raw_start_offset = Raw_start;
  delay(100);
  int full_column_fctr = EEPROM.read(43);
  delay(100);
  EEPROM.write(6, full_column_fctr );
  delay(100);
  EEPROM.commit();
  delay(100);
  int full_raw_fctr = EEPROM.read(44);
  delay(100);
  EEPROM.write(21, full_raw_fctr );
  delay(100);
  EEPROM.commit();
  delay(100);
  //  EEPROM.write(12, EEPROM.read(45));
  int column_offset_fctr = EEPROM.read(45);
  delay(100);
  EEPROM.write(5, column_offset_fctr);
  delay(100);
  EEPROM.commit();
  delay(100);
  int fltr_fctr = EEPROM.read(46);
  delay(100);
  EEPROM.write(9, fltr_fctr );
  delay(100);
  EEPROM.commit();
  delay(100);
  Filter_scan = (EEPROM.read(9) * 50);
  delay(100);
  int http_fltr_fctr = EEPROM.read(47);
  delay(100);
  EEPROM.write(10, http_fltr_fctr);
  delay(100);
  EEPROM.commit();
  Filter_tx = (EEPROM.read(10) * 50);
  delay(100);
  int transmission_ID_fctr = EEPROM.read(48);
  delay(100);
  EEPROM.write(11, transmission_ID_fctr );
  delay(100);
  EEPROM.commit();
  delay(100);
  int system_ID_fctr = EEPROM.read(49);
  delay(100);
  EEPROM.write(22, system_ID_fctr );
  delay(100);
  EEPROM.commit();
  delay(100);
  int wifi_pwr_fctr = EEPROM.read(50);
  delay(100);
  EEPROM.write(23, wifi_pwr_fctr );
  delay(100);
  EEPROM.commit();
  int Scan_mode_set = EEPROM.read(51);
  delay(100);
  EEPROM.write(12, Scan_mode_set );
  delay(100);
  EEPROM.commit();
  delay(100);
  int ValueDetectTH_ = (EEPROM.read(244) << 8) + EEPROM.read(245);
  EEPROM.write(242, ValueDetectTH_ >> 8);
  EEPROM.commit();
  EEPROM.write(243, ValueDetectTH_ );
  EEPROM.commit();

  int sampling_offset_ = (EEPROM.read(248) << 8) + EEPROM.read(249);
  EEPROM.write(246, sampling_offset_ >> 8);
  EEPROM.commit();
  EEPROM.write(247, sampling_offset_ );
  EEPROM.commit();
  sampling_offset = (EEPROM.read(246) << 8) + EEPROM.read(247);
  scan_mode = EEPROM.read(12);
  delay(100);
  Raw = EEPROM.read(24);       //   Scan Y Axis
  delay(100);
  Column_start = EEPROM.read(5);
  Column_start_offset = Column_start;
  x_offset = Column_start;
  delay(100);
  Raw_start = EEPROM.read(4);
  Raw_start_offset = Raw_start;
  delay(100);
  Full_mtrx_clmn = EEPROM.read(6);
  delay(100);
  Full_mtrx_raw = EEPROM.read(21);
  delay(100);
  scan_mode = EEPROM.read(12);
  delay(100);
  Filter_scan = (EEPROM.read(9) * 50);
  delay(100);
  Filter_tx = (EEPROM.read(10) * 50);
  delay(100);
  Sample_raw = EEPROM.read(11);
  delay(100);
  UniqueID = EEPROM.read(22);
  delay(100);
  sampling_offset = EEPROM.read(298);
  delay(100);
  ValueDetectTH = EEPROM.read(290);


  for (int i = 0; i < 20; i++)
  {
    EEPROM.write(60 + i, EEPROM.read(150 + i)); // SSID name factory to actual
    EEPROM.commit();

    EEPROM.write(80 + i, EEPROM.read(170 + i)); // SSID password factory to actual
    EEPROM.commit();

  }

  Serial.println("Factory Reset Done");
}

void Hard_factory_reset()
{
  Serial.println("Setting Hard Factory Reset");

  EEPROM.write(40, 45);    //column_fctr
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(25, 45);    //column
  Column = EEPROM.read(25);
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(41, 120 ); //raw_fctr
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(24, 120 ); //raw
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(42, 0 ); //raw offset factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(4, 0); //raw offset
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(42, 0); //raw offset factory
  delay(100);
  EEPROM.commit();
  Raw_start_offset = 0;
  delay(100);
  EEPROM.write(6, 45 );  // full column
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(43, 45 );  // full column factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(44, 120 ); // full raw factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(21, 120 );// full raw
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(45, 0); // column offset factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(5, 0); // column offset
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(9, 2 );  // scan filter
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(46, 2 ); //// scan filter factory
  delay(100);
  EEPROM.commit();
  delay(100);
  Filter_scan = (EEPROM.read(9) * 50);
  delay(100);
  EEPROM.write(47, 2); // HTTP filter factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(10, 2); // HTTP filter
  delay(100);
  EEPROM.commit();
  Filter_tx = (EEPROM.read(10) * 50);
  delay(100);
  EEPROM.write(48, 1 );  // Transmission ID factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(11, 1 );   // Transmission ID
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(49, 1 );  //System ID factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(22, 1 ); //System ID
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(50, 0 ); // Wifi power factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(23, 0 ); // Wifi power
  delay(100);
  EEPROM.commit();
  EEPROM.write(51, 0 );  // Scan mode factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(12, 0 ); // Scan mode
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(246, 0 ); // Sampling offset byte 0
  delay(100);
  EEPROM.write(247, 0 ); // Sampling offset byte 1
  delay(100);
  EEPROM.write(248, 0 ); // Sampling offset factory byte 0
  delay(100);
  EEPROM.write(249, 0 ); // Sampling offset factory byte 1
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(295, 7 ); // Change detection Threshold
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(296, 7 ); // Change detection Threshold factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(293, 100 ); // Change Buffer event Threshold
  delay(100);
  EEPROM.write(292, 100 ); // Change Buffer event Threshold factory
  delay(100);
  EEPROM.commit();
  EEPROM.write(290, 100 ); // Pixel detect Value Threshold
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(291, 100 ); // Pixel detect Value Threshold factory
  delay(100);
  EEPROM.commit();
  delay(100);
  EEPROM.write(242, 0x03); // Value Detection Threshold byte 0
  EEPROM.commit();
  EEPROM.write(243, 0xE8); // Value Detection Threshold byte 1
  EEPROM.commit();
  EEPROM.write(244, 0x03); // Value Detection Threshold factory byte 0
  EEPROM.commit();
  EEPROM.write(245, 0xE8); // Value Detection Threshold factory byte 1
  EEPROM.commit();
  EEPROM.write(246, 1); // Pixel Mark A/R Toggle
  EEPROM.commit();
  EEPROM.write(291, 1); // Dynamic Compensation Toggle 
  EEPROM.commit();


  delay(100);
  scan_mode = EEPROM.read(12);
  delay(100);
  Raw = EEPROM.read(24);       //   Scan Y Axis
  delay(100);
  Column_start = EEPROM.read(5);
  Column_start_offset = Column_start;
  x_offset = Column_start;
  delay(100);
  Raw_start = EEPROM.read(4);
  Raw_start_offset = Raw_start;
  delay(100);
  Full_mtrx_clmn = EEPROM.read(6);
  delay(100);
  Full_mtrx_raw = EEPROM.read(21);
  delay(100);
  scan_mode = EEPROM.read(12);
  delay(100);
  Filter_scan = (EEPROM.read(9) * 50);
  delay(100);
  Filter_tx = (EEPROM.read(10) * 50);
  delay(100);
  Sample_raw = EEPROM.read(11);
  delay(100);
  UniqueID = EEPROM.read(22);
  delay(100);
  sampling_offset = (EEPROM.read(246) << 8) + EEPROM.read(247);
  delay(100);
  ValueDetectTH = (EEPROM.read(288) << 8) + EEPROM.read(289);
  delay(100);
  ValueDetectTHfctr = (EEPROM.read(290) << 8) + EEPROM.read(291);
  delay(100);
  BufferEventTH = EEPROM.read(293);
  delay(100);
  PixelMarkState = EEPROM.read(250);
  delay(100);
  dyncomp = EEPROM.read(291);

  Serial.println("Hard Factory Reset Done");
}

//*************************************************************************************************************************************
//--------------------------------------------------------Function "HTTP ID request"---------------------------------------------------
//*************************************************************************************************************************************

void HTTP_ID_request()
{
  Serial.println("ID request");
  // digitalWrite(COM_LED, HIGH);
  LEDR(HIGH);
  //String URL = "http://nodeudpenv-env.eba-zkpremhg.us-east-1.elasticbeanstalk.com/api/shelf/getId?mac=";
  //String URL = "https://wa-bl-server.herokuapp.com/api/sensor/getid?";
   String URL = "http://34.134.199.126:2224/api/sensor/getid?";
  //String URL = "http://localhost:500/api/sensor/getid?";
  Message = URL + "mac=" + String(WiFi.macAddress()) + String("&type=") + Item;
  //Message = URL + "mac=" + String("&type=")+Item;
  
  // Serial.println("Request ID from the server: " + Message);
  HTTPClient http;    //Declare object of class HTTPClient

  //  http.begin("http://3.230.94.5:7000/api/surface/address");      //Specify request destination
  //http.begin("http://nodeudpenv-env.eba-zkpremhg.us-east-1.elasticbeanstalk.com/api/shelf/getId?mac=");
  http.begin(Message);
  //  delay(50);
  // Serial.println(http.POST(Message));

  // int httpResponseCode = http.POST("");
  int httpResponseCode = http.GET();
  
  //  int httpResponseCode = http.POST(Message);

  if (httpResponseCode > 0) {

    String response = http.getString();                       //Get the response to the request
    //
    Serial.println(String(httpResponseCode));   //Print return code
    Serial.println(response);           //Print request answer
    if (httpResponseCode==200)
    {
      Serial.println(String("getd ID successful"));
    
      getID=true;
      ID_name = response.substring(27, 32);
      addr = 289;
      for (byte i = 0; i < 5; i++)
      {

        if (ID_name[i] != '"')
        {
        EEPROM.write(addr + i, ID_name[i]);
        EEPROM.commit();
        } else
        {
        for (byte k = i; k < 5; k++)
        {
          EEPROM.write(addr + k, 0);
          EEPROM.commit();
        }
        break;
        }
      }
    } else {
           for (byte i = 0; i < 5; i++)
      {
      if (EEPROM.read(addr + i)!=0)
      {
      EEPROM.write(addr + i, 0);
      EEPROM.commit();
        
      }
      }
      Serial.println("can't get ID");
      getIDfaulseTimer=millis();
      LEDR(HIGH);
      LEDG(HIGH);
      }
    

  }

}

//*************************************************************************************************************************************
//--------------------------------------------------------Function "MQTT transmitt"----------------------------------------------------
//*************************************************************************************************************************************

void MQTT_transmit()
{
  if (client.connected())
  {
    client.publish(topic, "Testing matrix message :)");
    Serial.println("Message sent ");
  } else
  {
    Serial.println("MQTT Client not connected ");
  }

  //client.publish(topic, "testing message from matrix");
  //  client.subscribe(topic);
  //  Serial.println (Message);
  //  unsigned int packetIdPub1 = mqttClient.publish(MQTT_PUB_TEMP, 1, true, String(Message).c_str());
  //  Serial.printf("Publishing on topic %s at QoS 1, packetId: ", MQTT_PUB_TEMP);
  //  Serial.println(packetIdPub1);
}

void callback(char *topic, byte * payload, unsigned int length) {
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char) payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}

//*************************************************************************************************************************************
//--------------------------------------------------------Function "HTTP transmitt"----------------------------------------------------
//*************************************************************************************************************************************

void HTTP_transmit()
{

  LEDR(HIGH);

  if (string_show) {
    if (scan_show) {
      Serial.println(ScnShwMessage);
    } else
    {
      Serial.println(Message);

    }
  }
  //**********************************UDP transmission*************************************************

  int UDP_limit = 1400;
  //  int UDP_limit = 100;
  int UDP_buffer_index = Message.length() / UDP_limit;
  int UDP_SplitIndex = 0;
  int CharacterSum = 0;
  Serial.println (String("UDP buffer index: ") + UDP_buffer_index);
  int CommaCount = 0;
  String Message3_HIGH;
  String Temp1Message2;
  CharacterSum = Message.length();
  //------------------------------------------------------- Split string to be without header
  // Serial.println(Message);
  for (int i = 0; i < Message.length(); i++)
  {
    //      Serial.print(Message.charAt(i));
    if (Message.charAt(i) == ',')
    {
      CommaCount++;
      if (CommaCount == 6)
      {
        Temp1Message2 = Message.substring(i + 1);

        //               Serial.println();
        //              Serial.println(Temp1Message2);
        break;
      }
    }
  }


  for (int v = 0; v < UDP_buffer_index + 1; v++)
  {
    Message3_HIGH = Temp1Message2.substring(UDP_SplitIndex, UDP_SplitIndex + UDP_limit);
    // Serial.println(Message3_HIGH);
    UDP_SplitIndex += UDP_limit;
    Message = String (Header2 + v + String(",") + (UDP_buffer_index) + String(",") + Sampled_mesh + String(",!,") + Message3_HIGH + String("$"));

    // if (string_show) {
    //   Serial.println(Message);
    // }
    Message.getBytes(UDP_send_buffer, Message.length());
    //UDP_send_buffer_voaltile=strlen(UDP_send_buffer);
    //UDP_send_buffer=Message.getBytes(UDP_send_buffer,Message.length());
    udp.beginPacket(udpAddress, udpPort);
    udp.write(UDP_send_buffer, (sizeof(UDP_send_buffer) / sizeof(UDP_send_buffer[0])));

    udp.endPacket();

    memset(UDP_send_buffer, 0, sizeof(UDP_send_buffer));  // clear UDP_send_buffer array
    //****************************************************************************************************

    Serial.println(String ("Transmission ") + v + String("/") + UDP_buffer_index + String(" UDP Characters: ") + Message.length() + String (" of ") + CharacterSum + " Bytes: " + (sizeof(UDP_send_buffer) / sizeof(UDP_send_buffer[0])) + "b Sampled mesh:" + Sampled_mesh );

    //***************************** Webhook.site transmitt**********************
    // WiFiClient client;
    // HTTPClient http;


    // http.begin("https://webhook.site/27504462-d72a-40fc-960f-df63dcd63daf");
    // http.addHeader("Content-Type", "application/json");
    // //Serial.println(PostURL);
    // int httpResponseCode = http.POST(Message);
    // // int httpResponseCode = http.POST(PostURL);
    // if (httpResponseCode > 0) {
    //   String response = http.getString();
    //   Serial.println(httpResponseCode);
    //   Serial.println(response);
    // }
    //***************************************************************************
  }
  LEDR(LOW);
}

//*************************************************************************************************************************************
//--------------------------------------------------------Scan time interval restore"--------------------------------------------------
//*************************************************************************************************************************************

void Scan_time_interval_restore()
{
  String Time_delay;
  for (int i = 235; i < 241; i++) {
    byte readValue = EEPROM.read(i);
    /*    if (readValue == 0) {
          break;
        } */
    Time_delay += char(readValue);

  }
  Scan_Time_Delay_usec = Time_delay.toInt();
}

//*************************************************************************************************************************************
//--------------------------------------------------------URL restore"-----------------------------------------------------------------
//*************************************************************************************************************************************

void URL_restore()
{

  byte UDP_string_counter = 0;



  String UDP_TX_URL_read;
  for (int i = 200; i < 229; i++) {
    byte readValue = EEPROM.read(i);
    /*    if (readValue == 0) {
          break;
        } */
    UDP_TX_URL_read += char(readValue);
    if (readValue == 255)
    {
      UDP_string_counter++;
    }
    if (UDP_string_counter > 20)
    {
      addr = 200;
      for (int y = addr; y < addr + 30; y++)
      {
        if (udpAddress[y - addr] != EEPROM.read(y))
        {
          for (int i = 0; i < 30; i++) {
            EEPROM.write(addr + i, udpAddress[i]);
            EEPROM.commit();

          }
          udpAddress = UDP_TX_URL;
          break;
        }
      }

      break;
    }
  }

  UDP_TX_URL_read.toCharArray(UDP_TX_URL, 30);
  //  Serial.println(String ("TX URL:") + UDP_TX_URL);
  udpAddress = UDP_TX_URL;
  //  Serial.println(String ("UDP TX Address:") + UDP_TX_URL);
  //Serial.println(String ("UDP TX Address:") + udpAddress);


  String UDP_TX_Port_read;
  for (int i = 230; i < 235; i++) {
    byte UDPport_string_counter = 0;
    byte readValue = EEPROM.read(i);
    if (readValue == 255)
    {
      String udpstore = String(udpPort);
      udpstore.toCharArray(UDP_TX_Port, 5);
      addr = 230;
      for (int y = addr; y < addr + 5; y++)
      {
        if (UDP_TX_Port[y - addr] != EEPROM.read(y))
        {
          for (int i = 0; i < 5; i++)
          {
            EEPROM.write(addr + i, UDP_TX_Port[i]);
            EEPROM.commit();

          }
        }
      }
    }
    UDP_TX_Port_read += char(readValue);

  }
  UDP_TX_Port_read.toCharArray(UDP_TX_Port, 5 );

  unsigned int UDP_port = UDP_TX_Port_read.toInt();
  //  Serial.println(String ("UDP TX Port Real:") + UDP_port);

  udpPort = UDP_port;
  //   Serial.println(String ("TX URL:")+UDP_TX_Port);
  //      int *UDP_port;
  //      UDP_port=&udpPort; //pointer points the udpPort
  //      *UDP_port=UDP_TX_Port_read.toInt();
  Serial.println(String ("UDP TX Port:") + udpPort);


}

//*************************************************************************************************************************************
//--------------------------------------------------------"ID unique restore"----------------------------------------------------------
//*************************************************************************************************************************************

void Unique_ID_restore()
{
  ID_name = "";
  addr = 289;
  for (int i = addr; i < addr + 5; i++)
  {
    byte readValue = EEPROM.read(i);
    ID_name += char(readValue);
   
  }
    Serial.println(ID_name);
}

//*************************************************************************************************************************************
//--------------------------------------------------------"AP SSID and PSWRD restore"--------------------------------------------------
//*************************************************************************************************************************************

void SSID_name_and_pswrd_restore()
{
  String ssid_name;
  String ssid_name_fctr;

  for (int i = 55; i < 80; i++) {
    byte readValue = EEPROM.read(i);
    if (readValue == 0) {
      break;
    }
    ssid_name += char(readValue);

  }
  for (int i = 150; i < 169; i++) {
    byte readValue = EEPROM.read(i);
    if (readValue == 0) {
      break;
    }
    ssid_name_fctr += char(readValue);

  }


  String ssid_password_reconnect;
  String ssid_password_reconnect_fctr;
  passwrd_stars = "";
  for (int i = 80; i < 100; i++) {
    byte readValue = EEPROM.read(i);
    passwrd_stars += "*";
    if (readValue == 0) {
      break;
    }
    ssid_password_reconnect += char(readValue);
  }

  passwrd_stars_fctr = "";
  for (int i = 170; i < 189; i++) {
    byte readValue = EEPROM.read(i);
    passwrd_stars_fctr += "*";
    if (readValue == 0) {
      break;
    }
    ssid_password_reconnect_fctr += char(readValue);
  }

  ssid_name.toCharArray(ssid_reconnect, 25); // convert string to char array
  ssid_name_fctr.toCharArray(ssid_reconnect_fctr, 20); // convert string to char array
  ssid_password_reconnect.toCharArray(password_reconnect, 20); // convert string to char array
  ssid_password_reconnect_fctr.toCharArray(password_reconnect_fctr, 20); // convert string to char array
}

//void Sensors_serial_print()
//{
//  int  sensors_all = 0;
//  Serial.println(String("Sensor counter: ") + Sample_counter);
//  for (int y = 0; y < Raw; y++)
//  {
//    for (int x = 0; x < Column; x++)
//    {
//
//      //      Serial.print(String("X") + y + String("Y") + x + String("= ") + Sample[Sample_counter][sensors_all] + String(" , ") );
//      Serial.print(String("X") + y + String("Y") + x + String("= ") + sampling + String(" , ") );
//      sensors_all++;
//      //      if (sensors_all == (sizeof(Sample) / sizeof(Sample[0])))
//      //      {
//      //        sensors_all = 0;
//      //        Serial.println();
//      //      }
//    }
//
//  }
//}

//*************************************************************************************************************************************
//--------------------------------------------------------"Deep sleepe execution"-----------------------------------------------------------------
//*************************************************************************************************************************************

void deep_sleep_execute()
{



  if (TIME_TO_SLEEP > 0)
  {
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }

  //  if (loop_couter==100){
  //   esp_deep_sleep_start();
  //  }
  //  loop_couter++;
  // Serial.println(String("loop counter:") + loop_couter);


}


//*************************************************************************************************************************************
//--------------------------------------------------------"Test mode"-----------------------------------------------------------------
//*************************************************************************************************************************************

void TestingMode(int Axis_y, int Axis_x)
{
  if (sample_2 && Testing_scan)
  {
    sensor = 0;
    //    shifter.clear();

    //for (int y = Raw_start; y < Axis_y + 1; y += pair)
    for (int y = 0; y < Axis_y + 1; y += pair)

    {
      //  delay(0);
      if (y == 0)
      {
        SER(false);
        //   digitalWrite(Shift_Data, LOW);
      } else {
        SER(true);
        //   digitalWrite(Shift_Data, HIGH);
      }
      SR_CLK(true);
      SR_CLK(false);
      SR_LATCH(true);
      SR_LATCH(false);
      /*
            digitalWrite(Shift_Clk, HIGH);
            digitalWrite(Shift_Clk, LOW);
            digitalWrite(Shift_Latch, HIGH);
            digitalWrite(Shift_Latch, LOW);
      */
      delay(1);


      if (sample_1 || sample_2)
      {

        if (y == (Axis_y))
        {

          Serial.print(String("Y") + y + String(" "));
        }
      }
    }
    for (int x = 0; x < (Axis_x + 1); x += pair)
    {
      if (x == (Axis_x) && (sample_1 || sample_2))
      {
        //sensor_state_optimized(sensor);
        sensor_state(sensor);
        //      digitalWrite(FET_shift_clear, LOW);
        //      digitalWrite(FET_shift_clear, HIGH);
        nSRCLR2(false);
        nSRCLR2(true);
        SR_LATCH(true);
        SR_LATCH(false);
        //        digitalWrite(Shift_Latch, HIGH);
        //        digitalWrite(Shift_Latch, LOW);
//        FET_SR_control_shift((Axis_x + 1));
        //sampling = analogRead(Analog_read);
        sampling = A2D1_sampling();


        sampling = constrain(sampling, 0, 4095);   // Sampling range between 0-4095 (if sampling<0 then sampling=0, if sampling>4095 then sampling=4095)
        //       Serial.println(String("X") + x + ": Value:" + sampling );
        Serial.print(String("X") + x + ": Value:" + sampling );
        float ref_voltage = 3.3;
        float voltage = (sampling * ref_voltage) / 4096;
        Serial.print("  Voltage = ");
        Serial.print(voltage, 3);
        Serial.println("V");
        break;
      }

      Column_switch();

    }
    if (!auto_sample)
    {
      //     Serial.println("auto scan off");   // for debug only
      Testing_scan = false;
    }

  }

}

void FirstTimeFlashSetup()
{
  byte ReadFirstTimeFlash = EEPROM.read(299);
  Serial.println(ReadFirstTimeFlash);
  if (ReadFirstTimeFlash == 255)
  {
    EEPROM.write(299, 0);
    EEPROM.commit();
    Hard_factory_reset();
  }
}

bool ChangeTX (int SampledMesh, int SampledValueSum) {
  bool TxState = false;
  int MeshDelta = abs(SampledMesh - Sampled_mesh_prev);
  int SampleValueDelta = abs(SampledValueSum - SamplingValueSumPrev);

  // Serial.println(SampledMesh);
  // Serial.println(Sampled_mesh_prev);
  // Serial.println(MeshDelta);
  // Serial.println(SampleValueDelta);
  if (MeshDelta > ChangeDetectTH && SampleValueDelta > ValueDetectTH)
  {
    sample_mesh_Tx_duration = millis();
  }
  sampling_Top = MeshDelta;
  Sampled_mesh_prev = SampledMesh;
  // SamplingValueSumPrev=SampledValueSum;



  // bool ChangeTX (int SampledMesh, byte ValueDetectTH_)
  // {
  //   bool TxState = false;
  //     //  Serial.println(ChangeDetectTH);
  //   //  Serial.println(abs(SampledMesh - Sampled_mesh_prev));
  //   SamplingValueDelta = abs(SamplingValueSum - SamplingValueSumPrev);
  //   Serial.println(SamplingValueDelta);
  //   Serial.println(SampledMesh - Sampled_mesh_prev);
  //   if (abs(SampledMesh - Sampled_mesh_prev) > ChangeDetectTH && SamplingValueDelta>ValueDetectTH_)
  //   {
  //     sample_mesh_Tx_duration = millis();
  //   }
  //   sampling_Top = abs(SampledMesh - Sampled_mesh_prev);






  //  Serial.println(sample_mesh_Tx_duration);
  //  Serial.println((millis() - sample_mesh_Tx_duration));
  if ((millis() - sample_mesh_Tx_duration) < 500)
  {
    return TxState = true;
  } else {

    sample_mesh_Tx_duration = 0;
    return TxState = false;
  }
}

void SettingsRestore() {

  Column = EEPROM.read(25);     //  Scan X Axis
  Raw = EEPROM.read(24);       //   Scan Y Axis
  Column_start = EEPROM.read(5);
  Raw_start = EEPROM.read(4);
  Full_mtrx_clmn = EEPROM.read(6);
  Full_mtrx_raw = EEPROM.read(21);
  scan_mode = EEPROM.read(12);
  Filter_scan = (EEPROM.read(9) * 50);
  Filter_tx = (EEPROM.read(10) * 50);
  Sample_raw = EEPROM.read(11);
  UniqueID = EEPROM.read(22);
  sampling_offset = (EEPROM.read(246) << 8) + EEPROM.read(247);
  ChangeDetectTH = EEPROM.read(295);
  BufferEventTH = EEPROM.read(293);
  ValueDetectTH = (EEPROM.read(242) << 8) + EEPROM.read(243);
  ValueDetectTHfctr = (EEPROM.read(244) << 8) + EEPROM.read(245);
  PixelMarkState = EEPROM.read(250);
  dyncomp = EEPROM.read(291);

}

void PrimerServConfigRestore() {
  int addr = 300;
  for (int i = addr; i < addr + 100; i++) {
    byte readValue = EEPROM.read(i);
    FirstConfigURL[i - addr] += char(readValue);
  }
  //  Serial.println(FirstConfigURL);
}

void SecondServConfigRestore() {
  int addr = 400;
  for (int i = addr; i < addr + 100; i++) {
    byte readValue = EEPROM.read(i);
    SecondConfigURL[i - addr] += char(readValue);
  }
  //  Serial.println(SecondConfigURL);
}

void bufmem() {
  Serial.println();
  for (int i = 0; i < 120; i++) {
    Serial.print(String("Y") + String(i) + String(": "));
    for (int k = 0; k < 45; k++) {
      Serial.print(String(buffer_board_flag[i][k]) + String(","));
    }
    Serial.println();

  }
}


// void ColumnDeltaCount()
// {
//   // byte XActiveMeshFlagCount[45];
//   // int XActiveMeshValueDelta[45];


//   for (int x = 0; x < Column; x++) {
//     byte DeltaCount = 0;
//     int DeltaValue = 0;
//     int X_SumValue = 0;
//     for (int y = 0; y < Raw; y++) {
//       //    Serial.print(String(y) + String(","));
//       if (CompensationDelta[y][x] > 0) {
//         DeltaCount++;
//         X_SumValue += CompensationDelta[y][x];

//         // Serial.print(String("X") + String(x) + String(":") + String(CompensationDelta[y][x]) + String(","));

//       }

//     }
//     // Serial.println();

//     if (DeltaCount > 0)
//     {
//       XActiveMeshFlagCount[x] = DeltaCount;
//       XActiveMeshValueDelta[x] = (X_SumValue / XActiveMeshFlagCount[x]);
//       // Serial.println(String(DeltaCount) + String(" ") + String(XActiveMeshValueDelta[x]));
//       // Serial.println(X_SumValue);
//     } else {
//       XActiveMeshFlagCount[x] = 0;
//       XActiveMeshValueDelta[x] = 0;
//     }

//     XActiveMeshFlagCount[x] = DeltaCount;
//     //    X_SumValue = 0;
//     //    DeltaCount = 0;
//     //    Serial.print(String ("X") + String(x) + String(":") + String(XActiveMeshValueDelta[x]) + String(" ") + String(XActiveMeshFlagCount[x]) + String(","));
//   }
//   //  Serial.println();
// }

// void CompensatedSample()
// {
//   for (int y = 0; y < Raw; y++)
//   {
//     for (int x = 0; x < Column; x++)
//     {


//       Sampling_buffer += (y + String(",") + x + "," + buffer_board[y][x] + ",");
//     }
//   }


//   //   for (int y=0;y<Column;y++)
//   //   {
//   //     for (int x=0;x<Raw;x++)
//   //     {
//   //       XActiveMeshCount[y][x]=buffer_board[y][x];
//   //     }
//   //   }



//   //   if (sampling>buffer_CompensatedMesh[AxisY][AxisX])
//   // int Delta=sampling-buffer_CompensatedMesh[AxisY][AxisX];

//   // buffer_CompensatedMesh[AxisY][AxisX]=sampling;
// }




void CreateDelta(int y, int x)
{
  int MeshDelta = 0;

  // for (int x = 0; x < Column; x++)
  // {
  //   // Serial.print(String("X") + String(x) + String(":"));
  //   for (int y = 0; y < Raw; y++)
  //   {
  MeshDelta = buffer_board[y][x] - CompensatedMesh[y][x];
  if (abs(MeshDelta) > BufferEventTH) {
    XActiveMeshFlag[y][x] = 1;
  }
  // Serial.print(String(CompensationDelta[y][x]) + String(","));
  if ((abs(MeshDelta) > BufferEventTH && MeshDelta > 0) && (buffer_board[y][x] >= CompensatedMesh[y][x])) {
    CompensationDelta[y][x] = MeshDelta;
  } else if (abs(MeshDelta) < BufferEventTH && MeshDelta < 0 && buffer_board[y][x] == 0) {
    CompensationDelta[y][x] = 0;

  }
  if (buffer_board[y][x] == 0)
  {
    CompensationDelta[y][x] = 0;
  }
  //   }
  //   // Serial.println();
  // }
  // Serial.println();
  // Serial.println();
}


void DeltaSumValue()
{

  byte DeltaCount = 0;
  int X_SumValue = 0;

  for (int x = 0; x < Column; x++)
  {
    for (int y = 0; y < Raw; y++)
    {
      if (CompensationDelta[y][x] > 0)
      {
        //Serial.println(CompensationDelta[y][x]);
        DeltaCount++;
        X_SumValue += CompensationDelta[y][x];
      }
    }
    if (DeltaCount > 0) {
      XActiveMeshValueDelta[x] = X_SumValue / DeltaCount;
    }

    X_SumValue = 0;
  }
}


void MeshCompensate()
{
  Sampling_buffer_Comp = "";
  int CompSum = 0;
  for (int y = 0; y < Raw; y++)
  {
    for (int x = 0; x < Column; x++)
    {
      if (buffer_board[y][x] != 0)
      {
        if (PixelMarkState) {
          if (buffer_board_flag[y][x] == 1) {
            // buffer_board_flag[y][x] = 0;
            Sampling_buffer_Comp += "A";
          }

        }
        CompSum = buffer_board[y][x] + XActiveMeshValueDelta[x];
        if (CompSum > CompensatedMesh[y][x])
          CompensatedMesh[y][x] = CompSum;

      } else {
        CompensatedMesh[y][x] = 0;
        if (PixelMarkState) {
          if (buffer_board_flag[y][x] == 2) {
            // buffer_board_flag[y][x] = 0;
            Sampling_buffer_Comp += "R";
            Sampling_buffer_Comp += (y + String(",") + x + "," + String(CompensatedMesh[y][x]) + ",");
          }

        }
      }
      if (CompensatedMesh[y][x] != 0)
      {
        Sampling_buffer_Comp += (y + String(",") + x + "," + String(CompensatedMesh[y][x]) + ",");
      }





    }
  }
  // Serial.println(Sampling_buffer_Comp);
}

// void MeshCompensate() {
//  int MeshDelta = 0;
// byte DeltaCount = 0;
// int X_SumValue = 0;
// Sampling_buffer_Comp="";

//  for (int x = 0; x < Column; x++)
//   {
//     for (int y = 0; y < Raw; y++)
//     {
//       if ((abs(buffer_board[y][x] - CompensatedMesh[y][x]) > BufferEventTH) || (abs(CompensationDelta[y][x]) > BufferEventTH && CompensatedMesh[y][x] < 0 && buffer_board[y][x]!=0))
//       {

//       for (int i = 0; i < Raw; i++)
//           {

//               if (CompensationDelta[i][x] > 0)
//               {
//             // Serial.println(String("Delta:")CompensationDelta[y][x]);
//             DeltaCount++;
//             X_SumValue += CompensationDelta[i][x];

//               }

//           }
//           // Serial.println(DeltaCount);
//           if (DeltaCount>0) {
//             CompensatedMesh[y][x]=(X_SumValue/DeltaCount)+buffer_board[y][x];
//             // Serial.println(CompensatedMesh[y][x]);

//             DeltaCount=0;
//             X_SumValue=0;

//           }

//       }
//     }
//   }
// }

// void CompensateMeshMatrix()
// {

//   int MeshDelta = 0;
//   byte DeltaCount = 0;
//   // int DeltaValue = 0;
//   int X_SumValue = 0;

//   for (int x = 0; x < Column; x++)
//   {
//     Serial.print(String("X") + String(x) + String(":"));
//     for (int y = 0; y < Raw; y++)
//     {
//       MeshDelta = buffer_board[y][x] - CompensatedMesh[y][x];


//       // Serial.print(String("Y") + String(y) + String(":") + String(MeshDelta) + String(","));
//       // Serial.print(String(CompensatedMesh[y][x]) + String(","));

//       if ((abs(MeshDelta) > BufferEventTH && MeshDelta > 0) || (abs(MeshDelta) > BufferEventTH && MeshDelta < 0 && buffer_board[y][x]!=0))
//       {
//           for (int i = 0; i < Raw; i++)
//           {
//             if(i!=y)
//             {
//               if (CompensationDelta[y][x] > 0)
//               {
//                 //Serial.println(CompensationDelta[y][x]);
//             DeltaCount++;
//             X_SumValue += CompensationDelta[y][x];
//               }
//             }
//           }


//           CompensationDelta[y][x]=MeshDelta;


//         if (MeshDelta >= CompensationDelta[y][x] && CompensatedMesh[y][x] == 0)
//         {
//           if (DeltaCount > 0)
//           {
//             CompensatedMesh[y][x]=(X_SumValue/DeltaCount)+buffer_board[y][x];
//           } else {
//             CompensatedMesh[y][x]=buffer_board[y][x];
//           }

//         }
//       }
//        CompensationDelta[y][x] = 0;
//        CompensatedMesh[y][x]=buffer_board[y][x];


//       Serial.print(String(CompensationDelta[y][x]) + String(","));
//     }
//     Serial.println();
//   }
//   Serial.println();
//   Serial.println();
// }

void CompensatedMeshConsolePrint()
{
  Serial.println("Buffer board");
  for (int y = 0; y < Raw; y++)
  {
    Serial.print(String("Y:") + y + String(" "));
    for (int x = 0; x < Column; x++)
    {
      // Serial.print( CompensatedMesh[y][x] + String(","));
      Serial.print(buffer_board[y][x] + String(","));
    }

    Serial.println();

  }

  Serial.println();
  Serial.println();
  Serial.println("Delta");
  for (int y = 0; y < Raw; y++)
  {
    Serial.print(String("Y:") + y + String(" "));
    for (int x = 0; x < Column; x++)
    {
      Serial.print(CompensationDelta[y][x]  + String(","));

    }
    Serial.println();
  }
  Serial.println();
  Serial.println();
  Serial.println("Compensated board");
  for (int y = 0; y < Raw; y++)
  {
    Serial.print(String("Y:") + y + String(" "));
    for (int x = 0; x < Column; x++)
    {
      Serial.print(CompensatedMesh[y][x]  + String(","));

    }
    Serial.println();
  }
  Serial.println();
  Serial.println();
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                        END OF FUNCTIONS                                                                                                     //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                      START SETUP FUNCTION                                                                                                   //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void setup() {


  //  ESP.restart();
  Serial.begin(115200);
  EEPROM.begin(EEPROM_SIZE);
  SPI.begin();         // initialize the SPI library
  SPI.beginTransaction(SPISettings(SPI_clk, MSBFIRST, SPI_MODE0));

  delay(100);

  FirstTimeFlashSetup();
  SettingsRestore();
  PrimerServConfigRestore();
  SecondServConfigRestore();

  // LEDPWR(HIGH);
  ////eeprom cells read
  //for (int i=0;i<500;i++)
  //{
  // byte eeprom_count=EEPROM.read(i);
  // Serial.println(String("Eeprom ") + i + String(": ") + eeprom_count);
  //}
  int char_counter = 0;
  bool char_flag = LOW;
  web_update_date = "";
  for (int i = 100; i < 150; i++) {
    byte readValue = EEPROM.read(i);
    if (char_flag)
    {
      char_counter++;
    }

    if (char(readValue) == ':') {
      char_flag = HIGH;
    }

    web_update_date += char(readValue);

    if (char_counter == 5)
    {
      break;
    }
  }


  //  if (!SPIFFS.begin(true)) {
  //    Serial.println("An Error has occurred while mounting SPIFFS");
  //    return;
  //  }



  x_offset = Column_start;
  Column_start_offset = Column_start;
  Raw_start_offset = Raw_start;


  //  pinMode(WiFi_LED, OUTPUT);
  //  digitalWrite(WiFi_LED, LOW);

  //  pinMode(COM_LED, OUTPUT);
  //  digitalWrite(COM_LED, LOW);



  pinMode(WiFi_Button, INPUT_PULLUP);
  //  pinMode(Factory_Button, INPUT_PULLUP);
pinMode(OE2, OUTPUT);
digitalWrite(OE2, LOW);

  pinMode(S0_0, OUTPUT);
  digitalWrite(S0_0, LOW);
  pinMode(S0_1, OUTPUT);
  digitalWrite(S0_1, LOW);
  pinMode(S0_2, OUTPUT);
  digitalWrite(S0_2, LOW);
  pinMode(S0_3, OUTPUT);
  digitalWrite(S0_3, LOW);
  pinMode(S1_0, OUTPUT);
  digitalWrite(S0_0, LOW);
  pinMode(S1_1, OUTPUT);
  digitalWrite(S0_1, LOW);
  pinMode(S1_2, OUTPUT);
  digitalWrite(S1_2, LOW);
  pinMode(S1_3, OUTPUT);
  digitalWrite(S1_3, LOW);
  pinMode(S2_0, OUTPUT);
  digitalWrite(S2_0, LOW);
  pinMode(S2_1, OUTPUT);
  digitalWrite(S2_1, LOW);
  pinMode(S2_2, OUTPUT);
  digitalWrite(S2_2, LOW);
  pinMode(S2_3, OUTPUT);
  digitalWrite(S2_3, LOW);



//  pinMode(FET_Shift_Data, OUTPUT);
//  digitalWrite(FET_Shift_Data, LOW);
//  pinMode(FET_Shift_Clk, OUTPUT);
//  digitalWrite(FET_Shift_Clk, LOW);
//  //  pinMode(FET_shift_clear, OUTPUT);
//  //  digitalWrite(FET_shift_clear, HIGH);
//  pinMode(FET_shift_OE, OUTPUT);
//  digitalWrite(FET_shift_OE, HIGH);
//  FET_SR_control_HIGH_set();



  pinMode(A2D_nCS, OUTPUT); // set the CS pin for A2D
  digitalWrite(A2D_nCS, HIGH);

  pinMode(IO_exp_nCS, OUTPUT); // set the CS pin for I/O Expander
  digitalWrite(IO_exp_nCS, HIGH);

  delay(100);
  IO_reg_init(); // I/O expander init


  //>>>>>>>>>>>>>>>>>>>>Initialize a NTPClient to get time>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>



  // timeClient.begin();
  //timeClient.setTimeOffset(7200);   // initialize a NTP server to 7200 mS


  //  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //  printLocalTime();

  //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


  sensor = 0;
  Sample_counter = 0;







  //  UniqueID = String(WiFi.macAddress());
  //  UniqueID.remove(2,1);
  //  UniqueID.remove(5,1);
  //  Serial.println(UniqueID);

  Serial.println(String("MatrixID FW Ver: ") + FW_Version);
  Serial.println();
  Serial.println(String("Scan mode: ") + EEPROM.read(12));
  Serial.println(String("Full matrix H:") + Full_mtrx_raw + " W:" + Full_mtrx_clmn );
  Serial.println(String("Sampling Matrix H:") + Raw + String(" W:") + Column);
  Serial.println(String("Raw starts on: ") + Raw_start);
  Serial.println(String("X_Offset: ") + x_offset);
  Serial.println(String("Scan Filter values below of: ") + Filter_scan);
  Serial.println(String("Transmitting HTTP Filter is above of: ") + Filter_tx);
  Serial.println(String("Local System ID: ") + UniqueID);
  Serial.println(String("Global System ID: ") + ID_name);
  Serial.println(String("Wifi power: ") + EEPROM.read(23));


  //----------------------------------------- (Server) AP (Access point configuration --------------------------------------------

  WiFi.softAP(host_name, host_password);                // Provide the (SSID, password) //WiFi.softAP(ssid, password, channel, hidden) The first parameter of this function is required, remaining three are optional.

  /*ssid - character string containing network SSID (max. 63 characters)
    password - optional character string with a password. For WPA2-PSK network it should be at least 8 character long. If not specified, the access point will be open for anybody to connect.
    channel - optional parameter to set Wi-Fi channel, from 1 to 13. Default channel = 1.
    hidden - optional parameter, if set to true will hide SSID

    Function will return true or false depending on result of setting the soft-AP.
  */
  WiFi.mode(WIFI_STA);
  /* WiFi Mode:
    -------------------------------
    WIFI_AP - Soft AP (Access point),
    WIFI_STA - be a Station
    WIFI_AP_STA - be Soft AP and a station
    WIFI_OFF -
  */

  Serial.println(String("Code runs on Core: ") + xPortGetCoreID());


  for (int i = 0; i < 5; i++)
  {

    SSID_name_and_pswrd_restore();


    past_connection = 1;
    delay(500);
    Serial.println(String("Connecting to : ") + ssid_reconnect);
    reconnect();
    if (connection_abort_cmnd)
    {
      connection_abort_cmnd = false;
      break;
    }
    //    HTTP_flag = true;
    //    Serial.println(" HTTP mode activated");

    if (Serial.available() > 0) {
      delay (100);
      while (Serial.available() > 0) {

        serial_read = Serial.readStringUntil('\n'); // fill string with received information
      }
      serial_read.toLowerCase();  // Lower case the input information
      if (serial_read == con_abort) {
        connection_abort();
        break;
      }
    }

    if (WiFi.status() == WL_CONNECTED) {
      /*     client.setServer(mqtt_broker, mqtt_port);

           String client_id = String(WiFi.macAddress());
           if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
             Serial.println("Public emqx mqtt broker connected");
           } else {
             Serial.print("failed with state ");
             Serial.print(client.state());
             delay(2000);
           }
      */
      HTTP_flag = true;
      Serial.println(" HTTP mode activated");
      break;
    }
  }

  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);


  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  // FET_switch_init();
  Scan_time_interval_restore();
  URL_restore();
  HTTP_ID_request();
  Unique_ID_restore();


  //  #ifdef ESP32
  //    // Display free heap on startup
  //    Serial.print("Module free Heap at startup: ");
  //    Serial.println(ESP.getFreeHeap());
  //  #endif

  // SPI_LC_send(0xc4,0xF1); // configure input to AIN1
   SPI_LC_send(0xc6,0xDE); // Set Gain to 64
   SPI_LC_send(0xc6,0xDE); // Set Gain to 64
  
  // Serial.println(SPI_LC_send(0xc5,0x00)); // Read register CTRL2

}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                        END OF SETUP FUNCTIONS                                                                                               //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                                      START MAIN LOOP FUNCTION                                                                                              //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



void loop() {
//   digitalWrite(OE2, LOW);

//  Serial.println(SPI.transfer16(0xc700),HEX);
//   digitalWrite(OE2, HIGH);


// SPI_LC_send(0xc7,0x00);

// Serial.println(SPI_LC_send(0xc5,0x00),HEX); // Read register CTRL2
  // SPI_LC_send(0xc6,0xDE);
  // LC_send(0x03,0xDE);
  // delay(50);
  // LC_regRead(0x03);
  Serial.println(LC_dataRead(),HEX);
// Serial.println(SPI_LC_regRead(0x03), HEX);

delay(50);


  //***********************************************************************************************************************************************************************************
  //************************************************************************    SERIAL COMMUNICATION      *****************************************************************************
  //***********************************************************************************************************************************************************************************


  if (Serial.available() > 0) {
    delay (100);  // wait to message arrive

    // read all the available characters
    while (Serial.available() > 0) {

      serial_read = Serial.readStringUntil('\n'); // fill string with received information
    }
    serial_read.toLowerCase();  // Lower case the input information
    if (serial_read == "network") {
      scan_state = true;
      network_scan();
      //network_connect();   Remove comment if needed network connection after network scanning without input command

    } else if (serial_read == con_abort) {

      connection_abort();

    }  else if (serial_read == "status") {


      Status();
      IOregRead();

    } else if (serial_read == "reconnect") {                         // Reconnect to the last connected Network

      reconnect();
    } else if (serial_read == "strng") {
      string_show = !string_show;
      if (string_show) {

        Serial.println("Sending string show activated");
      } else if (scan_show) {
        scan_show = false;
        string_show = true;
      } else {
        scan_show = false;
        Serial.println(" Sending string show deactivated");
      }


    } else if (serial_read == "http") {

      HTTP_flag = true;
      Serial.println();
      Serial.println(" HTTP mode activated");

    } else if (serial_read == "httpstp") {

      HTTP_flag = false;
      Serial.println();
      Serial.println(" HTTP mode deactivated");

    } else if (serial_read == "offset") {
      offset_flag = true;
      Serial.println("Offset initialized");

    } else if (serial_read == "offtst") {

      offtst = true;
    } else if (serial_read == "offset0") {

      Serial.println("Offset reset initialized");
      offset_reset_flag = true;
    } else if (serial_read == "srvfeed") {
      serverfeedback = !serverfeedback;
      Serial.println( "Server feedback print set to: " + String(serverfeedback));
    } else if (serial_read == "1smpl") {

      sample_1 = true;
    } else if (serial_read == "testmode") {
      Testing_Mode = !Testing_Mode;
      if (Testing_Mode)
      {
        Serial.println("Testing mode set to: HIGH");
      } else {
        Serial.println("Testing mode set to: LOW");
      }

    } else if (serial_read.substring(0, 5) == "test$") {
      String Yaxis = serial_read.substring(5, 8);
      String Xaxis = serial_read.substring(8, 10);
      Test_y = Yaxis.toInt();
      Test_x = Xaxis.toInt();
      Serial.println(String("Testing pixel Y:") + Test_y + String(" X:") + Test_x);

    } else if (serial_read == "strngshw") {

      scan_show = !scan_show;
      if (scan_show) {
        string_show = true;
        Serial.println("Sending position scan show activated");
      } else {
        string_show = false;
        Serial.println(" Sending position scan show deactivated");
      }

    } else if (serial_read.substring(0, 6) == "mtrxh$") {
      String  mtrx_raw = serial_read.substring(6);
      EEPROM.write(24, mtrx_raw.toInt());
      EEPROM.commit();
      Raw = EEPROM.read(24);
      delay(100);
      Serial.println();
      Serial.println(String("Scanning raw set to: ") + Raw);
    } else if (serial_read.substring(0, 6) == "mtrxw$") {

      String  mtrx_column = serial_read.substring(6);
      EEPROM.write(25, mtrx_column.toInt());
      EEPROM.commit();
      Column = EEPROM.read(25);
      delay(100);
      Serial.println();
      Serial.println(String("Scanning column set to: ") + Column);
    } else if (serial_read.substring(0, 7) == "mtrxhs$") {

      String  mtrx_raw_strt = serial_read.substring(7);
      EEPROM.write(4, mtrx_raw_strt.toInt());
      EEPROM.commit();
      Raw_start = EEPROM.read(4);
      Raw_start_offset = Raw_start;
      delay(100);
      Serial.println();
      Serial.println(String("Scanning raw start set to: ") + Raw_start);
    } else if (serial_read.substring(0, 7) == "mtrxws$") {

      String  mtrx_clmn_strt = serial_read.substring(7);
      EEPROM.write(5, mtrx_clmn_strt.toInt());
      EEPROM.commit();
      Column_start = EEPROM.read(5);
      Column_start_offset = Column_start;
      x_offset = Column_start;
      Serial.println();
      Serial.println(String("Scanning column(X offset)start set to: ") + x_offset);

    } else if (serial_read.substring(0, 6) == "fullw$") {
      String  full_matrx_x = serial_read.substring(6);
      EEPROM.write(6, full_matrx_x.toInt());
      EEPROM.commit();
      Full_mtrx_clmn = EEPROM.read(6);
      delay(100);
      Serial.println();
      Serial.println(String("Full matrix width set to: ") + Full_mtrx_clmn);

    } else if (serial_read.substring(0, 6) == "fullh$") {
      String  full_matrx_y = serial_read.substring(6);
      EEPROM.write(21, full_matrx_y.toInt());
      EEPROM.commit();
      Full_mtrx_raw = EEPROM.read(21);

      delay(100);
      Serial.println();
      Serial.println(String("Full matrix height set to: ") + Full_mtrx_raw);

    } else if (serial_read.substring(0, 5) == "mode$") {
      if (serial_read.substring(5).toInt() > -1 && serial_read.substring(5).toInt() < 2) {
        EEPROM.write(12, serial_read.substring(5).toInt());
        EEPROM.commit();
        scan_mode = EEPROM.read(12);
        Serial.println();
        Serial.println(String("Scan mode set to: ") + scan_mode);
      } else {
        Serial.println(" Mode set incorrect...");
      }


    } else if (serial_read.substring(0, 7) == "fltrsc$") {
      int  fltrsc = serial_read.substring(7).toInt();

      if (fltrsc > Filter_tx) {
        fltrsc = Filter_tx;
        Serial.println();
        Serial.println("Scan filter is above Transmit filter and will be equilized");
      }
      if (fltrsc <= 1500 && fltrsc >= 0) {
        fltrsc /= 50;
      } else {
        fltrsc = 0;
      }
      EEPROM.write(9, fltrsc);
      EEPROM.commit();
      Filter_scan = (EEPROM.read(9) * 50);
      Serial.println();
      Serial.println(String("Scan filter offset set to: ") + Filter_scan);
      Serial.println("Note: Scan filter values range is (0-1500) with steps of 50");

    } else if (serial_read.substring(0, 7) == "fltrtx$") {
      int  fltrtx = (serial_read.substring(7).toInt());

      if (fltrtx < Filter_scan) {
        fltrtx = Filter_scan;
        Serial.println();
        Serial.println("Transmit filter value is below than Scan filter and will be equilized");
      }
      if (fltrtx >= 0 && fltrtx < 4095)
      {
        fltrtx /= 50;
        EEPROM.write(10, fltrtx);
        EEPROM.commit();
        Filter_tx = (EEPROM.read(10) * 50);
        Serial.println();
        Serial.println(String("Transmit filter offset set to: ") + Filter_tx);
        Serial.println("Note: Transmit filter values are steps of 50");
      }
    } else if (serial_read.substring(0, 7) == "sendid$")
    {
      int  mtrx_tx = (serial_read.substring(7).toInt());

      EEPROM.write(11, mtrx_tx);
      EEPROM.commit();
      Sample_raw = EEPROM.read(11);
      delay(100);
      Serial.println(String("Transmission ID set to: ") + Sample_raw );
      Serial.println();

    } else if (serial_read.substring(0, 6) == "sysid$")
    {
      int  sysid = (serial_read.substring(6).toInt());
      EEPROM.write(22, sysid);
      EEPROM.commit();
      UniqueID = EEPROM.read(22);
      delay(100);
      Serial.println(String("System ID set to: ") + UniqueID );
      Serial.println();
    } else if (serial_read == "smpl") {
      Testing_scan = true;
      sample_2 = true;
    } else if (serial_read == "asmpl") {

      Testing_scan = true;
      sample_2 = true;
      auto_sample = !auto_sample;
      if (auto_sample)
      {
        Serial.println("Auto sample ON");
      } else {
        Serial.println("Auto sample OFF");
      }
    } else if (serial_read == "shw") {

      Serial.println(ScnShwMessage);

    } else if (serial_read == "srate") {

      smple_rate_flag = true;
      Rate_counter = 0;
      sample_time = millis();

    }  else if (serial_read == "connect") {

      net_id = true;
      network_connect();
    } else if (serial_read == "bufmem") {
      bufmem();
    } else if (serial_read == "bufcnst") {

      bufconst = !bufconst;
      Serial.println("buffer constant set to: " + String(bufconst));

    } else if (serial_read == "bufclr") {
      for (int i = 0; i < 120; i++) {
        for (int k = 0; k < 45; k++) {
          buffer_board_flag[i][k] = 0;
        }

      }
      Serial.println("Buffer reset done");
    }  else if (serial_read == "factory") {

      factory_reset();
    }  else if (serial_read == "hfactory") {
      Hard_factory_reset();
    } else if (serial_read == "fet") {
      FET_flag = !FET_flag;
      if (FET_flag)
      {
        Serial.println("filter FET set: HIGH");

      } else {
        Serial.println("filter FET set: LOW");
      }

    } else if (serial_read == "dynprint") {
      dynprint = !dynprint;
      Serial.println ("Dynamic compensation set to: " + String(dynprint));


    } else if (serial_read == "dyncomp") {
      dyncomp = !dyncomp;
      Serial.println ("Dynamic compensation set to: " + String(dyncomp));


    } else if (serial_read == "server") {

      IPAddress HTTPS_ServerIP = WiFi.softAPIP();                       // Obtain the IP of the Server
      Serial.print("Server IP is: "); Serial.println(HTTPS_ServerIP);   // Print the IP to the monitor window


    } else if (serial_read.substring(0, 6) == "power$") {
      int  power_read = (serial_read.substring(6).toInt());
      if (power_read >= 0 && power_read <= 12) {
        wifi_power_set(power_read);
        EEPROM.put(23, power_read);
        EEPROM.commit();
        delay(100);

        Serial.println (String("Wifi Power set to: ") + EEPROM.read(23));

      } else {
        Serial.println ("Wif power range between Low=0 to High=12");
      }


    } else {

      Serial.println ();
      Serial.println (" Wrong command ");
    }

  }

  //  int loop_counter_stop = millis() - loop_counter_start;
  //  Serial.println(String("loop time: ") + loop_counter_stop + String(" mSec"));

  /*deep sleep call*/
  ///////////////////

  //  if (loop_couter==100){
  //   esp_deep_sleep_start();
  //  }
  //  loop_couter++;
  // Serial.println(String("loop counter:") + loop_couter);

}