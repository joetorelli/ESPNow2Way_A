/*************************************
   adafruit feather esp32
   adalogger SDcard, PCF8523 RTC
   OLED feather wing 128x32
// Date and time functions using a PCF8523 RTC connected via I2C and Wire lib
   purpose: log data
*************************************/

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include "Wire.h"
#include "SRF.h"
#include "OLED.h"
#include "settings.h"        // The order is important!
#include "sensor_readings.h" // The order is important!
#include "network_config.h"

#include "SD_Card.h"
#include <ezTime.h>
#include <TaskScheduler.h>
//#include "RTClib.h"
/**********************************************
  Pin Definitions
**********************************************/

// assign i2c pin numbers
#define I2c_SDA 21   //21 for nodemcu32s   //23 for feather
#define I2c_SCL 22

/*******************   oled display   ******************/
// Declaration for an SSD1306 OLED_Display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 OLED_Display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*******************  rtc  ***************************/
//RTC_PCF8523 rtc;    //used on feather

RTC_DS3231 rtc;
DateTime RTCClock = rtc.now();
Timezone CentralTZ;
struct OLED_SW Switch_State;
struct BME_Sensor Sensor_Values;
struct BME_Sensor incomingReadings;
// Define variables to store BME280 readings to be sent
float temperature;
float humidity;
float pressure;

// Define variables to store incoming readings
float incomingTemp;
float incomingHum;
float incomingPres;
/***********************   bme280 i2c sensor   **********/
Adafruit_BME280 bme;
// BME280_ADDRESS = 0x77
// BME280_ADDRESS_ALTERNATE = 0x76

/********  tasks callback functions  *********/
void sensor_update();
//void clock_update();
void SD_Update();
void getReadings();
/***************  task scheduler  **************************/
Task t1_Update(1000, TASK_FOREVER, &sensor_update); //can not pass vars with pointer in this function
//Task t2_clock(1000, TASK_FOREVER, &clock_update);
Task t3_SDCard(10000, TASK_FOREVER, &SD_Update);
//Task t5_indicators(2000, TASK_FOREVER, &indicators);
Scheduler runner;

/*************************  srf08   ********************/
struct SRFRanges SRFDist;
int Light = 0;

/*************************   esp_NOW   ******************************/

// MAC address of RECEIVER
uint8_t broadcastAddress[] {0x30, 0xAE, 0xA4, 0x46, 0xF0, 0xE4};
String ESPNOW_Success;
void updateDisplay();
// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status ==0){
    ESPNOW_Success = "Delivery Success :)";
  }
  else{
    ESPNOW_Success = "Delivery Fail :(";
  }
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  incomingTemp = incomingReadings.f_temperature;
  incomingHum = incomingReadings.f_humidity;
  incomingPres = incomingReadings.f_pressure;
}

/***********************   setup   *********************/
void setup()
{
  Serial.begin(115200);

  /*********   init i2c  *********/
  Wire.begin(I2c_SDA, I2c_SCL);
  bool status; // connect status
  DEBUGPRINTLN("I2C INIT OK");

  /************* set up task runner  *************/
  runner.init();
  runner.addTask(t1_Update); //for sensors
  //runner.addTask(t2_clock);
  runner.addTask(t3_SDCard); //for SD card
  t1_Update.enable();        //for sensors
  //t2_clock.enable();
  t3_SDCard.enable(); //for SD card

  /********************* oled  ********************/
  // SSD1306_SWITCHCAPVCC = generate OLED_Display voltage from 3.3V internally
  if (!OLED_Display.begin(SSD1306_SWITCHCAPVCC, 0x3c)) // Address 0x3C for 128x32
  {
    DEBUGPRINTLN(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }
  else
  {
    DEBUG_PRINTLN("SSD1306 Init");
  }

  // Clear the oled buffer.
  OLED_Display.clearDisplay();
  OLED_Display.display();
  OLED_Display.setTextSize(1);
  OLED_Display.setTextColor(SSD1306_WHITE);

  //buttons on OLED board
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  /**********************   wifi   ***********************/

// Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);


  /*
  DEBUGPRINT("Connect to SSID: ");
  DEBUGPRINTLN(WIFI_SSID);
  DEBUGPRINT("Waiting for Network:");

  OLED_Display.setCursor(0, 0);
  OLED_Display.println("Connecting to SSID:"); //line 1
  OLED_Display.println(WIFI_SSID);             //line 2
  OLED_Display.print("Waiting for Network:");
  OLED_Display.println("");
  OLED_Display.display();

  byte count = 0; //used for network and ntp timeout

  //connect to wifi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  //wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    DEBUGPRINT(".");
    count++;
    OLED_Display.print(".");
    OLED_Display.display();

    if (count > 20) //if not connected reboot
    {
      OLED_Display.clearDisplay();
      OLED_Display.display();
      OLED_Display.print("Time out Restarting");
      OLED_Display.display();
      delay(1000);
      break;
    }
  }

  if (count > 20) //if not connected reboot
  {
    ESP.restart();
  }
  else //continue
  {
    count = 0;
  }

  DEBUGPRINTLN("");
  DEBUGPRINTLN("WIFI Connected");
  DEBUGPRINT("IP ADR:");
  DEBUGPRINTLN(WiFi.localIP());

  //display connection on oled
  OLED_Display.println(""); //line 3
  OLED_Display.print("Connected");
  OLED_Display.println("IP:");
  OLED_Display.print(WiFi.localIP());
  OLED_Display.display();
  delay(500);
  OLED_Display.clearDisplay();
  OLED_Display.display();

 // *********************  ntp   *****************
  DEBUGPRINT("Waiting for NTP:");
  OLED_Display.setCursor(0, 0);
  OLED_Display.println("Waiting for NTP:");
  do
  {
    count++;
    DEBUGPRINT(".");
    OLED_Display.print(".");
    OLED_Display.display();

    if (count > 10) //if not connected reboot
    {
      OLED_Display.clearDisplay();
      OLED_Display.print("Time out Restarting");
      OLED_Display.display();
      delay(1000);
      break;
    }
  } while (!waitForSync(1));

  if (count > 10) //reboot
  {
    ESP.restart();
  }
  else //contimue
  {
    count = 0;
  }

  // get ntp time
  DEBUGPRINTLN("UTC: " + UTC.dateTime());
  CentralTZ.setLocation("America/Chicago");
  String NTPTime = CentralTZ.dateTime("Y/M/d H:i:s");
  DEBUGPRINTLN("NTP: " + NTPTime);
  OLED_Display.println();
  OLED_Display.println(NTPTime);
*/
  /*******************  rtc  **************************/
  //convert string from ntp to int for rtc
  String Y = CentralTZ.dateTime("Y");
  int Year = Y.toInt();
  String M = CentralTZ.dateTime("m");
  int Month = M.toInt();
  String D = CentralTZ.dateTime("d");
  int Day = D.toInt();
  String H = CentralTZ.dateTime("H");
  int Hour = H.toInt();
  String m = CentralTZ.dateTime("i");
  int Min = m.toInt();
  String S = CentralTZ.dateTime("s");
  int Sec = S.toInt();

  //init rtc
  if (!rtc.begin())
  {
    DEBUGPRINTLN("Couldn't find RTC");

    OLED_Display.clearDisplay();
    OLED_Display.print("Couldn't find RTC");

    rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec));
    DEBUGPRINTLN("Clock Set");
    OLED_Display.print("RTC set to NTP");
    OLED_Display.display();

    delay(1000);
    ESP.restart();
  }

  else
  {
    DEBUGPRINTLN("RTC Init");
    DEBUGPRINTLN("RTC Running - Updating Clock to NTP");
    rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec));
    DEBUGPRINTLN("Clock Set");
    OLED_Display.print("RTC set to NTP");
  }

  //used with feather clock pcf8523 does not work with ds3231
  // if (!rtc.initialized())
  // {
  //   //update rtc with ntp time
  //   DEBUGPRINTLN("RTC is NOT running! - Setting Clock to NTP");
  //   rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec));
  //   DEBUGPRINTLN("Clock Set");
  //   OLED_Display.print("RTC set to NTP");
  // }

  // else
  // {
  //   //update rtc with ntp time
  //   DEBUGPRINTLN("RTC Running - Updating Clock to NTP");
  //   rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec));
  //   DEBUGPRINTLN("Clock Set");
  //   OLED_Display.print("RTC set to NTP");
  // }

  //stop asking for internet ntp time
  void setInterval(uint16_t seconds = 0);

  /**********  init i2c sensor  ************/
  OLED_Display.print("Init Sensor");

  status = bme.begin(BME280_ADDRESS_ALTERNATE); // get status of sensor

  if (!status) // test status
  {
    OLED_Display.print("Can't find BME280");
    DEBUGPRINTLN("Can't find BME280, it may have fell on the floor");
    //while (1);
  }
  else
  {
    OLED_Display.print("Found BME280");
    DEBUGPRINTLN("Found BME280");
  }
  OLED_Display.display();
  delay(1000);


  /*
  ********************  SD Card  ************************
  SD.begin(SD_CS);
  if (!SD.begin(SD_CS))
  {
    DEBUGPRINTLN("SD Card failed");
    return;
  }

  uint8_t CardType = SD.cardType();

  if (CardType == CARD_NONE)
  {
    DEBUGPRINTLN("No SD Card Found");
    return;
  }

  DEBUGPRINT("SD Card Type: ");

  if (CardType == CARD_MMC)
  {
    DEBUGPRINTLN("MMC");
  }
  else if (CardType == CARD_SD)
  {
    DEBUGPRINTLN("SCSC");
  }
  else if (CardType == CARD_SDHC)
  {
    DEBUGPRINTLN("SDHC");
  }
  else
  {
    DEBUGPRINTLN("Unkown Type");
  }
  String Tempp;

  //   uint64_t CardSize = SD.cardSize() / (1024 * 1024);
  //   Serial.printf("SD Card Size: %lluMB\n", CardSize);
  //   Serial.printf("Total Space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
  //   Serial.printf("Used Space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
  //   listDir(SD, "/", 0);
*/
  /*****************************   srf08   *************************************/
  //srf max reliable range = 6meters
  //set max range
  /*
  Wire.beginTransmission(SRF_ADDR); //transmit to device
  Wire.write(byte(SRF_ECHO_H));     //range register
  Wire.write(byte(0x8C));           //max range 0x00=43mm,0x01=86mm,0x18=1000mm,0x8C=6000mm
  Wire.endTransmission();
*/
  //set gain
  // Wire.beginTransmission(0x70);    //transmit to device
  // Wire.write(byte(0x01));          //gain register
  // Wire.write(byte(0x00));          //max range 0x00=43mm,0x01=86mm,0x18=1000mm,0x8C=6000mm
  // Wire.endTransmission();
  // tell sensor to read echos

  delay(1000);
}

/****************   loop   ********************/
void loop()
{
  // start task manager
  runner.execute();
  //events();

  DEBUGPRINTLN("Read clock");
  RTCClock = rtc.now(); //read hardware clock

  if (secondChanged())
  {
    OLED_Display.clearDisplay();
  }

  OLED_Display.setCursor(0, 0);

  DEBUGPRINTLN("Read switches");
  ReadSwitches(&Switch_State);
/*
  DEBUGPRINTLN("Read Ping");
  SRFPing();
  Light = SRFLight();
  SRFDistance(&SRFDist);
*/
  DEBUGPRINTLN("Display Switches");
  DisplaySwitches(&OLED_Display, &Switch_State);

  OLED_Time(&OLED_Display, &RTCClock);
  OLED_Date(&OLED_Display, &RTCClock);
  //OLED_Day(&OLED_Display, &RTCClock);

  DEBUGPRINTLN("Display Env sensor");
  DisplaySensor(&OLED_Display, &Sensor_Values);

  DEBUGPRINTLN("Display SFR Sensor");
  OLED_Light(&OLED_Display, Light);
  OLED_Range(&OLED_Display, &SRFDist);

  OLED_Display.display(); // update OLED_Display
                          //delay(2000);


getReadings();
 
  // Set values to send
  Sensor_Values.f_temperature = temperature;
  Sensor_Values.f_humidity = humidity;
  Sensor_Values.f_pressure = pressure;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &Sensor_Values, sizeof(Sensor_Values));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  updateDisplay();
  delay(10000);






}


void getReadings(){
  temperature = bme.readTemperature();
  humidity = bme.readHumidity();
  pressure = (bme.readPressure() / 100.0F);
}

void updateDisplay(){
  // Display Readings on OLED Display
  OLED_Display.clearDisplay();
  OLED_Display.setTextSize(1);
  OLED_Display.setTextColor(WHITE);
  OLED_Display.setCursor(0, 0);
  OLED_Display.println("INCOMING READINGS");
  OLED_Display.setCursor(0, 15);
  OLED_Display.print("Temperature: ");
  OLED_Display.print(incomingTemp);
  OLED_Display.cp437(true);
  OLED_Display.write(248);
  OLED_Display.print("C");
  OLED_Display.setCursor(0, 25);
  OLED_Display.print("Humidity: ");
  OLED_Display.print(incomingHum);
  OLED_Display.print("%");
  OLED_Display.setCursor(0, 35);
  OLED_Display.print("Pressure: ");
  OLED_Display.print(incomingPres);
  OLED_Display.print("hPa");
  OLED_Display.setCursor(0, 56);
  OLED_Display.print(ESPNOW_Success);
  OLED_Display.display();
  
  // Display Readings in Serial Monitor
  Serial.println("INCOMING READINGS");
  Serial.print("Temperature: ");
  Serial.print(incomingReadings.f_temperature);
  Serial.println(" ºC");
  Serial.print("Humidity: ");
  Serial.print(incomingReadings.f_humidity);
  Serial.println(" %");
  Serial.print("Pressure: ");
  Serial.print(incomingReadings.f_pressure);
  Serial.println(" hPa");
  Serial.println();
}




void sensor_update()
{
  DEBUGPRINTLN("Read sensor***********");
  ReadSensor(&bme, &Sensor_Values);
}

void SD_Update()
{

  DEBUGPRINTLN("Write SD**************");
  Refresh_SD(&RTCClock, &Sensor_Values);
}
