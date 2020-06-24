/*************************************
   adafruit feather esp32
   adalogger SDcard, PCF8523 RTC
   OLED feather wing 128x32
// Date and time functions using a PCF8523 RTC connected via I2C and Wire lib
   purpose: log data
*************************************/

#include <Arduino.h>
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

/**********************************************
  Pin Definitions
**********************************************/

// assign i2c pin numbers
#define I2c_SDA 23
#define I2c_SCL 22

/*******************   oled display   ******************/
// Declaration for an SSD1306 OLED_Display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 OLED_Display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*******************  rtc  ***************************/
RTC_PCF8523 rtc;
DateTime RTCClock = rtc.now();
Timezone CentralTZ;
struct OLED_SW Switch_State;
struct BME_Sensor Sensor_Values;

/***********************   bme280 i2c sensor   **********/
Adafruit_BME280 bme;
// BME280_ADDRESS = 0x77
// BME280_ADDRESS_ALTERNATE = 0x76

/********  tasks callback functions  *********/
void sensor_update();
//void clock_update();
void SD_Update();

/***************  task scheduler  **************************/
Task t1_Update(1000, TASK_FOREVER, &sensor_update); //can not pass vars with pointer in this function
//Task t2_clock(1000, TASK_FOREVER, &clock_update);
Task t3_SDCard(10000, TASK_FOREVER, &SD_Update);
//Task t5_indicators(2000, TASK_FOREVER, &indicators);
Scheduler runner;

/*************************  srf08   ********************/
struct SRFRanges SRFDist;
int Light = 0;

/*******************************************************/
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

  /**********************  ntp   ******************/
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
    OLED_Display.display();

    delay(1000);
    ESP.restart();
  }

  else
  {
    DEBUGPRINTLN("RTC Init");
  }

  if (!rtc.initialized())
  {
    //update rtc with ntp time
    DEBUGPRINTLN("RTC is NOT running! - Setting Clock to NTP");
    rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec));
    DEBUGPRINTLN("Clock Set");
    OLED_Display.print("RTC set to NTP");
  }

  else
  {
    //update rtc with ntp time
    DEBUGPRINTLN("RTC Running - Updating Clock to NTP");
    rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec));
    DEBUGPRINTLN("Clock Set");
    OLED_Display.print("RTC set to NTP");
  }

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

  /*********************  SD Card  *************************/
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

  /*****************************   srf08   *************************************/
  //srf max reliable range = 6meters
  //set max range
  Wire.beginTransmission(SRF_ADDR); //transmit to device
  Wire.write(byte(SRF_ECHO_H));     //range register
  Wire.write(byte(0x8C));           //max range 0x00=43mm,0x01=86mm,0x18=1000mm,0x8C=6000mm
  Wire.endTransmission();

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

  DEBUGPRINTLN("Read Ping");
  SRFPing();
  Light = SRFLight();
  SRFDistance(&SRFDist);

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
