/*************************************
adafruit feather esp32
    
  adalogger SDcard - RTC clock and SD card 
    PCF8523 (I2C) RTC on Feather - working
    SD (SPI) - working

  OLED feather wing 128x32(I2C) - working

  Motor Control Wing (PWM)- working
    1 stepper or 2 DC motor

  Current Sensor Wing (I2C)- working 

  Added BME280 (I2C)- Working
    put on Current Monitor Feather

  switches - working

  eztime - working
    for ntp and time functions
    read on startup and set RTC
    
  taskScheduler-timer to run tasks

  esp_now - working
    sending packet back and forth

nodemcu32s

  128x64 OLED (I2C)- working
  added DS3231 (I2C) RTC - rtclib works for both - working

  SD card ****TBD

  Motor Control ****TBD

  srf08 (I2C)- working
    reads first 3 ranges
    prints out first
    also reads ambient light

  BME280 (I2C)- working

  switches working
    press awitch and led on other board lights

  analog input working
    Used LDR analog input to control PWM Out

  PWM working
    Controlled LED by LDR input

  VL53l0x Time of Flight Sensor - working
    used movingAvg Lib to get stable readings

  eztime - 
    for ntp and time functions
    read on startup to update RTC

  taskScheduler - working
    timer to run task
  
  esp_now working
    sending packet back and forth


From TechExplore 
  ESP for busy people
  ESP Unleashed

purpose: turd control, log data, remote sensing

*************************************/

#include "NetWork.h"
#include "settings.h"        // The order is important!
#include "sensor_readings.h" // The order is important!
#include "network_config.h"

#include "OLED.h"
#include "RTClib.h"
#include "SD_Card.h"
#include "SRF.h"
#include "Adafruit_VL53L0X.h"

#include <ezTime.h>
#include <TaskScheduler.h>
#include <movingAvg.h>

/*******************   oled display   ******************/
// Declaration for an SSD1306 OLED_Display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 OLED_Display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*******************  rtc  ***************************/
//RTC_PCF8523 rtc;    //used on feather

RTC_DS3231 rtc;                //select rtc chip  nodemcu32s
DateTime RTCClock = rtc.now(); //create clock obj
Timezone CentralTZ;            //describe timezone

/***********************   bme280 i2c sensor   **********/
Adafruit_BME280 bme; //sensor obj
// BME280_ADDRESS = 0x77
// BME280_ADDRESS_ALTERNATE = 0x76

/*************************  srf08   ********************/
struct SRFRanges SRFDist; //3 first returns
int Light = 0;            //light sensor value

/************************   analog control   ***************************/
const byte LightSensorPin = 36;
const byte LEDPWMPin = 33;
const byte PWMChannel = 0;
//int AnalogValue = 0;

/*************************  TOF VL53L0X   *************/
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;
movingAvg TOFMovingAvg(15); // define the moving average object
int TOFRangeValue = 0;
int TOFAvgValue = 0;
void GetTOF();
/*****************   structures   **************************/
struct OLED_SW LocalSwitch;       //state of onboard switch
struct OLED_SW RemoteSwitch;      //state of remote switch
struct BME_Sensor LocalReadings;  //onboard enviro sensor reading
struct BME_Sensor RemoteReadings; //remote enviro sensors reading
struct LEDS StatusLED;            //state of on board LED
struct AnalogControl PWMControl;

struct Packet Transmit_Pkt; //packet sent to remote
struct Packet Receive_Pkt;  //packet received from remote

/********  tasks callback functions  *********/
// functions will be called by the task manager
void sensor_update(); //get sensor readings
void clock_update();  //get time
void SD_Update();     //store to SD card
void updateDisplay(); //update oled run inside clock_update

/***************  task scheduler  **************************/
// named_task(run every ms, repeat, called function)
//can not pass vars with pointer in this function
Task t1_Update(1000, TASK_FOREVER, &sensor_update);
Task t2_clock(500, TASK_FOREVER, &clock_update);
//Task t3_SDCard(10000, TASK_FOREVER, &SD_Update);
//Task t5_indicators(2000, TASK_FOREVER, &indicators);
Scheduler runner; //schrduler obj

/*************************   esp_NOW   ******************************/

// MAC address of RECEIVER
//uint8_t broadcastAddress[]{0x30, 0xAE, 0xA4, 0x46, 0xF0, 0xE4}; // Unit A = outside sender 1
uint8_t broadcastAddress[]{0xA4, 0xCF, 0x12, 0x0B, 0x2B, 0xB4}; // Unit B = outside sender 2

String ESPNOW_Success; //vars to hold connect string

// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
  DEBUGPRINT("\r\nLast Packet Send Status:\t");
  DEBUGPRINTLN(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
  if (status == 0)
  {
    ESPNOW_Success = "Delivery Success :)";
  }
  else
  {
    ESPNOW_Success = "Delivery Fail :(";
  }
}

// Callback when data is received
//
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
  memcpy(&Receive_Pkt, incomingData, sizeof(Receive_Pkt)); //move incomming data to Receive_Pkt
  DEBUGPRINT("Bytes received: ");
  DEBUGPRINTLN(len);

  //move incomming packet to remote structures
  RemoteReadings.f_temperature = Receive_Pkt.f_temperature;
  RemoteReadings.f_humidity = Receive_Pkt.f_humidity;
  RemoteReadings.f_pressure = Receive_Pkt.f_pressure;
  RemoteReadings.f_altitude = Receive_Pkt.f_altitude;
  RemoteSwitch.Switch_A = Receive_Pkt.Switch_A;
  RemoteSwitch.Switch_B = Receive_Pkt.Switch_B;
  RemoteSwitch.Switch_C = Receive_Pkt.Switch_C;
  StatusLED.LED_Remote = Receive_Pkt.LED;
  PWMControl.PWM_Remote = Receive_Pkt.PWMValue;
}

/***********************   setup   *********************/
void setup()
{

  Serial.begin(115200);

  /*********   init i2c  *********/
  Wire.begin(I2c_SDA, I2c_SCL);
  bool status; // connect status
  DEBUGPRINTLN("I2C INIT OK");

  /**************   analog control   ***************/
  ledcAttachPin(LEDPWMPin, PWMChannel);
  ledcSetup(PWMChannel, 400, 12); //set to 12 so analog read and pwm is same bit width

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
    DEBUGPRINTLN("SSD1306 Init");
  }

  // Clear the oled buffer.
  OLED_Display.clearDisplay();
  OLED_Display.display();
  OLED_Display.setTextSize(1);
  OLED_Display.setTextColor(SSD1306_WHITE);

  /**********  init i2c temp sensor  ************/
  OLED_Display.print("Init Sensor");

  // get status of sensor
  status = bme.begin(BME280_ADDRESS_ALTERNATE);

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
  //buttons/LEDs on OLED board

  /*****************************   TOF VL53L0X   ******************************/
  Serial.println("Adafruit VL53L0X test");
  if (!lox.begin())
  {
    Serial.println(F("Failed to boot VL53L0X"));
    while (1)
      ;
  }

  TOFMovingAvg.begin();
  /************************   switches and leds   ******************************/
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  pinMode(Remote_LED_Pin, OUTPUT);
  pinMode(Local_LED_Pin, OUTPUT);

  /**********************   wifi   ***********************/

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK)
  {
    DEBUGPRINTLN("Error initializing ESP-NOW");
    return;
  }
  else
  {
    DEBUGPRINTLN("initi ESP-NOW OK");
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
  if (esp_now_add_peer(&peerInfo) != ESP_OK)
  {
    DEBUGPRINTLN("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  //connect to wifi
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

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  //wait for connection
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    DEBUGPRINT(".");
    count++;
    OLED_Display.print(".");
    OLED_Display.display();

    if (!digitalRead(BUTTON_A)) //bypass wifi if button pressed
    {
      count = 0;
      break;
    }

    if (count > 20) //if not connected reboot
    {
      OLED_Display.clearDisplay();
      OLED_Display.display();
      OLED_Display.print("Time out Restarting");
      OLED_Display.display();
      delay(1000);
      ESP.restart();
    }
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
  delay(1000);
  OLED_Display.clearDisplay();
  OLED_Display.display();

  // *********************  ntp   *****************
  DEBUGPRINT("Waiting for NTP:");
  OLED_Display.setCursor(0, 0);
  OLED_Display.println("Waiting for NTP:");

  //connection loop
  do
  {
    delay(500);
    count++;
    DEBUGPRINT(".");
    OLED_Display.print(".");
    OLED_Display.display();

    if (!digitalRead(BUTTON_A)) //bypass wifi if button pressed
    {
      count = 0;
      break;
    }

    if (count > 20) //if not connected reboot
    {
      OLED_Display.clearDisplay();
      OLED_Display.print("Time out Restarting");
      OLED_Display.display();
      delay(1000);
      ESP.restart();
    }
  } while (!waitForSync(1)); //sync from time server

  // get ntp time
  DEBUGPRINTLN("UTC: " + UTC.dateTime());
  CentralTZ.setLocation("America/Chicago");
  String NTPTime = CentralTZ.dateTime("Y/M/d H:i:s");
  DEBUGPRINTLN("NTP: " + NTPTime);
  OLED_Display.println();
  OLED_Display.println(NTPTime);
  delay(1000);

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

    // rtc.adjust(DateTime(Year, Month, Day, Hour, Min, Sec)); //set rtc to npt
    // DEBUGPRINTLN("Clock Set");
    // OLED_Display.print("RTC set to NTP");
    // OLED_Display.display();

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
    delay(1000);
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

  /************* set up task runner  *************/
  runner.init();
  runner.addTask(t1_Update); //for sensors
  runner.addTask(t2_clock);  // clock and display
  //runner.addTask(t3_SDCard);                //for SD card

  t1_Update.enable(); //for sensors
  t2_clock.enable();  //for clock and display
  //t3_SDCard.enable(); //for SD card

  //stop asking for internet ntp time
  void setInterval(uint16_t seconds = 0);

  //delay(1000);
}

/****************   loop   ********************/
void loop()
{
  // start task manager
  runner.execute();

  //events();

  // if (secondChanged())
  // {
  //   //OLED_Display.clearDisplay();
  // }

  //DEBUGPRINTLN("Read switches");
  ReadSwitches(&LocalSwitch);
  LED_Indicator(&StatusLED);

  //analog control
  //read light sensor analog value  and send to PWM
  PWMControl.PWM_Local = analogRead(LightSensorPin); //assign analog value to pwm
  DEBUGPRINT("Analog LightInput: ");
  DEBUGPRINTLN(PWMControl.PWM_Local);
  ledcWrite(PWMChannel, PWMControl.PWM_Remote); //write pwm value


  GetTOF();

  /*
  DEBUGPRINTLN("Read Ping");
  //SRFPing();
  //Light = SRFLight();
  //SRFDistance(&SRFDist);
*/

  //used with oled feather
  //DEBUGPRINTLN("Display Switches");
  //DisplaySwitches(&OLED_Display, &Switch_State);

  //OLED_Time(&OLED_Display, &RTCClock);
  //OLED_Date(&OLED_Display, &RTCClock);
  //OLED_Day(&OLED_Display, &RTCClock);

  //DEBUGPRINTLN("Display Env sensor");
  //DisplaySensor(&OLED_Display, &LocalReadings);

  //DEBUGPRINTLN("Display SFR Sensor");
  //OLED_Light(&OLED_Display, Light);
  //OLED_Range(&OLED_Display, &SRFDist);

  //OLED_Display.display(); // update OLED_Display
  //delay(2000);

  //move local structures  to remote packet
  Transmit_Pkt.f_temperature = LocalReadings.f_temperature;
  Transmit_Pkt.f_humidity = LocalReadings.f_humidity;
  Transmit_Pkt.f_pressure = LocalReadings.f_pressure;
  Transmit_Pkt.f_altitude = LocalReadings.f_altitude;
  Transmit_Pkt.Switch_A = LocalSwitch.Switch_A;
  Transmit_Pkt.Switch_B = LocalSwitch.Switch_B;
  Transmit_Pkt.Switch_C = LocalSwitch.Switch_C;
  Transmit_Pkt.LED = StatusLED.LED_Local;
  Transmit_Pkt.PWMValue = PWMControl.PWM_Local;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&Transmit_Pkt, sizeof(Transmit_Pkt));

  if (result == ESP_OK)
  {
    DEBUGPRINTLN("Sent data with success");
  }
  else
  {
    DEBUGPRINTLN("Error sending the data");
  }

  // updateDisplay();
  //delay(1000);
}

/////////////////////////////////////////////////////////////////////////////////////////////
void GetTOF()
{

// time of flight
//Serial.println("Reading a measurement... ");
lox.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

if (measure.RangeStatus != 4)
{ // phase failures have incorrect data

    TOFRangeValue = measure.RangeMilliMeter;
    TOFAvgValue = TOFMovingAvg.reading(measure.RangeMilliMeter);
}
else
{
    Serial.println(" out of range ");
}


}
void updateDisplay()
{
  // Display Readings on OLED Display
  OLED_Display.clearDisplay();
  OLED_Display.setCursor(0, 0);
  OLED_Display.setTextSize(1);
  OLED_Display.setTextColor(WHITE);
  OLED_Display.print("      Lcl     Rmt");
  OLED_Display.setCursor(0, 15);

  OLED_Display.print("Temp: ");
  OLED_Display.print(LocalReadings.f_temperature);
  OLED_Display.print("   ");
  OLED_Display.print(RemoteReadings.f_temperature);
  // OLED_Display.cp437(true);
  // OLED_Display.write(248);
  // OLED_Display.print("C");
  OLED_Display.setCursor(0, 25);
  OLED_Display.print("Hum:  ");
  //OLED_Display.print();
  OLED_Display.print(LocalReadings.f_humidity);
  OLED_Display.print("   ");
  OLED_Display.print(RemoteReadings.f_humidity);
  //OLED_Display.print("%");
  OLED_Display.setCursor(0, 35);
  OLED_Display.print("Pres: ");
  OLED_Display.print(LocalReadings.f_pressure);
  OLED_Display.print("  ");
  OLED_Display.print(RemoteReadings.f_pressure);
  //OLED_Display.print("hPa");
  OLED_Display.setCursor(0, 56);
  OLED_Display.print(ESPNOW_Success);
  if (!Transmit_Pkt.Switch_A)
  {
    OLED_Display.print("*");
  }
  else
  {
    OLED_Display.print(" ");
  }

  OLED_Display.display();
  /*
  // Display Readings in Serial Monitor
  Serial.println("      Rmt    Lcl");
  Serial.print("Temp: ");
  Serial.print(LocalReadings.f_temperature);
  Serial.print("  ");
  Serial.print(RemoteReadings.f_temperature);
  //Serial.println(" ºC");
  Serial.print("Humd: ");
  Serial.print(LocalReadings.f_humidity);
  Serial.print("  ");
  Serial.print(RemoteReadings.f_humidity);
  //Serial.println(" %");
  Serial.print("Pres: ");
  Serial.print(LocalReadings.f_pressure);
  Serial.print("  ");
  Serial.print(RemoteReadings.f_pressure);
  //Serial.println(" hPa");
  Serial.println();
*/
}

void clock_update()
{

  DEBUGPRINTLN("Read clock");
  RTCClock = rtc.now(); //read hardware clock
  updateDisplay();
}

void sensor_update()
{
  DEBUGPRINTLN("Read sensor***********");
  ReadSensor(&bme, &LocalReadings);


  Serial.print("Distance (mm): ");
  Serial.print(TOFRangeValue);
  Serial.print("   Avg (mm): ");
  Serial.println(TOFAvgValue);
}

void SD_Update()
{

  DEBUGPRINTLN("Write SD**************");
  //Refresh_SD(&RTCClock, &LocalReadings);
}
