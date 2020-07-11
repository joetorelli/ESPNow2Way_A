
#ifndef SETTINGS_H

#define SETTINGS_H
//#include "TFT_eSPI.h" // ESP32 Hardware-specific library
//#include "AdafruitIO_WiFi.h"
#include "network_config.h"
//#include "ezTime.h" //ntp and other functions
//#include "aux_functions.h"
//#include "clock.h" //clock settings
//#include "EEPROM.h"
//#include "esp_system.h"
//#include "mqttController.h"
//#include "SD_Card.h"

//#define UpdateLED 17
//#define TouchLED 16
//#define IFTTTLED 12
//#define EEPROM_SIZE 4
//#define CALIBRATION_FILE "/TFT_Touch_CalData" //touch cal data in SPIFF
#endif

#define DEBUGLOG

#ifdef DEBUGLOG

#define DEBUGPRINT(x) Serial.print(x)
#define DEBUGPRINTDEC(x) Serial.print(x, DEC)
#define DEBUGPRINTLN(x) Serial.println(x)

#else

#define DEBUGPRINT(x)
#define DEBUGPRINTDEC(x)
#define DEBUGPRINTLN(x)

#endif
/**********************************************
  Pin Definitions
**********************************************/

// assign i2c pin numbers
#define I2c_SDA 21 //21 for nodemcu32s   //23 for feather
#define I2c_SCL 22

#define SD_CS 33

#define Remote_LED_Pin 25
#define Local_LED_Pin 26

#define BUTTON_A 15 //oled button
#define BUTTON_B 32 //oled button
#define BUTTON_C 14 //oled button
