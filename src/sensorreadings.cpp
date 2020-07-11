
#include <Arduino.h>
#include "sensor_readings.h"
//#include "OLED.h"
// //#include "mqttController.h"

void ReadSensor(Adafruit_BME280 *Sensor, BME_Sensor *SenVal)
{

    //read sensor ad load vars
    SenVal->f_temperature = Sensor->readTemperature();
    SenVal->f_humidity = Sensor->readHumidity();
    SenVal->f_pressure = Sensor->readPressure() / 100.0F;
    SenVal->f_altitude = Sensor->readAltitude(SEALEVELPRESSURE_HPA);

    DEBUGPRINTLN("ReadSensor: ");
    DEBUGPRINT("f_temp: ");
    DEBUGPRINTLN(SenVal->f_temperature);
    DEBUGPRINT("f_humid: ");
    DEBUGPRINTLN(SenVal->f_humidity);
    DEBUGPRINT("f_pres: ");
    DEBUGPRINTLN(SenVal->f_pressure);
}

void ReadSwitches(OLED_SW *SwState) //Adafruit_SSD1306 *OLED_Display)
{

    if (!digitalRead(BUTTON_A))
    {
        SwState->Switch_A = 1;
        DEBUGPRINTLN("A=1");
    }
    else
    {
        SwState->Switch_A = 0;
        DEBUGPRINTLN("A=0");
    }

    if (!digitalRead(BUTTON_B))
    {
        SwState->Switch_B = 1;
        DEBUGPRINTLN("B=1");
    }
    else
    {
        SwState->Switch_B = 0;
        DEBUGPRINTLN("B=0");
    }

    if (!digitalRead(BUTTON_C))
    {
        SwState->Switch_C = 1;
        DEBUGPRINTLN("C=1");
    }
    else
    {
        SwState->Switch_C = 0;
        DEBUGPRINTLN("C=0");
    }
}

void LED_Indicator(LEDS *Ind)
{
    if (digitalRead(BUTTON_A))
    {
        digitalWrite(Local_LED_Pin, 1);
        Ind->LED_Local = 1;
        DEBUGPRINTLN("Local LED ON");
    }
    else
    {
        digitalWrite(Local_LED_Pin, 0);
        Ind->LED_Local = 0;
        DEBUGPRINTLN("Local LED OFF");
    }

    if (Ind->LED_Remote)
    {
        digitalWrite(Remote_LED_Pin, 1);
        DEBUGPRINTLN("Remote LED ON");
    }
    else
    {
        digitalWrite(Remote_LED_Pin, 0);
        DEBUGPRINTLN("Remote LED OFF");
    }
}