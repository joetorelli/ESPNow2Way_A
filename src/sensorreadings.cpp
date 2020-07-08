
#include <Arduino.h>
#include "sensor_readings.h"
//#include "OLED.h"
// //#include "mqttController.h"

//BME_Sensor *ReadSensor(Adafruit_BME280 *bme) //, Adafruit_SSD1306 *OLED_Display)
void ReadSensor(Adafruit_BME280 *Sensor, BME_Sensor *SenVal)
{

    //read sensor ad load vars
    SenVal->f_temperature = Sensor->readTemperature();
    SenVal->f_humidity = Sensor->readHumidity();
    SenVal->f_pressure = Sensor->readPressure() / 100.0F;
    SenVal->f_altitude = Sensor->readAltitude(SEALEVELPRESSURE_HPA);

    //     if (digitalRead(BUTTON_A) == 0)
    // {
    //    SenVal->Switch_A = LocalReadings.Switch_A;
    // }
    // else
    // {
    //     SenVal->Switch_A = 0;
    // }

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

    if (digitalRead(BUTTON_A) == 0)
    {
        SwState->Switch_A = 1;
        Serial.println("A=1");
    }
    else
    {
        SwState->Switch_A = 0;
        Serial.println("A=0");
    }

    if (!digitalRead(BUTTON_B))
    {
        SwState->Switch_B = 1;
        Serial.println("B=1");
    }
    else
    {
        SwState->Switch_B = 0;
        Serial.println("B=0");
    }

    if (!digitalRead(BUTTON_C))
    {
        SwState->Switch_C = 1;
        Serial.println("C=1");
    }
    else
    {
        SwState->Switch_C = 0;
        Serial.println("C=0");
    }
}

void LED_Indicator(LEDS *Ind)
{
   if (digitalRead(BUTTON_A) == 0)
    {
        digitalWrite(Local_LED, 1);
        Ind->LED_L = 1;
        Serial.println("Local LED ON");
    }
    else
    {
        digitalWrite(Local_LED, 0);
        Ind->LED_L = 0;
        Serial.println("Local LED OFF");
    }


if (Ind->LED_R)
{
    digitalWrite(Remote_LED, 1);
    Serial.println("Remote LED ON");
}
else
{
    digitalWrite(Remote_LED, 0);
    Serial.println("Remote LED OFF");
}


}