
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
}

void ReadSwitches(OLED_SW *SwState) //Adafruit_SSD1306 *OLED_Display)
{

    if (digitalRead(BUTTON_A) == 0)
    {
        SwState->Switch_A = 1;
    }
    else
    {
        SwState->Switch_A = 0;
    }

    if (!digitalRead(BUTTON_B))
    {
        SwState->Switch_B = 1;
    }
    else
    {
        SwState->Switch_B = 0;
    }

    if (!digitalRead(BUTTON_C))
    {
        SwState->Switch_C = 1;
    }
    else
    {
        SwState->Switch_C = 0;
    }
}
