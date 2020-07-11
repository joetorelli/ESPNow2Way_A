
#ifndef SENSOR_READINGS_H

#define SENSOR_READINGS_H
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "settings.h"
#include "sensor_readings.h"

#define SEALEVELPRESSURE_HPA (1013.25)

//store sensor values
struct BME_Sensor
{
    float f_temperature = 0;
    float f_humidity = 0;
    float f_pressure = 0;
    float f_altitude = 0;
};

//the state of the switches
struct OLED_SW
{
    int Switch_A = 0;
    int Switch_B = 0;
    int Switch_C = 0;
};

//store the status of leds
struct LEDS
{

    int LED_Remote = 0;
    int LED_Local = 0;
};


//store the status of leds
struct AnalogControl
{

    int PWM_Remote = 0;
    int PWM_Local = 0;
};

//packet to transfer data
struct Packet
{
    float f_temperature = 0;
    float f_humidity = 0;
    float f_pressure = 0;
    float f_altitude = 0;
    int Switch_A = 0;
    int Switch_B = 0;
    int Switch_C = 0;
    int LED = 0;
    int PWMValue = 0;
};
// extern struct OLED_SW Switch_State;
// extern struct BME_Sensor Sensor_Values;

void ReadSensor(Adafruit_BME280 *Sensor, BME_Sensor *SenseVal);
void ReadSwitches(OLED_SW *SwState);
void LED_Indicator(LEDS *Ind);
#endif