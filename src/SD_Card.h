#ifndef SD_CARD_H

#define SD_CARD_H
#include <Arduino.h>
//#include "SD_Card.h"
#include "SD.h"
#include "RTClib.h"
#include "sensor_readings.h"
#include "settings.h"

#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#endif
void listDir(fs::FS &fs, const char *dirname, uint8_t levels);

void createDir(fs::FS &fs, const char *path);

void removeDir(fs::FS &fs, const char *path);

void readFile(fs::FS &fs, const char *path);

void writeFile(fs::FS &fs, const char *path, const char *message);

void appendFile(fs::FS &fs, const char *path, const char *message);

void renameFile(fs::FS &fs, const char *path1, const char *path2);

void deleteFile(fs::FS &fs, const char *path);

void Refresh_SD(DateTime *RTCClk, BME_Sensor *SenVal);
