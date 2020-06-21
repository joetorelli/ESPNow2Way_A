#include "NetWork.h"

void GetMacAdr()
{
    WiFi.mode(WIFI_MODE_STA);
    Serial.println(WiFi.macAddress());
}