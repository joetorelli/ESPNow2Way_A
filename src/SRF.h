#ifndef SRF_H

#define SRF_H
#include <Arduino.h>
#include "settings.h"
#include "Wire.h"




// Base address = 0xE0. 0xE0-0xFF valid. addr = base + (2 * device_no)
#define SRF_ADDR 0x70
#define SRF_COMMAND 0x00 // command register

#define SRF_LIGHT 0x01   // light sensor register
#define SRF_ECHO_H 0x02  // 1st echo high byte
#define SRF_ECHO_L 0x03  // 1st echo low byte
#define SRF_ECHO2_H 0x04 // 2nd echo high byte
#define SRF_ECHO2_L 0x05 // 2nd echo low byte
#define SRF_ECHO3_H 0x06 // 3rd echo high byte
#define SRF_ECHO3_L 0x07 // 3rd echo low byte
#define SRF_ECHO4_H 0x08 // 4th echo high byte
#define SRF_ECHO4_L 0x09 // 4th echo low byte

#define SRF_CMD_IN 0x50 // Ranging mode - result in inches
#define SRF_CMD_CM 0x51 // Ranging mode - result in centimeters
#define SRF_CMD_US 0x52 // Ranging mode - result in micro sec

struct SRFRanges
{
    int Range = 0;
    int Range2 = 0;
    int Range3 = 0;
};

//byte SRFStat = 0xFF;

void SRFPing();
int SRFLight();
void SRFDistance(SRFRanges *SRFD);
#endif