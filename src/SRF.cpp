#include <Arduino.h>
#include "SRF.h"

void SRFPing()
{
    //adr = 0xE0 i2c addressing is high 7 bit (right shift 1) so send 0x70
    Wire.beginTransmission(SRF_ADDR); //transmit to device
    Wire.write(byte(SRF_COMMAND));    //command register
    Wire.write(byte(SRF_CMD_IN));     //0x50=inches, 0x51=cent; 0x52=microsec
    Wire.endTransmission();

    //wait for reading
    delay(70); //wait for return, max 65ms
}

int SRFLight()
{
    int LightValue;

    Wire.beginTransmission(SRF_ADDR); //transmit to device
    Wire.write(byte(SRF_LIGHT));      //light register adr
    Wire.endTransmission();

    Wire.requestFrom(SRF_ADDR, 1); // read one register  byte
    while (Wire.available() < 1)
    {
        // need some sort of timeout!
    }
    LightValue = Wire.read();
    return LightValue;
    //OLED_Display.println(Light);
}

void SRFDistance(SRFRanges *SRFD)
{
    //goto firsst echo register
    Wire.beginTransmission(SRF_ADDR);
    Wire.write(byte(SRF_ECHO_H)); //1st range high byte adr
    Wire.endTransmission();
    //get values
    Wire.requestFrom(SRF_ADDR, 2); // read two registers hi and low byte

    while (Wire.available() < 2)
    {
        // need some sort of timeout!
    }

    SRFD->Range = Wire.read();
    SRFD->Range = SRFD->Range << 8;
    SRFD->Range |= Wire.read();
    //OLED_Display.println(Range);

    Wire.beginTransmission(SRF_ADDR);
    Wire.write(byte(SRF_ECHO2_H)); //2nd range high byte
    Wire.endTransmission();
    //get values
    Wire.requestFrom(SRF_ADDR, 2); // read two registers hi and low buyte
    while (Wire.available() < 2)
    {
        // need some sort of timeout!
    }

    SRFD->Range2 = Wire.read();
    SRFD->Range2 = SRFD->Range2 << 8;
    SRFD->Range2 |= Wire.read();
    //OLED_Display.println(Range2);

    ////3rd reading
    //
    Wire.beginTransmission(SRF_ADDR);
    Wire.write(byte(SRF_ECHO3_H)); //3rd range high byte
    Wire.endTransmission();

    //get values
    Wire.requestFrom(SRF_ADDR, 2); // read two registers hi and low buyte
    while (Wire.available() < 2)
    {
        // need some sort of timeout!
    }

    SRFD->Range3 = Wire.read();
    SRFD->Range3 = SRFD->Range3 << 8;
    SRFD->Range3 |= Wire.read();
    //OLED_Display.println(Range3);
}

/** Change address of SRF 08
int srf08ChangeAddr(unsigned char oldAddr, unsigned char newAddr){
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xA0);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xAA);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    i2cWrite(0xA5);
    i2cStop();   
    i2cStart(SRF_ADDR + I2C_WRITE + (2 * oldAddr));
    i2cWrite(SRF_COMMAND);
    // actually send address
    i2cWrite(SRF_ADDR + (2 * newAddr));
    i2cStop(); 
} NOTE THIS CODE WORKS, BUT IS NOT STANDARD TO INCLUDE. */