#ifndef QRCODE_H
#define QRCODE_H

#include <stdint.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdio.h>

#define EN_485 (4)

class QRCode
{
private:
    int fd;    
    const char txData[2] = {0xC8,0x37};
public:
    QRCode();
    ~QRCode();
    uint8_t getInformation(int16_t& x, int16_t& y, int16_t& angle, uint32_t& tagnum);
};


#endif