#ifndef QRCODE_H
#define QRCODE_H

#include <cstdint>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <cstdio>
#include <unistd.h>  // for STDIN_FILENO

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