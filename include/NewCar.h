#ifndef NEWCAR_H
#define NEWCAR_H

#include <cstdint>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <cstdio>
#include <cmath>
#include <unistd.h> // for STDIN_FILENO

class NewCar
{
private:
    int fd;

public:
    NewCar(/* args */);
    ~NewCar();
    void setCarParams(int16_t V, float W);
    uint8_t getOdometry(int16_t &x, int16_t &y, float &angle);
    void setOdometry(const int16_t x, const int16_t y, const float angle);
    void clearOdometry();
    void sendStartSignal();
};

#endif