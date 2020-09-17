#ifndef NEWCAR_H
#define NEWCAR_H

#include <stdint.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdio.h>
#include <math.h>

class NewCar
{
private:
    int fd; 
public:
    NewCar(/* args */);
    ~NewCar();
    void setCarParams(int16_t V,float W);
    void getOdometry(int16_t &x, int16_t &y, float &angle);
};



#endif