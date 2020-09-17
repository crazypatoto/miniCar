#ifndef CAR_H
#define CAR_H

#include <stdint.h>
#include <wiringSerial.h>
#include <wiringPi.h>
#include <stdio.h>

class Car
{
    typedef enum _RW
    {
        Read = 'R',
        Write = 'W'
    } RW;

private:
    int fd;    
    uint16_t sendCommand(RW rw, char address, uint16_t data);
public:
    Car();
    ~Car();
    void setMaxSpeed(uint16_t speed);
    void setAcceleration(uint16_t acceleration);
    void move(int16_t distance);
    void turn(int16_t angle);
    uint8_t isCarRunning();
};

#endif