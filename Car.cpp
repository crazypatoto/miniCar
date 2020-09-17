#include "Car.h"

Car::Car(/* args */)
{
    fd = serialOpen("/dev/ttyAMA1", 115200);
    if (fd < 0)
    {
        throw "Serial Error!";
    }
    printf("Car Initialized!\n");
    serialFlush(fd);
}

uint16_t Car::sendCommand(RW rw, char address, uint16_t data)
{
    char txbuff[6] = {0xAB, rw, address, (char)(data >> 8), (char)data, 0xCD};
    char rxbuff[6] = {0};
    for (int i = 0; i < 6; i++)
    {
        serialPutchar(fd, txbuff[i]);
    }

    while (serialDataAvail(fd) == 0)
        ;
    for (int i = 0; i < 6; i++)
    {
        rxbuff[i] = serialGetchar(fd);
        //printf("%02X\t", rxbuff[i]);
    }

    return (uint16_t)((rxbuff[3] << 8) | rxbuff[4]);
}

void Car::setMaxSpeed(uint16_t speed)
{
    sendCommand(Write, 0x00, speed);
}
void Car::setAcceleration(uint16_t acceleration)
{
    sendCommand(Write, 0x01, acceleration);
}
void Car::move(int16_t distance)
{
    sendCommand(Write, 0x02, (uint16_t)distance);
}
void Car::turn(int16_t angle)
{
    sendCommand(Write, 0x04, (uint16_t)angle);
}

uint8_t Car::isCarRunning()
{
    return sendCommand(Read, 0x06, 0);
}

Car::~Car()
{
    serialClose(fd);
}