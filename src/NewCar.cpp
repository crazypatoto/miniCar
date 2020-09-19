#include "NewCar.h"

NewCar::NewCar(/* args */)
{
    fd = serialOpen("/dev/ttyAMA1", 115200);
    if (fd < 0)
    {
        throw "Serial Error!";
    }
    printf("NewCar Initialized!\n");
    serialFlush(fd);
}

NewCar::~NewCar()
{
}

void NewCar::setCarParams(int16_t V, float W)
{
    char txbuff[6] = {'S', 0, 0, 0, 0, 'E'};
    W *= 1000;
    int16_t W1000 = (int16_t)(fabs(W) + 0.5);  //Round to interger
    W1000 = W < 0 ? -W1000 : W1000;

    txbuff[1] = (V >> 8) & 0xFF;
    txbuff[2] = V & 0xFF;
    txbuff[3] = (W1000 >> 8) & 0xFF;
    txbuff[4] = W1000 & 0xFF;

    for (uint8_t i = 0; i < 6; i++)
    {
        serialPutchar(fd, txbuff[i]);
    }
    delay(1);
}

void NewCar::getOdometry(int16_t &x, int16_t &y, float &angle)
{
    const char txbuff[3] = {'O', 'D', 'M'};
    char rxbuff[6] = {0};
    for (uint8_t i = 0; i < 3; i++)
    {
        serialPutchar(fd, txbuff[i]);
    }

    while (serialDataAvail(fd) == 0)
        ;

    for (uint8_t i = 0; i < 6; i++)
    {
        rxbuff[i] = serialGetchar(fd);
    }

    x = y = 0;
    x = (rxbuff[0] << 8) | rxbuff[1];
    y = (rxbuff[2] << 8) | rxbuff[3];
    int16_t _angle = (rxbuff[4] << 8) | rxbuff[5];
    angle = _angle / 10000.0;
}