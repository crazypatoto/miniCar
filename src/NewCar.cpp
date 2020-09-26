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
    int16_t W1000 = (int16_t)(fabs(W) + 0.5); //Round to interger
    W1000 = W < 0 ? -W1000 : W1000;

    txbuff[1] = (V >> 8) & 0xFF;
    txbuff[2] = V & 0xFF;
    txbuff[3] = (W1000 >> 8) & 0xFF;
    txbuff[4] = W1000 & 0xFF;

    for (uint8_t i = 0; i < 6; i++)
    {
        serialPutchar(fd, txbuff[i]);
    }
    usleep(1000);
}

uint8_t NewCar::getOdometry(int16_t &x, int16_t &y, float &angle)
{
    const char txbuff[3] = {'O', 'D', 'M'};
    char rxbuff[6] = {0};
    uint16_t timeout_count = 0;
    for (uint8_t i = 0; i < 3; i++)
    {
        serialPutchar(fd, txbuff[i]);
    }

    while (serialDataAvail(fd) == 0)
    {
        if (timeout_count++ > 3)
        {
            return 0;
        }
        usleep(1000);
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        rxbuff[i] = serialGetchar(fd);
    }

    x = y = 0;
    x = (rxbuff[0] << 8) | rxbuff[1];
    y = (rxbuff[2] << 8) | rxbuff[3];
    int16_t _angle = (rxbuff[4] << 8) | rxbuff[5];
    angle = _angle / 1000.0;
    angle = angle / M_PI * 180.0;
    if (angle > 180.0)
    {
        angle -= 360.0;
    }
    else if (angle < -180.0)
    {
        angle += 360.0;
    }

    return 1;
}

void NewCar::setOdometry(const int16_t x, const int16_t y, const float angle)
{
    char txbuff[8] = {'O', 0, 0, 0, 0, 0, 0, 'E'};    

    txbuff[1] = (x >> 8) & 0xFF;
    txbuff[2] = x & 0xFF;
    txbuff[3] = (y >> 8) & 0xFF;
    txbuff[4] = y & 0xFF;

    int16_t angle1000 = (int16_t)(angle * 1000.0);
    txbuff[5] = (angle1000 >> 8) & 0xFF;
    txbuff[6] = angle1000 & 0xFF;

    for (uint8_t i = 0; i < 8; i++)
    {
        serialPutchar(fd, txbuff[i]);
    }
    usleep(1000);
}

void NewCar::clearOdometry()
{
    const char txbuff[3] = {'C', 'L', 'R'};
    for (uint8_t i = 0; i < 3; i++)
    {
        serialPutchar(fd, txbuff[i]);
    }
    usleep(1000);
}