#include <iostream>
#include <cstdio>
#include <string>

#include "include/NewCar.h"

//Car Variables
NewCar car;

int main(int argc, char *argv[])
{
    car.setCarParams(0, 0);
    usleep(1000000);
    car.sendStartSignal();
}
