#include <stdio.h>
#include <unistd.h>

//#include "src/Car.h"
#include "src/NewCar.h"
#include "src/QRCode.h"

int main(void)
{
    QRCode qrCode;
    NewCar car;
    int16_t x, y;
    float angle;

    car.setCarParams(100, 0.314159);    
    delay(1000);
    car.setCarParams(0, 0);      

    while(1){
        car.getOdometry(x,y,angle);
        printf("x:%d y:%d angle:%f\n",x,y,angle);
        delay(10);
    }
    // car.setCarParams(10, 0.1);
    // delay(5000);
    // car.setCarParams(-10, -0.1);
    // delay(5000);
    // car.setCarParams(0, 0);

    // for (int i = 0; i <= 1000; i += 1)
    // {
    //     car.setCarParams(i, 0);
    //     delay(10);
    // }    
    // for (int i = 1000; i >= 0; i -= 1)
    // {
    //     car.setCarParams(i, 0);
    //     delay(10);
    // }

    // int16_t x, y, angle;
    // uint32_t num;
    // if (qrCode.getInformation(x, y, angle, num))
    // {
    //     printf("X: %d\tY: %d\tAngle: %d\tTag: %d\n", x, y, angle, num);
    // }
    // else
    // {
    //     printf("Tag Not Found!\n");
    // }

    return 0;
}
