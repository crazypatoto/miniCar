#include "miniCar.h"

int main(void)
{
    Car miniCar;
    QRCode qrCode;

    miniCar.setAcceleration(1000);
    delay(5);
    miniCar.turn(90);
    delay(1000);
    miniCar.move(5000);

    int16_t x,y,angle;
    uint32_t num;

    if(qrCode.getInformation(x,y,angle,num)){
        printf("X: %d\tY: %d\tAngle: %d\tTag: %ul\n",x,y,angle,num);
    }else{
        printf("Tag Not Found!\n");
    }
  
   
    return 0;
}
