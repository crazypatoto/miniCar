#include <stdio.h>
#include <string.h>
#include <termios.h> // for tcxxxattr, ECHO, etc ..
#include <unistd.h>  // for STDIN_FILENO

//#include "src/Car.h"
#include "src/NewCar.h"
#include "src/QRCode.h"
#include "lib/rpi_ws281x/ws2811.h"

ws2811_t ledstring =

    {

        .freq = WS2811_TARGET_FREQ,

        .dmanum = 10,

        .channel =

            {

                [0] =

                    {

                        .gpionum = 18,

                        .invert = 0,

                        .count = 64,

                        .strip_type = WS2811_STRIP_GBR,

                        .brightness = 64,

                    },

                // [1] =

                //     {

                //         .gpionum = 0,

                //         .invert = 0,

                //         .count = 0,

                //         .brightness = 0,

                //     },

            },

};

char getch(void)
{
    int ch;
    struct termios oldt, newt;

    tcgetattr(STDIN_FILENO, &oldt);
    memcpy(&newt, &oldt, sizeof(newt));
    newt.c_lflag &= ~(ECHO | ICANON | ECHOE | ECHOK |
                      ECHONL | ECHOPRT | ECHOKE | ICRNL);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
}

//0xWWBBGGRR
uint32_t matrix_Right[64] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00001111, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00001111, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00001111, 0x00000000,
    0x00001111, 0x00001111, 0x00001111, 0x00001111, 0x00001111, 0x00001111, 0x00001111, 0x00001111,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00001111, 0x00000000,
    0x00000000, 0x00000000, 0x00001111, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00001111, 0x00000000, 0x00000000, 0x00000000
};

uint32_t matrix_Left[64] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00001111, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00001111, 0x00000000, 0x00000000,
    0x00000000, 0x00001111, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00001111, 0x00001111, 0x00001111, 0x00001111, 0x00001111, 0x00001111, 0x00001111, 0x00001111,
    0x00000000, 0x00001111, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00001111, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00001111, 0x00000000, 0x00000000, 0x00000000, 0x00000000
};

int
main(void)
{
    QRCode qrCode;
    NewCar car;
    int16_t x, y;
    float angle;
    int16_t V = 0;
    float W = 0;
    char ch;

    ws2811_init(&ledstring);
    // for (int i = 0; i < 64; i++)
    // {
    //     ledstring.channel[0].leds[i] = matrix_Left[i]; //0xWWBBGGRR
    // }
    memcpy(ledstring.channel[0].leds,matrix_Left,4*64);
    ledstring.channel[0].brightness = 32;
    ws2811_render(&ledstring);
    //car.setCarParams(0, -0.314);
    //delay(10000);
    //car.setCarParams(0, 0);

    //system ("/bin/stty raw");
    while (1)
    {
        //    car.getOdometry(x,y,angle);
        //    printf("x:%d y:%d angle:%f\n",x,y,angle);
        //    delay(500);
        ch = getch();
        if (ch == '\033')
        {            // if the first value is esc
            getch(); // skip the [
            switch (getch())
            { // the real value
            case 'A':
                // code for arrow up
                puts("Up");
                V += 10;
                break;
            case 'B':
                // code for arrow down
                puts("Down");
                V -= 10;
                break;
            case 'C':
                // code for arrow right
                puts("Right");
                W += 0.1;
                break;
            case 'D':
                // code for arrow left
                puts("Left");
                W -= 0.1;
                break;
            }
            printf("V:%d\tW:%f\n", V, W);
            car.setCarParams(V, W);
        }
        else
        {
            for (int i = V; i >= 0; i--)
            {
                car.setCarParams(i, 0);
                delay(1);
            }
            V = 0;
            W = 0;
        }
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
