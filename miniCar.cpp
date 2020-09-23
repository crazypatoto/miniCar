#include <stdio.h>
#include <string.h>
#include <math.h>
#include <termios.h> // for tcxxxattr, ECHO, etc ..
#include <unistd.h>  // for STDIN_FILENO

//#include "src/Car.h"
#include "src/NewCar.h"
#include "src/QRCode.h"
#include "src/PIDController.h"
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
                    }},
};

//0xWWBBGGRR
uint32_t matrix_Right[64] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00002266, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00002266, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00002266, 0x00000000,
    0x00002266, 0x00002266, 0x00002266, 0x00002266, 0x00002266, 0x00002266, 0x00002266, 0x00002266,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00002266, 0x00000000,
    0x00000000, 0x00000000, 0x00002266, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00002266, 0x00000000, 0x00000000, 0x00000000};

uint32_t matrix_Left[64] = {
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00002266, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00002266, 0x00000000, 0x00000000,
    0x00000000, 0x00002266, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00002266, 0x00002266, 0x00002266, 0x00002266, 0x00002266, 0x00002266, 0x00002266, 0x00002266,
    0x00000000, 0x00002266, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00002266, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x00000000, 0x00002266, 0x00000000, 0x00000000, 0x00000000, 0x00000000};

uint32_t matrix_Stop[64] = {
    0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066,
    0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066,
    0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066,
    0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066,
    0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066,
    0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066,
    0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066,
    0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066, 0x00000066};

uint32_t matrix_Idle[64] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000};
uint32_t matrix_1[64] = {0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000};
uint32_t matrix_2[64] = {0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000};
uint32_t matrix_3[64] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000};

QRCode qrCode;
int16_t qrX, qrY, qrAngle;
uint32_t tagnum;

NewCar car;
int16_t V = 0;
float W = 0;
int16_t x, y;
float angle;

PIDContorller anglePID(0.035, 0.000, 0.0, 3, -3, 0.05);
PIDContorller linearPID(1, 0.00, 0.0, 930, -930, 5);

void manualControl(void);
char getch(void);

int main(void)
{
    ws2811_init(&ledstring);
    ledstring.channel[0].brightness = 32;
    memcpy(ledstring.channel[0].leds, matrix_Idle, 4 * 64);
    //memset(ledstring.channel[0].leds, 0, 4 * 64);
    ws2811_render(&ledstring);

    //manualControl();

start:
    
    car.clearOdometry();
    while (1)
    {
        car.getOdometry(x, y, angle);
        if ((qrCode.getInformation(qrX, qrY, qrAngle, tagnum)) && (abs(x) > 100))
        {
            x = 930 + qrX;
            if ((abs(qrX) <= 3) && (abs(V) < 25))
            {

                car.setCarParams(0, 0);
                break;
            }
        }
        V = linearPID.calculate(930 - x);
        printf("%d\n",V);
        car.setCarParams(V, 0);
        usleep(10000);
    }

    usleep(10000);
    while (1)
    {

        if (qrCode.getInformation(qrX, qrY, qrAngle, tagnum))
        {
            //printf("QRCode: X:%d, Y:%d, Angle:%d, Num:%d\n", qrX, qrY, qrAngle, tagnum);
            float err = 90 - qrAngle;
            if (err > 180.0)
                err -= 360.0;
            else if (err < -180.0)
                err += 360.0;
            else if (fabs(err) < 1.0)
            {
                car.setCarParams(0, 0);
                break;
            }

            W = anglePID.calculate(err);
        }
        else
        {
            // puts("NO TAG!");
        }
        car.setCarParams(0, W);
        car.getOdometry(x, y, angle);
        //printf("%lf,%lf\n", angle, W);
        usleep(10000);
    }

    usleep(10000);
    car.clearOdometry();
    while (1)
    {
        car.getOdometry(x, y, angle);
        if ((qrCode.getInformation(qrX, qrY, qrAngle, tagnum)) && (abs(x) > 100))
        {
            x = 930 - qrY;
            if ((abs(qrY) <= 3) && (abs(V) < 25))
            {

                car.setCarParams(0, 0);
                break;
            }
        }
        V = linearPID.calculate(930 - x);
        printf("%d\n",V);
        car.setCarParams(V, 0);
        usleep(10000);
    }

    usleep(10000);
    while (1)
    {

        if (qrCode.getInformation(qrX, qrY, qrAngle, tagnum))
        {
            //printf("QRCode: X:%d, Y:%d, Angle:%d, Num:%d\n", qrX, qrY, qrAngle, tagnum);
            float err = 180 - qrAngle;
            if (err > 180.0)
                err -= 360.0;
            else if (err < -180.0)
                err += 360.0;
            else if (fabs(err) < 1.0)
            {
                car.setCarParams(0, 0);
                break;
            }

            W = anglePID.calculate(err);
        }
        else
        {
            // puts("NO TAG!");
        }
        car.setCarParams(0, W);
        car.getOdometry(x, y, angle);
        //printf("%lf,%lf\n", angle, W);
        usleep(10000);
    }

    usleep(10000);
    car.clearOdometry();
    while (1)
    {
        car.getOdometry(x, y, angle);
        if ((qrCode.getInformation(qrX, qrY, qrAngle, tagnum)) && (abs(x) > 100))
        {
            x = 930 - qrX;
            if ((abs(qrX) <= 3) && (abs(V) < 25))
            {

                car.setCarParams(0, 0);
                break;
            }
        }
        V = linearPID.calculate(930 - x);
        printf("%d\n",V);
        car.setCarParams(V, 0);
        usleep(10000);
    }

    usleep(10000);
    while (1)
    {

        if (qrCode.getInformation(qrX, qrY, qrAngle, tagnum))
        {
            //printf("QRCode: X:%d, Y:%d, Angle:%d, Num:%d\n", qrX, qrY, qrAngle, tagnum);
            float err = -90 - qrAngle;
            if (err > 180.0)
                err -= 360.0;
            else if (err < -180.0)
                err += 360.0;
            else if (fabs(err) < 1.0)
            {
                car.setCarParams(0, 0);
                break;
            }

            W = anglePID.calculate(err);
        }
        else
        {
            // puts("NO TAG!");
        }
        car.setCarParams(0, W);
        car.getOdometry(x, y, angle);
        //printf("%lf,%lf\n", angle, W);
        usleep(10000);
    }

    usleep(10000);
    car.clearOdometry();
    while (1)
    {
        car.getOdometry(x, y, angle);
        if ((qrCode.getInformation(qrX, qrY, qrAngle, tagnum)) && (abs(x) > 100))
        {
            x = 930 + qrY;
            if ((abs(qrY) <= 3) && (abs(V) < 25))
            {

                car.setCarParams(0, 0);
                break;
            }
        }
        V = linearPID.calculate(930 - x);
        printf("%d\n",V);
        car.setCarParams(V, 0);
        usleep(10000);
    }

     usleep(10000);
    while (1)
    {

        if (qrCode.getInformation(qrX, qrY, qrAngle, tagnum))
        {
            //printf("QRCode: X:%d, Y:%d, Angle:%d, Num:%d\n", qrX, qrY, qrAngle, tagnum);
            float err = 0 - qrAngle;
            if (err > 180.0)
                err -= 360.0;
            else if (err < -180.0)
                err += 360.0;
            else if (fabs(err) < 1.0)
            {
                car.setCarParams(0, 0);
                break;
            }

            W = anglePID.calculate(err);
        }
        else
        {
            // puts("NO TAG!");
        }
        car.setCarParams(0, W);
        car.getOdometry(x, y, angle);
        //printf("%lf,%lf\n", angle, W);
        usleep(10000);
    }

    goto start;
    return 0;
}

void manualControl(void)
{
    char ch;
    car.setCarParams(0, 0);
    car.clearOdometry();
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
                //puts("Up");
                V += 10;
                break;
            case 'B':
                // code for arrow down
                //puts("Down");
                V -= 10;
                break;
            case 'C':
                // code for arrow right
                //puts("Right");
                W += 0.1f;
                break;
            case 'D':
                // code for arrow left
                //puts("Left");
                W -= 0.1f;
                break;
            }
            // printf("V:%d\tW:%lf\n", V, W);
            car.setCarParams(V, W);
        }
        else if (ch == 'o' || ch == 'O')
        {
            car.clearOdometry();
        }
        else if (ch == ' ')
        {
            memcpy(ledstring.channel[0].leds, matrix_Stop, 4 * 64);
            ws2811_render(&ledstring);
            while (abs(V) > 0 || fabs(W) > 0.0f)
            {
                if (V > 0)
                {
                    V -= 10;
                }
                else if (V < 0)
                {
                    V += 10;
                }
                if (W > 0.1f)
                {
                    W -= 0.1f;
                }
                else if (W < -0.01f)
                {
                    W += 0.1f;
                }
                else
                {
                    W = 0;
                }
                //printf("V:%d\tW:%lf\n", V, W);
                car.setCarParams(V, W);
                delay(10);
            }
            V = 0;
            W = 0.0f;
        }

        if (W > 0.0f)
        {
            memcpy(ledstring.channel[0].leds, matrix_Right, 4 * 64);
            ws2811_render(&ledstring);
        }
        else if (W < 0.0f)
        {
            memcpy(ledstring.channel[0].leds, matrix_Left, 4 * 64);
            ws2811_render(&ledstring);
        }
        else
        {
            memcpy(ledstring.channel[0].leds, matrix_Idle, 4 * 64);
            ws2811_render(&ledstring);
        }

        //car.getOdometry(x, y, angle);
        //printf("%d,%d\n", x, y);

        qrCode.getInformation(qrX, qrY, qrAngle, tagnum);
        printf("QRCode: X:%d, Y:%d, Angle:%d, Num:%d\n", qrX, qrY, qrAngle, tagnum);

        //printf("x:%d y:%d angle:%f\n", x, y, angle / M_PI * 180.0);
    }
}

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