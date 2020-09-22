#include <stdio.h>
#include <string.h>
#include <math.h>
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
uint32_t matrix_1[64] = {0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000 };
uint32_t matrix_2[64] = {0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000 };
uint32_t matrix_3[64] = {0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x0059339F, 0x0059339F, 0x0059339F, 0x00000000, 0x00000000 };

struct _pid
{
    float Target = 0;   //定義設定值
    float Actual = 0;   //定義實際值
    float err = 0;      //定義偏差值
    float err_last = 0; //定義上一個偏差值
    float Kp, Ki, Kd;   //定義比例、積分、微分系數
    float Output = 0;   //定義電壓值（控制執行器的變數）
    float integral = 0; //定義積分值
} pid;

float PID_Calculate(float target, float actual)
{
    pid.err = target - actual;
    pid.integral = pid.err;
    pid.Output = pid.Kp * pid.err + pid.Ki * pid.integral + pid.Kd * (pid.err - pid.err_last);
    pid.err_last = pid.err;

    return pid.Output;
}

QRCode qrCode;
NewCar car;
int16_t V = 0;
float W = 0;
int16_t x, y;
float angle;

int16_t tar_x = 1000;
int16_t tar_y = 0;
int16_t lastV = 0;

void manualControl(void);
char getch(void);

int main(void)
{
    ws2811_init(&ledstring);
    ledstring.channel[0].brightness = 32;
    memcpy(ledstring.channel[0].leds, matrix_Idle, 4 * 64);
    ws2811_render(&ledstring);

    manualControl();

    pid.Kp = 0.66f;
    pid.Ki = 0.55;
    pid.Kd = 0;

    car.clearOdometry();
    while (1)
    {
        car.getOdometry(x, y, angle);
        //int16_t err_distance = pow((pow((tar_x - x), 2) + pow((tar_y - y), 2)), 0.5);
        int16_t err_distance = tar_x - x;
        //printf("x:%d y:%d angle:%f\n", x, y, angle / M_PI * 180.0);
        V = -PID_Calculate(0, err_distance);
        if (V > 1000)
        {
            V = 1000;
        }
        else if (V < -1000)
        {
            V = -1000;
        }

        if (V - lastV > 5)
        {
            V = lastV + 5;
        }
        else if (V - lastV < -5)
        {
            V = lastV - 5;
        }
        printf("%d,%d,%d\n", x, y, V);
        car.setCarParams(V, 0);
        lastV = V;
        //printf("V:%d\n", V);
        delay(10);
    }

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

        car.getOdometry(x, y, angle);
        printf("%d,%d\n", x, y);
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