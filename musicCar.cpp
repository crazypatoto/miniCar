#include <iostream>
#include <cstdio>
#include <string>
#include <cmath>
#include <vector>
#include <thread>
#include <termios.h>    // for tcxxxattr, ECHO, etc ..
#include <unistd.h>     // for STDIN_FILENO
#include <sys/socket.h> //for TCP socket
#include <netinet/in.h>
#include <netdb.h> //hostent
#include <arpa/inet.h>

#include "include/NewCar.h"
#include "include/QRCode.h"
#include "include/PIDController.h"
#include "include/LedMatrix.h"

#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0f)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0f / M_PI)

//LED Variables
LedMatrix led;

//QRCode Variables
QRCode qrCode;
int16_t qrX, qrY, qrAngle;
uint32_t qrTagNum;
QRCode::qrcode_node_t *headQR;

//Car Variables
NewCar car;
int16_t setV = 0;
float setW = 0;
int16_t odmX = 0, odmY = 0;
float odmAngle = 0;

uint32_t matrix[64] = {0x00000000, 0x00000000, 0x008B008B, 0x008B008B, 0x008B008B, 0x008B008B, 0x008B008B, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x008B008B, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x008B008B, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x008B008B, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x008B008B, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x008B008B, 0x00000000, 0x008B008B, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x008B008B, 0x008B008B, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x008B008B, 0x00000000, 0x00000000, 0x00000000, 0x00000000};

void manualControl(void);
char getch(void);

int main(int argc, char *argv[])
{
    system("clear");
    car.setCarParams(0, 0);
    led.clear();
    manualControl();
    car.setCarParams(0, 0);
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
                setV += 10;
                break;
            case 'B':
                // code for arrow down
                //puts("Down");
                setV -= 10;
                break;
            case 'C':
                // code for arrow right
                //puts("Right");
                setW += 0.1f;
                break;
            case 'D':
                // code for arrow left
                //puts("Left");
                setW -= 0.1f;
                break;
            }
            // printf("setV:%d\tW:%lf\n", setV, setW);
            car.setCarParams(setV, setW);
        }
        else if (ch == 'o' || ch == 'O')
        {
            car.clearOdometry();
        }
        else if (ch == ' ')
        {
            while (abs(setV) > 0 || fabs(setW) > 0.0f)
            {
                if (setV > 0)
                {
                    setV -= 10;
                }
                else if (setV < 0)
                {
                    setV += 10;
                }
                if (setW > 0.1f)
                {
                    setW -= 0.1f;
                }
                else if (setW < -0.01f)
                {
                    setW += 0.1f;
                }
                else
                {
                    setW = 0;
                }
                //printf("setV:%d\tW:%lf\n", setV, setW);
                car.setCarParams(setV, setW);
                delay(10);
            }
            setV = 0;
            setW = 0.0f;
        }
        else if (ch == 'c' || ch == 'C')
        {
            led.setPattern(led.matrix_Idle);
            led.render();
            setW = 0.0f;
            car.setCarParams(setV, setW);
        }
        else if (ch == 'm' || ch == 'M')
        {
            car.sendStartSignal();
            led.setPattern(matrix);
            led.render();
        }
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
