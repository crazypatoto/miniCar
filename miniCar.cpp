#include <cstdio>
#include <cstring>
#include <cmath>
#include <termios.h> // for tcxxxattr, ECHO, etc ..
#include <unistd.h>  // for STDIN_FILENO

//#include "src/Car.h"
#include "src/NewCar.h"
#include "src/QRCode.h"
#include "src/PIDController.h"
#include "lib/rpi_ws281x/ws2811.h"

#define degToRad(angleInDegrees) ((angleInDegrees)*M_PI / 180.0f)
#define radToDeg(angleInRadians) ((angleInRadians)*180.0f / M_PI)

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

//QRCode Variables
QRCode qrCode;
int16_t qrX, qrY, qrAngle;
uint32_t qrTagNum;

//Car Variables
NewCar car;
int16_t setV = 0;
float setW = 0;
int16_t odmX = 0, odmY = 0;
int16_t odmX_offset = 0, odmY_offset = 0;
float odmAngle = 0;
float odmAngle_offset = 0;

//PID Controllers
//PIDContorller angularPID(0.035, 0.000, 0.0, 3, -3, 0.05);
PIDContorller angularPID(0.035, 0.000, 0.0, 3, -3, 0.05);
PIDContorller angularPID2(0.055, 0.000, 0.0, 3, -3, 0.05);
PIDContorller linearPID(0.7, 0.00, 0.01, 1000, -1000, 5);

void manualControl(void);
char getch(void);

void setAngle(int16_t targetX, int16_t targetY)
{
    car.getOdometry(odmX, odmY, odmAngle);
    float targetAngle = (atan2(targetY, targetX)) / M_PI * 180.0;
    printf("Target = %f\n", targetAngle);

    while (1)
    {
        if (qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum))
        {
            //printf("Now = %d\n", qrAngle);
            //printf("Err = %f\n", targetAngle - qrAngle);
            float err = targetAngle - qrAngle;
            if (err > 180.0)
            {
                err -= 360.0;
            }
            else if (err < -180.0)
            {
                err += 360.0;
            }

            setW = -angularPID.calculate(err);
            if (abs(err) <= 0.02 && (abs(setW) < 0.02))
            {
                car.setCarParams(setV, 0);
                break;
            }
            car.setCarParams(setV, setW);
            usleep(10000);
        }
    }
}

int main(int argc, char *argv[])
{
    system("clear");
    ws2811_init(&ledstring);
    ledstring.channel[0].brightness = 32;
    //memcpy(ledstring.channel[0].leds, matrix_Idle, 4 * 64);
    memset(ledstring.channel[0].leds, 0, 4 * 64);
    ws2811_render(&ledstring);

    // while (1)
    // {
    //     if(qrCode.getInformation(qrX,qrY,qrAngle,qrTagNum)){
    //         printf("qrX: %d, qrY: %d, qrAngle: %d, qrTagNum: %d\n",qrX,qrY,qrAngle,qrTagNum);
    //         car.setOdometry(qrX,qrY,degToRad(qrAngle));
    //     }
    //     car.getOdometry(odmX,odmY,odmAngle);
    //     printf("x:%d y:%d angle:%f\n",odmX,odmY,odmAngle);
    //     usleep(1000000);
    // }
    //  car.clearOdometry();
    // setAngle(1, 0);
    // setAngle(-1, 0);
    // setAngle(0, 1);
    // setAngle(0, -1);
    // setAngle(1, 1);
    // setAngle(-1, -1);
    // setAngle(-1, 1);
    // setAngle(1, -1);
    // setAngle(1, 0);
    // return 0;
    setAngle(100, 0);
    car.clearOdometry();

    int16_t targetX = 900 * 7, targetY = 0;
    int16_t currentTargetX = 900 * 7;
    setAngle(targetX, targetY);
    usleep(500000);
    qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum);
    car.setOdometry(qrX, qrY, degToRad(qrAngle));
    while (1)
    {

        if (qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum))
        {
            // switch (qrTagNum)
            // {
            // case 2:
            //     car.setOdometry(-890 + qrX, qrY, degToRad(qrAngle));
            //     break;
            // case 3:
            //     car.setOdometry(qrX, qrY, degToRad(qrAngle));
            //     break;
            // case 5:
            //     car.setOdometry(-890 * 2 + qrX, qrY, degToRad(qrAngle));
            //     break;
            // // case 6:
            // //     car.setOdometry(-890 * 3 + qrX, qrY, degToRad(qrAngle));
            // default:
            //     break;
            // }
            car.setOdometry(890*(qrTagNum-1) + qrX, qrY, degToRad(qrAngle));
            // odmX -= odmX_offset;
            // odmY -= odmY_offset;
            // odmAngle -= odmAngle_offset;
            // odmAngle = atan2(sinf(odmAngle), cosf(odmAngle));

            // else if (qrTagNum == 5)
            // {
            //     start_flag = 0;
            //     setW = 0;
            // }
            // else if (qrTagNum == 3)
            // {
            //     odmAngle_offset = odmAngle - qrAngle;
            //     odmX_offset = odmX - (0 + qrX);
            //     odmY_offset = odmY - qrY;
            // }
        }
        car.getOdometry(odmX, odmY, odmAngle);

        printf("XERR: %d, YERR: %d\n", targetX - odmX, targetY - odmY);
        float dErr = powf(powf(targetX - odmX, 2) + powf(targetY - odmY, 2), 0.5);
        float dotErr = cosf(degToRad(odmAngle)) * (targetX - odmX) + sinf(degToRad(odmAngle)) * (targetY - odmY);
        dErr = dotErr > 0 ? dErr : -dErr;
        printf("dErr = %f\n", dErr);
        if (abs(dErr) < 3 && abs(setV) < 10)
        {
            car.setCarParams(0, 0);
            break;
        }
        //printf("dRrr = %f\n", dErr);
        setV = linearPID.calculate(dErr);
        
        if(targetX - odmX > 900){
            currentTargetX = odmX + 900;
        }else{
            currentTargetX = targetX;
        }

        float targetAngle = (atan2(targetY - odmY, currentTargetX - odmX)) / M_PI * 180.0;
        float err;

        err = targetAngle - odmAngle;

        if (err > 180.0)
        {
            err -= 360.0;
        }
        else if (err < -180.0)
        {
            err += 360.0;
        }
        // if (abs(err) <= 1 && setW < 0.01)
        // {
        //     setW = 0;
        //     start_flag = 0;
        // }
        setW = -angularPID2.calculate(err);

        printf("V: %d W%f\n", setV, setW);
        car.setCarParams(setV, setW);
        printf("odmX = %d, odmY = %d, odmAngle = %f\n", odmX, odmY, odmAngle);
        usleep(10000);
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
            memcpy(ledstring.channel[0].leds, matrix_Stop, 4 * 64);
            ws2811_render(&ledstring);
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

        if (setW > 0.0f)
        {
            memcpy(ledstring.channel[0].leds, matrix_Right, 4 * 64);
            ws2811_render(&ledstring);
        }
        else if (setW < 0.0f)
        {
            memcpy(ledstring.channel[0].leds, matrix_Left, 4 * 64);
            ws2811_render(&ledstring);
        }
        else
        {
            memcpy(ledstring.channel[0].leds, matrix_Idle, 4 * 64);
            ws2811_render(&ledstring);
        }

        car.getOdometry(odmX, odmY, odmAngle);
        printf("X:%d Y:%d Angle:%f\n", odmX, odmY, odmAngle);

        //qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum);
        //printf("QRCode: X:%d, Y:%d, Angle:%d, Num:%d\n", qrX, qrY, qrAngle, qrTagNum);

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