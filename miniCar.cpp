#include <cstdio>
#include <cstring>
#include <cmath>
#include <termios.h> // for tcxxxattr, ECHO, etc ..
#include <unistd.h>  // for STDIN_FILENO

//#include "src/Car.h"
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
QRCode::qrcode_node_t *tailQR;
QRCode::qrcode_node_t *currentQR;

//Car Variables
NewCar car;
int16_t setV = 0;
float setW = 0;
int16_t odmX = 0, odmY = 0;
int16_t odmX_offset = 0, odmY_offset = 0;
float odmAngle = 0;
float odmAngle_offset = 0;

//PID Controllers
PIDContorller angularPID(0.035, 0.000, 0.0, 3, -3, 0.05);
PIDContorller angularPID2(0.1, 0.001, 0.0, 6, -6, 0.1);
PIDContorller linearPID(0.58, 0.0, 0.01, 1000, -1000, 5);

void manualControl(void);
char getch(void);

void setAngle(int16_t targetX, int16_t targetY)
{
    car.getOdometry(odmX, odmY, odmAngle);
    float targetAngle = (atan2(targetY - odmY, targetX - odmX)) / M_PI * 180.0;
    //printf("%d %d\n", targetY - odmY, targetX - odmX);
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

void genQR()
{
    QRCode::qrcode_node_t *newQR = (QRCode::qrcode_node_t *)malloc(sizeof(QRCode::qrcode_node_t));
    headQR = currentQR = newQR;

    // currentQR->xPos = 0;
    // currentQR->yPos = 0;
    // currentQR->tagNum = 1;
    // for (int i = 2; i <= 8; i++)
    // {
    //     newQR = (QRCode::qrcode_node_t *)malloc(sizeof(QRCode::qrcode_node_t));
    //     currentQR->next = newQR;
    //     currentQR = currentQR->next;
    //     currentQR->xPos = 980 * (i - 1);
    //     currentQR->yPos = 0;
    //     currentQR->tagNum = i;
    // }

    currentQR->xPos = 980 * 7;
    currentQR->yPos = 0;
    currentQR->tagNum = 8;
    for (int i = 7; i >= 1; i--)
    {
        newQR = (QRCode::qrcode_node_t *)malloc(sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 980 * (i - 1);
        currentQR->yPos = 0;
        currentQR->tagNum = i;
    }

    currentQR = headQR;
    while (currentQR != NULL)
    {
        printf("X:%d Y:%d tagNum:%d\n", currentQR->xPos, currentQR->yPos, currentQR->tagNum);
        currentQR = currentQR->next;
    }
}

int main(int argc, char *argv[])
{
    system("clear");
    genQR();
    led.clear();

    tailQR = currentQR = headQR;
    while (tailQR->next != NULL)
    {
        tailQR = tailQR->next;
    }

    int16_t targetX = tailQR->xPos, targetY = tailQR->yPos;
    int16_t currentTargetX = 0;

    while (1)
    {
        if (qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum))
        {
            if (qrTagNum == headQR->tagNum)
            {
                car.setOdometry(currentQR->xPos + qrX, currentQR->yPos + qrY, degToRad(qrAngle));
                break;
            }
        }
        usleep(1000);
    }
    setAngle(targetX, targetY);
    usleep(500000);

    while (1)
    {

        if (qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum))
        {
            if (currentQR->next != NULL)
            {
                if (qrTagNum == currentQR->next->tagNum)
                {
                    currentQR = currentQR->next;
                    //printf("QR%d passed!\n", currentQR->tagNum);
                }
            }

            car.setOdometry(currentQR->xPos + qrX, currentQR->yPos + qrY, degToRad(qrAngle));
        }
        car.getOdometry(odmX, odmY, odmAngle);

        //printf("XERR: %d, YERR: %d\n", targetX - odmX, targetY - odmY);
        float dErr = powf(powf(targetX - odmX, 2) + powf(targetY - odmY, 2), 0.5);
        float dotErr = cosf(degToRad(odmAngle)) * (targetX - odmX) + sinf(degToRad(odmAngle)) * (targetY - odmY);
        dErr = dotErr > 0 ? dErr : -dErr;
        //printf("dErr = %f\n", dErr);
        if (abs(dotErr) < 10 && abs(setV) < 10)
        {
            car.setCarParams(0, 0);
            break;
        }
        //printf("dRrr = %f\n", dErr);
        setV = linearPID.calculate(dErr);

        // if (targetX - odmX > 980)
        // {
        //     currentTargetX = odmX + 980;
        // }
        // else
        // {
        //     currentTargetX = targetX;
        // }
        currentTargetX = odmX - 490;

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

        //printf("QRCode: X:%d, Y:%d, Angle:%d, Num:%d\n", qrX, qrY, qrAngle, qrTagNum);
        // if (qrTagNum == tailQR->tagNum)
        // {
        //     setW = 0;
        // }
        // else
        // {
        //     setW = -angularPID2.calculate(err);
        // }
        setW = -angularPID2.calculate(err);

        // printf("V: %d W%f\n", setV, setW);
        car.setCarParams(setV, setW);
        printf("odmX = %d, odmY = %d, odmAngle = %f\n", odmX, odmY, odmAngle);
        //printf("%d,%d\n", odmX, odmY);
        usleep(10000);
    }

    return 0;
}

// void manualControl(void)
// {
//     char ch;
//     car.setCarParams(0, 0);
//     car.clearOdometry();
//     while (1)
//     {
//         //    car.getOdometry(x,y,angle);
//         //    printf("x:%d y:%d angle:%f\n",x,y,angle);
//         //    delay(500);
//         ch = getch();
//         if (ch == '\033')
//         {            // if the first value is esc
//             getch(); // skip the [
//             switch (getch())
//             { // the real value
//             case 'A':
//                 // code for arrow up
//                 //puts("Up");
//                 setV += 10;
//                 break;
//             case 'B':
//                 // code for arrow down
//                 //puts("Down");
//                 setV -= 10;
//                 break;
//             case 'C':
//                 // code for arrow right
//                 //puts("Right");
//                 setW += 0.1f;
//                 break;
//             case 'D':
//                 // code for arrow left
//                 //puts("Left");
//                 setW -= 0.1f;
//                 break;
//             }
//             // printf("setV:%d\tW:%lf\n", setV, setW);
//             car.setCarParams(setV, setW);
//         }
//         else if (ch == 'o' || ch == 'O')
//         {
//             car.clearOdometry();
//         }
//         else if (ch == ' ')
//         {
//             memcpy(ledstring.channel[0].leds, matrix_Stop, 4 * 64);
//             ws2811_render(&ledstring);
//             while (abs(setV) > 0 || fabs(setW) > 0.0f)
//             {
//                 if (setV > 0)
//                 {
//                     setV -= 10;
//                 }
//                 else if (setV < 0)
//                 {
//                     setV += 10;
//                 }
//                 if (setW > 0.1f)
//                 {
//                     setW -= 0.1f;
//                 }
//                 else if (setW < -0.01f)
//                 {
//                     setW += 0.1f;
//                 }
//                 else
//                 {
//                     setW = 0;
//                 }
//                 //printf("setV:%d\tW:%lf\n", setV, setW);
//                 car.setCarParams(setV, setW);
//                 delay(10);
//             }
//             setV = 0;
//             setW = 0.0f;
//         }

//         if (setW > 0.0f)
//         {
//             memcpy(ledstring.channel[0].leds, matrix_Right, 4 * 64);
//             ws2811_render(&ledstring);
//         }
//         else if (setW < 0.0f)
//         {
//             memcpy(ledstring.channel[0].leds, matrix_Left, 4 * 64);
//             ws2811_render(&ledstring);
//         }
//         else
//         {
//             memcpy(ledstring.channel[0].leds, matrix_Idle, 4 * 64);
//             ws2811_render(&ledstring);
//         }

//         car.getOdometry(odmX, odmY, odmAngle);
//         printf("X:%d Y:%d Angle:%f\n", odmX, odmY, odmAngle);

//         //qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum);//

//         //printf("x:%d y:%d angle:%f\n", x, y, angle / M_PI * 180.0);
//     }
// }

// char getch(void)
// {
//     int ch;
//     struct termios oldt, newt;

//     tcgetattr(STDIN_FILENO, &oldt);
//     memcpy(&newt, &oldt, sizeof(newt));
//     newt.c_lflag &= ~(ECHO | ICANON | ECHOE | ECHOK |
//                       ECHONL | ECHOPRT | ECHOKE | ICRNL);
//     tcsetattr(STDIN_FILENO, TCSANOW, &newt);
//     ch = getchar();
//     tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

//     return ch;
// }