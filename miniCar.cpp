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
#include<netdb.h>	//hostent
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

//PID Controllers
PIDContorller angularPID(0.025, 0.000, 0.0, 3, -3, 0.05);
PIDContorller linearPID(0.45, 0.000, 0.00, 1000, -1000, 3.0);
//PIDContorller linearPID(0.7, 0.0, 0.01, 1000, -1000, 5);

//TCP Client socket
int socketfd = 0;
struct sockaddr_in info;

void tcpOdometry(void);
void turnTo(float targetAngle);
int genQR(char ch);
void run(QRCode::qrcode_node_t *startQR);
void calculateAndRun(QRCode::qrcode_node_t *head);
void manualControl(void);
char getch(void);
int hostname_to_ip(const char * hostname , char* ip);

int main(int argc, char *argv[])
{
    system("clear");
    car.setCarParams(0, 0);
    // if (!genQR(argv[1][0]))
    // {
    //     puts("Invalid Input!");
    //     return 0;
    // }
    led.clear();    

    char ipa[15];
    const char* hostname = "DARREN-LAPTOP";
    if(hostname_to_ip(hostname,ipa) == -1){
        printf("Server %s not found!!!\n",hostname);
        return -1;
    }

    socketfd = socket(AF_INET, SOCK_STREAM, 0);
    if (socketfd == -1)
    {
        puts("Fail to create a socket.");
        return 0;
    }
    memset(&info, 0, sizeof(info));                     //初始化 將struct涵蓋的bits設為0
    info.sin_family = PF_INET;                          //sockaddr_in為Ipv4結構   
    info.sin_addr.s_addr = inet_addr(ipa); //IP address
    info.sin_port = htons(6666);
    if (connect(socketfd, (const struct sockaddr *)&info, sizeof(info)) == -1)
    {
        return 0;
    }
    std::thread tcpOdometry_t(tcpOdometry);

    while (1)
    {
        char buf[1024] = {0};
        int bytes = recv(socketfd, buf, 1024, 0);
        buf[bytes] = '\0';
        printf("%d bytes received!\n", bytes);

        headQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        QRCode::qrcode_node_t *currQR = headQR;
        QRCode::qrcode_node_t *prevQR;

        const char *split_qr = ",;";
        char *qrStr;
        qrStr = strtok(buf, split_qr);
        while (qrStr != NULL)
        {
            currQR->tagNum = atoi(qrStr);
            qrStr = strtok(NULL, split_qr);
            currQR->xPos = atoi(qrStr);
            qrStr = strtok(NULL, split_qr);
            currQR->yPos = atoi(qrStr);
            qrStr = strtok(NULL, split_qr);
            printf("TAG:%d X:%d Y:%d\n", currQR->tagNum, currQR->xPos, currQR->yPos);
            currQR->next = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
            prevQR = currQR;
            currQR = currQR->next;
        }
        free(currQR);
        prevQR->next = NULL;

        calculateAndRun(headQR);        
    }

    return 0;
}

void tcpOdometry(void)
{
    while (1)
    {
        int16_t odm[3] = {0};
        odm[0] = odmX;
        odm[1] = odmY;
        odm[2] = (int16_t)odmAngle;

        if (send(socketfd, odm, sizeof(odm), 0) == -1)
        {
            //close(socketfd);
            puts("Host Disconnected!");
            return;
        }
        if (setW > 0.1)
        {
            led.setPattern(led.matrix_Right);
        }
        else if (setW < -0.1)
        {
            led.setPattern(led.matrix_Left);
        }
        else
        {
            led.clear();
        }
        led.render();

        usleep(10000);
    }
}

void turnTo(float targetAngle)
{
    //float targetAngle = (atan2(targetY - odmY, targetX - odmX)) / M_PI * 180.0;
    //printf("%d %d\n", targetY - odmY, targetX - odmX);
    printf("Target = %f\n", targetAngle);

    while (1)
    {
        if (qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum))
        {
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
            car.getOdometry(odmX, odmY, odmAngle);
            usleep(10000);
        }
    }
}

int genQR(char ch)
{
    QRCode::qrcode_node_t *newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
    QRCode::qrcode_node_t *currentQR;
    headQR = currentQR = newQR;

    if (ch == 'f')
    {
        currentQR->xPos = 0;
        currentQR->yPos = 0;
        currentQR->tagNum = 1;
        for (int i = 2; i <= 8; i++)
        {
            newQR = (QRCode::qrcode_node_t *)malloc(sizeof(QRCode::qrcode_node_t));
            currentQR->next = newQR;
            currentQR = currentQR->next;
            currentQR->xPos = 980 * (i - 1);
            currentQR->yPos = 0;
            currentQR->tagNum = i;
        }
    }
    else if (ch == 'b')
    {
        currentQR->xPos = 980 * 7;
        currentQR->yPos = 0;
        currentQR->tagNum = 8;
        for (int i = 7; i >= 1; i--)
        {
            newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
            currentQR->next = newQR;
            currentQR = currentQR->next;
            currentQR->xPos = 980 * (i - 1);
            currentQR->yPos = 0;
            currentQR->tagNum = i;
        }
    }
    else if (ch == 'r')
    {
        currentQR->xPos = 0;
        currentQR->yPos = 980;
        currentQR->tagNum = 9;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 980;
        currentQR->yPos = 0;
        currentQR->tagNum = 2;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 980 * 2;
        currentQR->yPos = -980;
        currentQR->tagNum = 19;
    }
    else if (ch == 'i')
    {
        currentQR->xPos = 1250;
        currentQR->yPos = -890;
        currentQR->tagNum = 8;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 890;
        currentQR->yPos = -890;
        currentQR->tagNum = 5;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 0;
        currentQR->yPos = 0;
        currentQR->tagNum = 1;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 0;
        currentQR->yPos = -890;
        currentQR->tagNum = 4;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 0;
        currentQR->yPos = -890 * 2;
        currentQR->tagNum = 6;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 0;
        currentQR->yPos = -890 * 3;
        currentQR->tagNum = 7;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 0;
        currentQR->yPos = -890 * 2;
        currentQR->tagNum = 6;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 890;
        currentQR->yPos = -890;
        currentQR->tagNum = 5;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 890 * 2;
        currentQR->yPos = 0;
        currentQR->tagNum = 3;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 890;
        currentQR->yPos = 0;
        currentQR->tagNum = 2;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 0;
        currentQR->yPos = 0;
        currentQR->tagNum = 1;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 890;
        currentQR->yPos = -890;
        currentQR->tagNum = 5;

        newQR = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
        currentQR->next = newQR;
        currentQR = currentQR->next;
        currentQR->xPos = 1220;
        currentQR->yPos = -890;
        currentQR->tagNum = 8;
    }

    currentQR = headQR;
    while (currentQR != NULL)
    {
        printf("X:%d Y:%d tagNum:%d\n", currentQR->xPos, currentQR->yPos, currentQR->tagNum);
        currentQR = currentQR->next;
    }

    return 1;
}

void run(QRCode::qrcode_node_t *startQR)
{
    QRCode::qrcode_node_t *currentQR = startQR;
    QRCode::qrcode_node_t *endQR = startQR;
    while (endQR->next != NULL)
    {
        endQR = endQR->next;
    }

    int16_t targetX = endQR->xPos, targetY = endQR->yPos;
    int16_t currentTargetX = 0;
    int16_t currentTargetY = 0;
    int16_t currentTargetOffsetX = 0;
    int16_t currentTargetOffsetY = 0;
    float targetAngle = atan2(currentQR->next->yPos - currentQR->yPos, currentQR->next->xPos - currentQR->xPos);

    while (1)
    {
        if (qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum))
        {
            if (qrTagNum == startQR->tagNum)
            {
                car.setOdometry(currentQR->xPos + qrX, currentQR->yPos + qrY, degToRad(qrAngle));
                currentTargetOffsetX = currentQR->next->xPos - currentQR->xPos;
                currentTargetOffsetY = currentQR->next->yPos - currentQR->yPos;
                break;
            }
        }
    }

    //turnTo(targetX, targetY);
    turnTo(radToDeg(targetAngle));
    usleep(100000);
    while (1)
    {
        if (qrCode.getInformation(qrX, qrY, qrAngle, qrTagNum))
        {
            if (qrTagNum == currentQR->tagNum)
            {
                //printf("Update ODM with Tag %d\n", qrTagNum);
                car.setOdometry(currentQR->xPos + qrX, currentQR->yPos + qrY, degToRad(qrAngle));
            }
            if (currentQR->next != NULL)
            {
                //currentTargetOffset = pow(pow((currentQR->next->xPos - currentQR->xPos), 2) + pow(currentQR->next->yPos - currentQR->yPos, 2), 0.5);
                currentTargetOffsetX = currentQR->next->xPos - currentQR->xPos;
                currentTargetOffsetY = currentQR->next->yPos - currentQR->yPos;
                if (qrTagNum == currentQR->next->tagNum)
                {
                    currentQR = currentQR->next;
                    //printf("QR%d passed!\n", currentQR->tagNum);
                }
            }
        }
        car.getOdometry(odmX, odmY, odmAngle);

        int16_t odmRX = odmX - startQR->xPos; //Relative position to startQR
        int16_t odmRY = odmY - startQR->yPos; //Relative position to startQR
        currentTargetX = ((((float)odmRX * currentTargetOffsetX) + ((float)odmRY * currentTargetOffsetY)) / ((float)currentTargetOffsetX * currentTargetOffsetX + (float)currentTargetOffsetY * currentTargetOffsetY)) * currentTargetOffsetX + currentTargetOffsetX / 1.0 + startQR->xPos;
        currentTargetY = ((((float)odmRX * currentTargetOffsetX) + ((float)odmRY * currentTargetOffsetY)) / ((float)currentTargetOffsetX * currentTargetOffsetX + (float)currentTargetOffsetY * currentTargetOffsetY)) * currentTargetOffsetY + currentTargetOffsetY / 1.0 + startQR->yPos;

        //printf("XERR: %d, YERR: %d\n", targetX - odmX, targetY - odmY);
        float dErr = powf(powf(targetX - odmX, 2) + powf(targetY - odmY, 2), 0.5);
        //float dotErr = cosf(degToRad(odmAngle)) * (targetX - odmX) + sinf(degToRad(odmAngle)) * (targetY - odmY);
        float dotErr = dErr * cos(targetAngle - atan2(targetY - odmY, targetX - odmX));
        dErr = dotErr > 0 ? dErr : -dErr;
        //printf("dotErr = %f\n", dotErr);
        if (abs(dotErr) < 10 && abs(setV) < 10)
        {
            car.setCarParams(0, 0);
            break;
        }
        //printf("dRrr = %f\n", dErr);
        setV = linearPID.calculate(dErr);

        //printf("ctoX = %d\tctoY = %d\n", currentTargetOffsetX, currentTargetOffsetY);

        // currentTargetX = targetX;
        // currentTargetY = targetY;
        //printf("ctoX = %d\tctoY = %d\n", currentTargetX, currentTargetY);

        float currentTargetAngle = radToDeg(atan2(currentTargetY - odmY, currentTargetX - odmX));
        float err;
        err = currentTargetAngle - odmAngle;

        if (err > 180.0)
        {
            err -= 360.0;
        }
        else if (err < -180.0)
        {
            err += 360.0;
        }

        setW = -angularPID.calculate(err);

        // printf("V: %d W%f\n", setV, setW);
        car.setCarParams(setV, setW);
        printf("odmX = %d, odmY = %d, odmAngle = %f\n", odmX, odmY, odmAngle);
        //printf("%d,%d\n", odmX, odmY);
        usleep(10000);
    }
}

void calculateAndRun(QRCode::qrcode_node_t *head)
{
    QRCode::qrcode_node_t *currentQR = head;
    QRCode::qrcode_node_t *headQR_new;
    std::vector<QRCode::qrcode_node_t *> headQRs;
    headQRs.push_back(head);

    float prevTargetAngle = atan2f(currentQR->next->yPos - currentQR->yPos, currentQR->next->xPos - currentQR->xPos);
    float currentTargetAngle = 0;

    // printf("Tag%d: curr = %f\t prev=%f\n", currentQR->tagNum, currentTargetAngle, prevTargetAngle);
    currentQR = currentQR->next;
    while (currentQR->next != NULL)
    {
        // printf("xerr = %d yerr = %d\n", currentQR->next->xPos - currentQR->xPos, currentQR->next->yPos - currentQR->yPos);
        currentTargetAngle = atan2f(currentQR->next->yPos - currentQR->yPos, currentQR->next->xPos - currentQR->xPos);
        if (currentTargetAngle != prevTargetAngle)
        {
            headQR_new = (QRCode::qrcode_node_t *)calloc(0, sizeof(QRCode::qrcode_node_t));
            memcpy(headQR_new, currentQR, sizeof(QRCode::qrcode_node_t));
            headQRs.push_back(headQR_new);
            currentQR->next = NULL;
            currentQR = headQR_new->next;
            prevTargetAngle = currentTargetAngle;
            continue;
        }
        currentQR = currentQR->next;
    }

    for (int i = 0; i < headQRs.size(); i++)
    {
        printf("Turn at tag%d\n", headQRs[i]->tagNum);
    }
    //return 0;

    for (int i = 0; i < headQRs.size(); i++)
    {
        //printf("Turn at tag%d\n", headQRs[i]->tagNum);
        run(headQRs[i]);
    }
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


int hostname_to_ip(const char * hostname , char* ip)
{
	struct hostent *he;
	struct in_addr **addr_list;
	int i;
		
	if ( (he = gethostbyname( hostname ) ) == NULL) 
	{
		// get the host info		
		return -1;
	}    

	addr_list = (struct in_addr **) he->h_addr_list;    
	
	for(i = 0; addr_list[i] != NULL; i++) 
	{
		//Return the first one;
		strcpy(ip , inet_ntoa(*addr_list[i]) );
		return 0;
	}
	
	return -1;
}