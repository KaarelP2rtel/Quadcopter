#include "mbed.h"
#include "MPU6050.h"
#include <inttypes.h>
#include "nRF24L01P.h"

#define LOOP_PERIOD_US 2500 //Loop frequency 400Hz
#define ESC_BASE_SPEED 900
#define BAUDRATE 460800

#define KP 100 //Pid loop values
#define KD 300
#define KI 300

MPU6050 mpu;

Serial pc(USBTX, USBRX);
float pitchF, rollF, pG, rG, pA, rA, tA;
Ticker loopTicker;

float prevPitchError, prevRollError;

PwmOut m0(D0), //ESC0
    m1(D1),    //ESC1
    m2(D7),    //ESC2
    m3(D9);    //ESC3

Timer timer; //Timer for printing loop refresh rate

volatile bool loopTick = false;

void loopIsr()
{
    loopTick = true;
}
void pidLoop()
{
    float pitchError = 0 - pitchF;
    float rollError = 0 - rollF;

}

void initMotors()
{

    m0.period_us(LOOP_PERIOD_US);
    m1.period_us(LOOP_PERIOD_US);
    m2.period_us(LOOP_PERIOD_US);
    m3.period_us(LOOP_PERIOD_US);

    m0.pulsewidth_us(0); //Start off motors at 0us
    m1.pulsewidth_us(0);
    m2.pulsewidth_us(0);
    m3.pulsewidth_us(0);
    pc.printf("ESCs waiting\r\n");
}
void initSerial()
{
    pc.baud(BAUDRATE);
    pc.printf("Serial started :)\r\n");
}
void initTicker()
{
    loopTicker.attach_us(loopIsr, LOOP_PERIOD_US);
    pc.printf("Loop Ticker attached\r\n");
}

void calculateAndPrintRefreshRate()
{

    int rate = 1000000 / timer.read_us();
    pc.printf("Rate: %d \n\r", rate);
    timer.reset();
}
void readPitchRoll()
{

    int16_t acc[3];
    int16_t gyr[3];
    mpu.readAccelData(acc);
    mpu.readGyroData(gyr);
    float rate = 400.0;

    pG += (gyr[0] / rate) / 65.5;
    rG += (gyr[1] / rate) / 65.5;

    //Translate Yaw into Pitch and roll
    rG -= pG * sin(((gyr[2] / rate) / 65.5) * (3.14 / 180));
    pG += rG * sin(((gyr[2] / rate) / 65.5) * (3.14 / 180));

    //Calculate angle by accelerometer
    tA = sqrt(pow(acc[0], 2) + pow(acc[1], 2) + pow(acc[2], 2));
    pA = asin(acc[1] / tA) * 57.96;
    rA = asin(acc[0] / tA) * -57.96;

    //pc.printf("R: %d\t P: %d\n\r",(int)(rG*100),(int)(pG*100));
    //Calclulate Angle from Gyro and accelerometer
    pitchF = 0.95 * pG + 0.05 * pA;
    rollF = 0.95 * rG + 0.05 * rA;

    pG = pitchF;
    rG = rollF;
}

int main()
{

    timer.start();
    initSerial();

    mpu.initMPU6050();
    pc.printf("MPU started\n\r");

    initTicker();

    while (1)
    {
        //calculateAndPrintRefreshRate();

        if (loopTick)
        {
            readPitchRoll();
            pidLoop();
            loopTick = false;
        }
    }
}
