#include "mbed.h"
#include "MPU6050.h"
#include <inttypes.h>
#include "RF24.h"

#define LOOP_PERIOD_US 2500 //Loop frequency 400Hz
#define ESC_BASE_SPEED 900
#define BAUDRATE 460800
#define INTEGRAL_MEMORY 0.75 //For Integral wzindup

#define KP 10 //Pid loop values
#define KD 30
#define KI 30
#define GYRO_RATIO 0.95

MPU6050 mpu;
RF24 radio(SPI_MOSI, SPI_MISO, SPI_SCK, D6, SPI_CS);

InterruptIn radioReadyInterrupt(D8);

Serial pc(USBTX, USBRX);
float pitchF, rollF, pG, rG, pA, rA, tA;
Ticker loopTicker;

float prevPitchError, prevRollError;
float pitchIntegral, rollIntegral;

PwmOut m0(D0), //ESC0
    m1(D1),    //ESC1
    m2(D7),    //ESC2
    m3(D9);    //ESC3

Timer timer; //Timer for printing loop refresh rate

int16_t gyroOffsetX, gyroOffsetY, gyroOffsetZ;

volatile bool loopTick = false;

volatile bool radioDataReady = false;

void loopIsr()
{
    loopTick = true;
}

void pidLoop()
{
    float pitchError = 0 - pitchF;
    float rollError = 0 - rollF;

    pitchIntegral += pitchError;
    rollIntegral += rollError;

    pitchIntegral *= INTEGRAL_MEMORY;
    rollIntegral *= INTEGRAL_MEMORY;

    float pitchDerivative = pitchError - prevPitchError;
    float rollDerivative = rollError - prevRollError;
    prevPitchError = pitchError;
    prevRollError = rollError;

    int pitchOff = KP * pitchError + KI * pitchIntegral + KD * pitchDerivative;
    int rollOff = KP * rollError + KI * rollIntegral + KD * rollDerivative;
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

    pc.printf("\n\r Serial started :)\r\n");
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
    gyr[0] -= gyroOffsetX;
    gyr[1] -= gyroOffsetY;
    gyr[2] -= gyroOffsetZ;
    // pc.printf("GyrX:%d Off:%d \n\r",gyr[0],gyroOffsetX);
    // pc.printf("GyrY:%d Off:%d \n\r",gyr[1],gyroOffsetY);
    // pc.printf("GyrZ:%d Off:%d \n\r",gyr[2],gyroOffsetZ );
    // pc.printf("GyX: %d\tGyY: %d\tGyZ: %d \r\n", gyr[0],gyr[1],gyr[2]);

    float rate = 400.0;

    pG -= (gyr[0] / rate) / 65.5;
    rG -= (gyr[1] / rate) / 65.5;

    //Translate Yaw into Pitch and roll
    rG -= pG * sin(((gyr[2] / rate) / 65.5) * (3.14 / 180));
    pG += rG * sin(((gyr[2] / rate) / 65.5) * (3.14 / 180));

    //Calculate angle by accelerometer
    tA = sqrt(pow(acc[0], 2) + pow(acc[1], 2) + pow(acc[2], 2));
    pA = asin(acc[1] / tA) * -57.96;
    rA = asin(acc[0] / tA) * +57.96;

    //Calclulate Angle from Gyro and accelerometer
    pitchF = GYRO_RATIO * pG + (1 - GYRO_RATIO) * pA;
    rollF = GYRO_RATIO * rG + (1 - GYRO_RATIO) * rA;

    pG = pitchF;
    rG = rollF;
}
void dataReadyIsr()
{
    radioDataReady = true;
}
void initMpu()
{
    mpu.initMPU6050();
    wait_ms(300);

    int16_t gyr[3];
    int16_t xSum, ySum, zSum;
    int16_t count = 50;
    for (int i = 0; i < count; i++)
    {

        mpu.readGyroData(gyr);
        xSum += gyr[0];
        ySum += gyr[1];
        zSum += gyr[2];
        wait_ms(10);
    }

    gyroOffsetX = xSum / count;
    gyroOffsetY = ySum / count;
    gyroOffsetZ = zSum / count;
    pc.printf("GyroX:%d Offset:%d\n\r", xSum, gyroOffsetX);
    pc.printf("GyroY:%d Offset:%d\n\r", ySum, gyroOffsetY);
    pc.printf("GyroZ:%d Offset:%d\n\r", zSum, gyroOffsetZ);

    pc.printf("MPU Started\n\r");
}
void initRadio()
{
    radioReadyInterrupt.fall(&dataReadyIsr);

    radio.begin();

    radio.setChannel(0x7D);
    radio.setPALevel(RF24_PA_MIN);
    radio.setDataRate(RF24_1MBPS);

    radio.maskIRQ(true, true, false);

    radioReadyInterrupt.enable_irq();

    radio.openReadingPipe(0, 0xE7E7E7E7E7);

    radio.startListening();
    pc.printf("Channel: %d \n\r", radio.getChannel());
}
struct RxPackage
{
    uint16_t throttle, pitch, roll, yaw;
    uint8_t bluePot;
    uint8_t redPot;
};
RxPackage rxPackage;

int main()
{

    timer.start();
    initSerial();
    initMpu();
    initTicker();
    initRadio();
    while (1)
    {
        calculateAndPrintRefreshRate();

        if (loopTick)
        {
            readPitchRoll();
            pidLoop();
            loopTick = false;
        }

        if (radioDataReady)
        {
            radio.read(&rxPackage, sizeof(rxPackage));

            //pc.printf("%d\n\r", rxPackage.throttle);

            radioDataReady = false;
        }
        // pc.printf("Pitch: %d ", (int)(pitchF * 100));
        // pc.printf("Roll: %d\n\r", (int)(rollF * 100));
    }
}