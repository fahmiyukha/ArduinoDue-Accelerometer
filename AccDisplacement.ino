/*
*Author : Fahmi Yukha S.
*Device : Arduino DUe
*/

/************************************HEADER***********************************/
//TASK SCHEDULER
//#include <Scheduler.h>
#include "math.h"
#include "MadgwickAHRS\MadgwickAHRS.h"

//Kalman Filter
#include "KalmanFilter\Kalman.h"

//MPU6050DMP
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//ESC Motor
#include "Servo.h"

//Magneto
#include <HMC5883L.h>

/************************************HEADER***********************************/

/************************************MACRO***********************************/
//MPU6050DMP
#define LED_PIN 13 
#define DMP_INT 5

#define NOW 0
#define BEFORE 1

//LED-Notif
#define B_LED 52
#define G_LED 53
#define R_LED A11

//ESC-Motor
#define PIN_M1 8
#define PIN_M2 9
#define PIN_M3 6
#define PIN_M4 7

#define FRONT_RIGHT 0
#define FRONT_LEFT 1
#define REAR_LEFT 3
#define REAR_RIGHT 4

//REMOTE
#define CH1_PIN A9
#define CH2_PIN A8
#define CH3_PIN A7
#define CH4_PIN A6
#define CH5_PIN A5
#define CH6_PIN A4
#define CH7_PIN A3
#define CH8_PIN A2
#define CH9_PIN A10

#define RCH1 0
#define RCH2 1
#define RCH3 2
#define RCH4 3
#define RCH5 4
#define RCH6 5
#define RCH7 6
#define RCH8 7
#define RCH9 8

#define PTC 0
#define RLL 1
#define THR 2
#define YAW 3

//Kalman
#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf


/************************************MACRO***********************************/


/**********************************REFERENCE**********************************/
//MPU6050DMP
MPU6050 mpu;

//ESC-Motor
Servo motor1, motor2, motor3, motor4;

//Magneto
HMC5883L compass;
//Magdwick
Madgwick filter;

//Kalman
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

//GPS
struct NAV_POSLLH {
	unsigned char cls;
	unsigned char id;
	unsigned short len;
	unsigned long iTOW;
	long lon;
	long lat;
	long height;
	long hMSL;
	unsigned long hAcc;
	unsigned long vAcc;
};

struct dataF {
	float now;
	float before;
};

struct dataD {
	double now;
	double before;
};

struct dataI {
	int16_t now;
	int16_t before;
};

struct dataU {
	uint16_t now;
	uint16_t before;
};

struct dataL{
	long now;
	long before;
};

struct accelRaw {
	dataF accX;
	dataF accY;
	dataF accZ;
	dataF velX;
	dataF velY;
	dataF velZ;
	dataF disX;
	dataF disY;
	dataF disZ;
	dataL timer;
	long deltaTime;
	double deltaSec;
};

accelRaw acc,tim;
NAV_POSLLH posllh;
/**********************************REFERENCE**********************************/

/**********************************VARIABLE***********************************/
/*
//MPU6050DMP
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
*/
dataF yaw;
dataF pitch,pitchKal;
dataF roll,rollKal;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//int16_t ax, ay, az;
//int16_t gx, gy, gz;


//motor
uint16_t pwmMotor[4];

//Magneto
int16_t mx, my, mz;
float kompas;
float degrees;
float heading;
float frontHeading;
boolean active;

//GPS
const unsigned char UBX_HEADER[] = { 0xB5, 0x62 };
bool GPSDone = false;
bool GPSState = false;

//Telemetry
unsigned long timerPrintOut;

//TestMotor
char data;
int val = 1000;
int status = 0;

//Kendali
const double AInv[4][4] = {
	{ -290903.6748, 2461167.149, -2461167.149, 72726003.33 },
	{ -290903.6748, 2461167.149, 2461167.149, -72726003.33 },
	{ -290903.6748, -2461167.149, 2461167.149, 72726003.33 },
	{ -290903.6748, -2461167.149, -2461167.149, -72726003.33 }
};

float kRoll = 3.18, gRoll = 1.6; //0.34 0.16
float kPitch = 3.13, gPitch = 1.6; //0.36 0.16
float kYaw = 4, gYaw = 1.04; // 0.27 1.76

double u1, u2, u3, u4;
double w0, w1, w2, w3, w4;

double constM1 = 0.7066;
double constM2 = 0.7344;
double constM3 = 0.7161;
double constM4 = 0.6998;

//Remote
int remotePin[9] = { CH1_PIN, CH2_PIN, CH3_PIN, CH4_PIN, CH5_PIN, CH6_PIN, CH7_PIN, CH8_PIN, CH9_PIN };
long remoteTimer[9], remoteSignal[9];
int remote[9];
int remoteIn[4];
bool motorOn = false;

//Magdwick
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;
int16_t aix, aiy, aiz;
int16_t gix, giy, giz;
float ax, ay, az;
float gx, gy, gz;
//float roll, pitch, heading;
unsigned long microsNow;

//Kalman

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
/**********************************VARIABLE***********************************/

void setup()
{
	setupTele();
	//setupIMU();
	//Scheduler.startLoop(loop2);
	setupAHRS(); 
	setupKalman();
	pinMode(13, OUTPUT);
	/* add setup code here */

}

void loop()
{

	acc.timer.before = acc.timer.now;
	acc.timer.now = micros();

	acc.deltaTime = abs(acc.timer.now - acc.timer.before);
	acc.deltaSec = (double)(acc.deltaTime / 1000000.0);
	//mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	//readIMU();
	readAHRS();
	kalmanFilter();
	//LQRCalc();
	if ((millis() - timerPrintOut) > 100)
	{
		timerPrintOut = millis();
		teleSend();
	}


	/* add main program code here */

}
