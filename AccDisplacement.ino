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

//Magdwick
Madgwick filter;

//Kalman
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
Kalman kalmanAccX;
Kalman kalmanAccY;

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
/**********************************REFERENCE**********************************/

/**********************************VARIABLE***********************************/

//MPU6050DMP

dataF yaw;
dataF pitch,pitchKal;
dataF roll,rollKal;

//Telemetry
unsigned long timerPrintOut;
int status = 0;

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
	Serial.begin(115200);
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
		Serial.print(" |P: ");		Serial.print(pitch.now);//Print Pitch Madgwick AHRS
		Serial.print(" R: ");		Serial.print(roll.now);//Print Roll Madgwick AHRS
		Serial.print(" |KP: ");		Serial.print(kalAngleY);//Print Pitch Kalman
		Serial.print(" KR: ");		Serial.print(kalAngleX);//Print Roll Kalman
		Serial.print(" | ");		Serial.print(acc.deltaTime);
		Serial.print(" | ");		Serial.print(tim.deltaTime);
	}


	/* add main program code here */

}
