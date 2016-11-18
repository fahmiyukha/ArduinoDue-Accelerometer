/*
*Author : Fahmi Yukha S.
*Device : Arduino Due
*/

/************************************HEADER***********************************/
//TASK SCHEDULER

#include <DueTimer.h>
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

/************************************HEADER***********************************/

/************************************MACRO***********************************/
//MPU6050DMP
#define LED_PIN 13 
#define DMP_INT 5

#define NOW 0
#define BEFORE 1

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

accelRaw acc, tim;
/**********************************REFERENCE**********************************/

/**********************************VARIABLE***********************************/

//MadwickAHRS

dataF yaw;
dataF pitch, pitchKal;
dataF roll, rollKal;

//Telemetry
unsigned long timerPrintOut;

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

double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint32_t kalTim;
/**********************************VARIABLE***********************************/

void setup()
{
	Serial.begin(115200);
	setupAHRS();

	Timer8.attachInterrupt(readAHRS);
	Timer8.setPeriod(1000);
	Timer8.start(1000);

	setupKalman();
	pinMode(13, OUTPUT);
	/* add setup code here */

}

void loop()
{

	acc.timer.before = acc.timer.now;
	acc.timer.now = micros();

	acc.deltaTime = abs(acc.timer.now - acc.timer.before);//Calc Delta Loop
	acc.deltaSec = (double)(acc.deltaTime / 1000000.0);

	//readAHRS();
	kalmanFilter();

	if ((millis() - timerPrintOut) > 100)
	{
		timerPrintOut = millis();
		Serial.print(" |P: ");		Serial.print(pitch.now);//Print Pitch Madgwick AHRS
		Serial.print(" R: ");		Serial.print(roll.now);//Print Roll Madgwick AHRS
		Serial.print(" |KP: ");		Serial.print(kalAngleY);//Print Pitch Kalman
		Serial.print(" KR: ");		Serial.print(kalAngleX);//Print Roll Kalman
		Serial.print(" | ");		Serial.print(acc.deltaTime);
		Serial.print(" | ");		Serial.print(tim.deltaTime);
		Serial.print(" | ");		Serial.print(kalTim);

		Serial.println();
	}


	/* add main program code here */

}
