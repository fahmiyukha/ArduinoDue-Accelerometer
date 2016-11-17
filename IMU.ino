
void setupAHRS(){

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(1000000);
	//TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	mpu.initialize();
	mpu.setXGyroOffset(70);
	mpu.setYGyroOffset(-35);
	mpu.setZGyroOffset(32);
	mpu.setZAccelOffset(800);

	acc.timer.now = micros();
	acc.accX.now = 0;
	acc.accY.now = 0;
	acc.accZ.now = 0;

	acc.velX.now = 0;
	acc.velY.now = 0;
	acc.velZ.now = 0;

	acc.disX.now = 0;
	acc.disY.now = 0;
	acc.disZ.now = 0;

	filter.begin(1000);
	// initialize variables to pace updates to correct rate
	microsPerReading = 1000000 / 1000;
	microsPrevious = micros();

}

void readAHRS(){

	// check if it's time to read data and update the filter
	microsNow = micros();
	
	if (microsNow - microsPrevious >= microsPerReading) {

		tim.timer.before = tim.timer.now;
		tim.timer.now = micros();

		tim.deltaTime = abs(tim.timer.now - tim.timer.before);
		tim.deltaSec = (double)(tim.deltaTime / 1000000.0);

		// read raw data from CurieIMU
		mpu.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);
		//CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

		// convert from raw data to gravity and degrees/second units
		ax = convertRawAcceleration(aix);
		ay = convertRawAcceleration(aiy);
		az = convertRawAcceleration(aiz);
		gx = convertRawGyro(gix);
		gy = convertRawGyro(giy);
		gz = convertRawGyro(giz);

		// update the filter, which computes orientation
		filter.updateIMU(gx, gy, gz, ax, ay, az);

		// print the heading, pitch and roll
		roll.now = filter.getRoll();
		pitch.now = filter.getPitch();
		yaw.now = filter.getYaw();
		/*
		Serial.print("Orientation: ");
		Serial.print(yaw.now);
		Serial.print(" ");
		Serial.print(pitch.now);
		Serial.print(" ");
		Serial.println(roll.now);
		*/

		// increment previous time, so we keep proper pace
		microsPrevious = microsPrevious + microsPerReading; //microsPrevious + microsPerReading;
	}

}

float convertRawAcceleration(int aRaw) {
	// since we are using 2G range
	// -2g maps to a raw value of -32768
	// +2g maps to a raw value of 32767

	float a = (aRaw * 2.0) / 32768.0;
	return a;
}

float convertRawGyro(int gRaw) {
	// since we are using 250 degrees/seconds range
	// -250 maps to a raw value of -32768
	// +250 maps to a raw value of 32767

	float g = (gRaw * 250.0) / 32768.0;
	return g;
}

/*
void dmpDataReady() {
	mpuInterrupt = true;
}

void setupIMU() {

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	//TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	mpu.initialize();
	attachInterrupt(digitalPinToInterrupt(5), dmpDataReady, RISING);
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(12); //12
	mpu.setYGyroOffset(-18); //-18
	mpu.setZGyroOffset(41); //41
	mpu.setZAccelOffset(1009); //1009
	mpu.setYAccelOffset(-338); //-338
	mpu.setXAccelOffset(220); //220

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		attachInterrupt(digitalPinToInterrupt(DMP_INT), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		Serial.println("ERROR INITIALIZE!");
		while (1)
		{}
	}

	acc.timer.now = micros();
	acc.accX.now = 0;
	acc.accY.now = 0;
	acc.accZ.now = 0;

	acc.velX.now = 0;
	acc.velY.now = 0;
	acc.velZ.now = 0;

	acc.disX.now = 0;
	acc.disY.now = 0;
	acc.disZ.now = 0;

}

void readIMU() {
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	fifoCount = mpu.getFIFOCount();

	acc.timer.now = micros();
	acc.deltaTime = acc.timer.now - acc.timer.before;
	acc.timer.before = acc.timer.now;
	
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {

		yaw.now = yaw.before;
		pitch.now = pitch.before;
		roll.now = roll.before;
		Serial.println("ERROR");
		mpu.resetFIFO();

	}
	else if (mpuIntStatus & 0x02) {
		if (fifoCount < packetSize) {}//fifoCount = mpu.getFIFOCount();
		else
		{
			mpu.getFIFOBytes(fifoBuffer, packetSize);
			fifoCount -= packetSize;

			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

			yaw.now = ypr[0] * 180 / M_PI;
			pitch.now = ypr[1] * 180 / M_PI;
			roll.now = ypr[2] * 180 / M_PI;

			yaw.before = yaw.now;
			pitch.before = pitch.now;
			roll.before = roll.now;

			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			
			acc.accX.before = acc.accX.now;
			acc.accY.before = acc.accY.now;
			acc.accZ.before = acc.accZ.now;

			acc.accX.now = aaReal.x;
			acc.accY.now = aaReal.y;
			acc.accZ.now = aaReal.z;


			acc.timer.now = micros();
			acc.deltaTime = acc.timer.now - acc.timer.before;
			acc.timer.before = acc.timer.now;

			acc.deltaSec = (double)(acc.deltaTime / 1000000.0);

			acc.velX.before = acc.velX.now;
			acc.velY.before = acc.velY.now;
			acc.velZ.before = acc.velZ.now;

			acc.velX.now = acc.velX.before + (aaReal.x * acc.deltaSec * cos(DEG_TO_RAD * pitch.now));
			acc.velY.now = acc.velY.before + (aaReal.y * acc.deltaSec * cos(DEG_TO_RAD * roll.now));
			acc.velZ.now = acc.velZ.before + (aaReal.z * acc.deltaSec);

			acc.disX.before = acc.disX.now;
			acc.disY.before = acc.disY.now;
			acc.disZ.before = acc.disZ.now;

			acc.disX.now = acc.disX.before + (acc.velX.now *acc.deltaSec * cos(DEG_TO_RAD * pitch.now));
			acc.disY.now = acc.disY.before + (acc.velY.now *acc.deltaSec * cos(DEG_TO_RAD * roll.now));
			acc.disZ.now = acc.disZ.before + (acc.velZ.now *acc.deltaSec);
		
			// blink LED to indicate activity
			blinkState = !blinkState;
			digitalWrite(13, blinkState);
		}
		
	}
	
}
*/
void setupKalman(){

	kalmanX.setAngle(roll.now); // Set starting angle
	kalmanY.setAngle(pitch.now);
	gyroXangle = roll.now;
	gyroYangle = pitch.now;
	compAngleX = roll.now;
	compAngleY = pitch.now;

	timer = micros();
}

void kalmanFilter(){
	double dt = (double)(micros() - timer) / 1000000.0; // Calculate delta time
	timer = micros();

	double gyroXrate = gix / 131.0; // Convert to deg/s
	double gyroYrate = giy / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll.now < -90 && kalAngleX > 90) || (roll.now > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll.now);
		compAngleX = roll.now;
		kalAngleX = roll.now;
		gyroXangle = roll.now;
	}
	else
		kalAngleX = kalmanX.getAngle(roll.now, gyroXrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleX) > 90)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleY = kalmanY.getAngle(pitch.now, gyroYrate, dt);
#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch.now < -90 && kalAngleY > 90) || (pitch.now > 90 && kalAngleY < -90)) {
		kalmanY.setAngle(pitch.now);
		compAngleY = pitch.now;
		kalAngleY = pitch.now;
		gyroYangle = pitch.now;
	}
	else
		kalAngleY = kalmanY.getAngle(pitch.now, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleY) > 90)
		gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleX = kalmanX.getAngle(roll.now, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

	//gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	//gyroYangle += gyroYrate * dt;
	gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	gyroYangle += kalmanY.getRate() * dt;

	compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll.now; // Calculate the angle using a Complimentary filter
	compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch.now;

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
		gyroYangle = kalAngleY;


}

