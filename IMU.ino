void setupAHRS(){

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000);//I2C Clock Speed
	//TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif

	mpu.initialize();
	mpu.setXGyroOffset(70);
	mpu.setYGyroOffset(-35);
	mpu.setZGyroOffset(32);
	mpu.setZAccelOffset(800);

	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_1000);
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8);


	acc.timer.now = micros();
	tim.timer.now = micros();

	filter.begin(1000);

	// initialize variables to pace updates to correct rate
	microsPerReading = 1000000 / 1000;
	microsPrevious = micros();

}

void readAHRS(){

	// check if it's time to read data and update the filter
	//microsNow = micros();

	//if (microsNow - microsPrevious >= microsPerReading) {

	tim.timer.before = tim.timer.now;
	tim.timer.now = micros();

	tim.deltaTime = abs(tim.timer.now - tim.timer.before);//Calc Delta getDegreeAHRS
	tim.deltaSec = (double)(tim.deltaTime / 1000000.0);

	// read raw data from MPU6050
	mpu.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

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
	microsPrevious = microsPrevious + microsPerReading;
	//}

}

float convertRawAcceleration(int aRaw) {
	// since we are using 2G range
	// -2g maps to a raw value of -32768
	// +2g maps to a raw value of 32767

	float a = (aRaw * 8.0) / 32768.0;//change it if you change setRange accel MPU6050
	return a;
}

float convertRawGyro(int gRaw) {
	// since we are using 250 degrees/seconds range
	// -250 maps to a raw value of -32768
	// +250 maps to a raw value of 32767

	float g = (gRaw * 1000.0) / 32768.0;//change it if you change setRange gyro MPU6050
	return g;
}

void setupKalman(){

	kalmanX.setAngle(roll.now); // Set starting angle
	kalmanY.setAngle(pitch.now);

	kalmanAccX.setAngle(ax); // Set starting angle
	kalmanAccY.setAngle(ay);

	timer = micros();
}

void kalmanFilter(){
	kalTim = (micros() - timer);
	double dt = (double)(micros() - timer) / 1000000.0; // Calculate delta time
	timer = micros();

	double gyroXrate = gix / 131.0; // Convert to deg/s
	double gyroYrate = giy / 131.0; // Convert to deg/s
	double accelXrate = ax;
	double accelYrate = ay;


#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll.now < -90 && kalAngleX > 90) || (roll.now > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll.now);
		kalAngleX = roll.now;
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
		kalAngleY = pitch.now;
	}
	else
		kalAngleY = kalmanY.getAngle(pitch.now, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleY) > 90)
		gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleX = kalmanX.getAngle(roll.now, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

	//accX = kalmanAccX.getAngle()

}

