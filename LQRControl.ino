
/*
void LQRCalc(){
	u1 = 0;
	u2 = (kRoll * (roll.now + remoteIn[RLL]) / 10000000) + (gRoll*(gx) / 100000000);
	u3 = (kPitch * (pitch.now + remoteIn[PTC]) / 10000000) + (gPitch*(-gy) / 100000000);
	u4 = (kYaw * (degrees) / 100000000) + (gYaw*(-gz) / 1000000000);

	//omega 
	w1 = AInv[0][0] * u1 + AInv[0][1] * u2 + AInv[0][2] * u3 + AInv[0][3] * u4;
	w2 = AInv[1][0] * u1 + AInv[1][1] * u2 + AInv[1][2] * u3 + AInv[1][3] * u4;
	w3 = AInv[2][0] * u1 + AInv[2][1] * u2 + AInv[2][2] * u3 + AInv[2][3] * u4;
	w4 = AInv[3][0] * u1 + AInv[3][1] * u2 + AInv[3][2] * u3 + AInv[3][3] * u4;


	//omega to pwm
	pwmMotor[0] = (int)((w1 / constM1) + remoteIn[THR]);
	pwmMotor[1] = (int)((w2 / constM2) + remoteIn[THR]);
	pwmMotor[2] = (int)((w3 / constM2) + remoteIn[THR]);
	pwmMotor[3] = (int)((w4 / constM4) + remoteIn[THR]);


	pwmMotor[0] = constrain(pwmMotor[0], 1061, 1900);
	pwmMotor[1] = constrain(pwmMotor[1], 1061, 1900);
	pwmMotor[2] = constrain(pwmMotor[2], 1061, 1900);
	pwmMotor[3] = constrain(pwmMotor[3], 1061, 1900);

}
*/