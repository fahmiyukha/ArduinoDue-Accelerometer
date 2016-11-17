void setupTele(){
	Serial.begin(115200);
}

void teleSend(){
	/*
	Serial.print("iTOW:");      Serial.print(posllh.iTOW);
	Serial.print(" lat/lon: "); Serial.print(posllh.lat / 10000000.0f); Serial.print(","); Serial.print(posllh.lon / 10000000.0f);
	Serial.print(" height: ");  Serial.print(posllh.height / 1000.0f);
	Serial.print(" hMSL: ");    Serial.print(posllh.hMSL / 1000.0f);
	Serial.print(" hAcc: ");    Serial.print(posllh.hAcc / 1000.0f);
	Serial.print(" vAcc: ");    Serial.print(posllh.vAcc / 1000.0f);

	Serial.print(" F: ");    Serial.print(frontHeading);
	Serial.print(" H: ");    Serial.print(heading);
	Serial.print(" HD: ");    Serial.print(degrees);
	Serial.print(" Y: ");    Serial.print(yaw.now);
	Serial.print(" P: ");    Serial.print(pitch.now);
	Serial.print(" R: ");    Serial.print(roll.now);

	Serial.print(" | ");		Serial.print(remote[RCH3]);
	Serial.print(" | ");		Serial.print(remoteIn[PTC]);
	Serial.print(" | ");		Serial.print(remoteIn[RLL]);
	Serial.print(" | ");		Serial.print(remoteIn[THR]);

	Serial.print(" | ");		Serial.print(remoteIn[YAW]);
	Serial.print(" | ");		Serial.print(pwmMotor[0]);
	Serial.print(" | ");		Serial.print(pwmMotor[1]);
	Serial.print(" | ");		Serial.print(pwmMotor[2]);
	Serial.print(" | ");		Serial.print(pwmMotor[3]);

	Serial.print(" || ");		Serial.print(remote[RCH1]);
	Serial.print(" | ");		Serial.print(remote[RCH2]);
	Serial.print(" | ");		Serial.print(remote[RCH3]);
	Serial.print(" | ");		Serial.print(remote[RCH4]);
	Serial.print(" | ");		Serial.print(remote[RCH5]);
	Serial.print(" | ");		Serial.print(remote[RCH6]);
	Serial.print(" | ");		Serial.print(remote[RCH7]);
	Serial.print(" | ");		Serial.print(remote[RCH8]);
	Serial.print(" | ");		Serial.print(remote[RCH9]);


	Serial.print(" |T ");		Serial.print(remoteIn[THR]);
	Serial.print(" || ");		Serial.print(pwmMotor[0]);
	Serial.print(" | ");		Serial.print(pwmMotor[1]);
	Serial.print(" | ");		Serial.print(pwmMotor[2]);
	Serial.print(" | ");		Serial.print(pwmMotor[3]);
	Serial.print(" ||R ");		Serial.print(kRoll);
	Serial.print(" | ");		Serial.print(gRoll);
	Serial.print(" |P ");		Serial.print(kPitch);
	Serial.print(" | ");		Serial.print(gPitch);
	Serial.print(" |Y ");		Serial.print(kYaw);
	Serial.print(" | ");		Serial.print(gYaw);
	*/
	Serial.print(" |P: ");		Serial.print(pitch.now);
	Serial.print(" R: ");		Serial.print(roll.now);
	Serial.print(" |KP: ");		Serial.print(kalAngleY);
	Serial.print(" KR: ");		Serial.print(kalAngleX);
	/*
	Serial.print(" Y: ");		Serial.print(degrees);
	
	
	Serial.print(" |A ");		Serial.print(acc.accX.now);
	Serial.print(" | ");		Serial.print(acc.accY.now);
	Serial.print(" | ");		Serial.print(acc.accZ.now);
	Serial.print(" |V ");		Serial.print(acc.velX.now);
	Serial.print(" | ");		Serial.print(acc.velY.now);
	Serial.print(" | ");		Serial.print(acc.velZ.now);
	Serial.print(" |D ");		Serial.print(acc.disX.now);
	Serial.print(" | ");		Serial.print(acc.disY.now);
	Serial.print(" | ");		Serial.print(acc.disZ.now);
	*/
	Serial.print(" | ");		Serial.print(acc.deltaSec);
	Serial.print(" | ");		Serial.print(acc.deltaTime);
	Serial.print(" | ");		Serial.print(tim.deltaTime);

	Serial.println();



}

void serialEvent()
{
	digitalWrite(53, LOW);
	digitalWrite(A11, HIGH);

	if (Serial.available())
	{
		data = Serial.read();


		if (data == '1'){ status = 1; }
		else if (data == '2'){ status = 2; }
		else if (data == '3'){ status = 3; }

		if (data == 'w'){ val += 50; }
		else if (data == 's'){ val -= 50; }
		else if (data == 'z')
		{
			val = 1061;
		}


		if (status == 1)
		{
			if (data == 'r'){ kRoll += 0.1; }
			else if (data == 'f'){ kRoll -= 0.1; }
			else if (data == 't'){ kRoll += 0.01; }
			else if (data == 'g'){ kRoll -= 0.01; }
			else if (data == 'v')
			{
				kRoll = 0;
			}

			else if (data == 'u'){ gRoll += 0.1; }
			else if (data == 'j'){ gRoll -= 0.1; }
			else if (data == 'i'){ gRoll += 0.01; }
			else if (data == 'k'){ gRoll -= 0.01; }
			else if (data == 'm')
			{
				gRoll = 0;
			}

		}
		else if (status == 2)
		{
			if (data == 'r'){ kPitch += 0.1; }
			else if (data == 'f'){ kPitch -= 0.1; }
			else if (data == 't'){ kPitch += 0.01; }
			else if (data == 'g'){ kPitch -= 0.01; }
			else if (data == 'v')
			{
				kPitch = 0;
			}

			else if (data == 'u'){ gPitch += 0.1; }
			else if (data == 'j'){ gPitch -= 0.1; }
			else if (data == 'i'){ gPitch += 0.01; }
			else if (data == 'k'){ gPitch -= 0.01; }
			else if (data == 'm')
			{
				gPitch = 0;
			}

		}
		else if (status == 3)
		{
			if (data == 'r'){ kYaw += 0.1; }
			else if (data == 'f'){ kYaw -= 0.1; }
			else if (data == 't'){ kYaw += 0.01; }
			else if (data == 'g'){ kYaw -= 0.01; }
			else if (data == 'v')
			{
				kYaw = 0;
			}

			else if (data == 'u'){ gYaw += 0.1; }
			else if (data == 'j'){ gYaw -= 0.1; }
			else if (data == 'i'){ gYaw += 0.01; }
			else if (data == 'k'){ gYaw -= 0.01; }
			else if (data == 'm')
			{
				gYaw = 0;
			}

		}


	}


}

