/*
Driving PID and turning libraries courtesy of Brian Boxell, DCL team 9605A

TrueSpeed mapping courtesy of Jordan Kiesel, BNS/24c

PID and Gyro libraries courtesy of Jason McKinney, QCC2/2625

All of this is designed to work with the Vex EDR system. Motors are given values between -127 and 127. The Truespeed mapping accounts
for the nonlinearity of the motor values so that you get smoother and more accurate accelerations. The gyro library by Jason McKinney is
specifically designed to account for drift in the Vex yaw rate gyroscope. Depending on the quality of your gyro, you may be able to omit
the functions in the monitorGyro task, and just use the raw data of the gyro.

In the implementation of this code, there are a bunch of variables and things defined elsewhere. These include global variables (notably the
gyroAngle float), and the initializations of the PIDs. You must initialize all PIDs at the beginning of your program.4
*/


/*
* uses PID to drive to the desired target, within the specified accuracy
* @param target: integer target value to drive to
* @param accuracy: integer value asssigned to the desired accuracy (drives to target +- this number)
*/
void driveTarget(int target, int accuracy) {
	bool atTarget = false;
	int targetAngle = gyroAngle;
	int repsAtTarget = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!atTarget) {
		//calculate the value the motors should be set at based on its position relative to the target
		int val = pidCalculate(drivePID, target, SensorValue[encoder]);
		//the left and right drive values should be different in order to correct getting turned as specified by the gyro value
		int leftVal = val + pidCalculate(gyroDrivePID, targetAngle, gyroAngle);
		int rightVal = val - pidCalculate(gyroDrivePID, targetAngle, gyroAngle);
		motor[frontRightMotor] = motor[frontLeftMotor] = ((leftVal) >= 0 ? 1 : -1)*TrueSpeed[min(fabs(leftVal),127)];
		motor[rearRightMotor] = motor[rearLeftMotor] = ((rightVal) >= 0 ? 1 : -1)*TrueSpeed[min(fabs(rightVal),127)]
		//if the sensor value is within the desired range of the target
		if (abs(SensorValue[encoder]-target) < accuracy) {
			//if the sensor value is within the range for multiple iterations of the loop where each loop is approximately 20ms
			if (repsAtTarget > 12) {
				//break out of the while loop
				atTarget = true;
			}
			else {
				repsAtTarget++;
			}
		}
		else {
			repsAtTarget = 0;
		}
	}
	//zero the value of the encoder so that the next time this procedure is called, the encoder will be starting at zero again
	SensorValue[encoder] = 0;
}


//updates the angle of the robot, makes it so that the angle is always described to be within -180 <= angle <= 180 degrees.
//this task runs in the background, the code could also just be copy and pasted into the main loop of the program and work the same.
task monitorGyro() {
	while (true) {
		//takes the summation of the gyro turning rates over time (the integral of the rate is the position)
		float deltaTime = (nPgmTime - time)/1000.0;
		time = nPgmTime;
		gyroAngle += SensorValue[gyroRaw]*deltaTime;
		wait1Msec(20);
		//makes sure that the gyro angle is always represented between -180 and 180
		if (fabs(gyroAngle)>180) {
			gyroAngle = ((gyroAngle>0)? -1: 1)*(360-fabs(gyroAngle));
		}
		wait1Msec(20);
	}
}


/*
* uses PID to turn to the desired target, within the specified accuracy
* @param target: integer target value to drive to
* @param accuracy: integer value asssigned to the desired accuracy (drives to target +- this number)
*/
void gyroTurn(int target, int accuracy) {
	gyroAtTarget = false;
	gyroTarget = target;
	int repsAtTarget = 0;
	//go into the loop that will repeat to update motor values and break when at target
	while (!gyroAtTarget) {
		// calculate the desired motor value based on the sensor value relative to the target
		float drive = pidCalculate(gyroPID, target, gyroAngle);
		drive = ((fabs(gyroAngle-target)>180)? -1 : 1)*drive;
		driveL(drive);
		driveR(drive);
		gyroError = fabs(gyroAngle-target);
		//if the sensor value is within the desired range of the target
		if (fabs(gyroAngle-target) < accuracy) {
			//if the sensor value is within the range for multiple iterations of the loop where each loop is approximately 20ms
			if (repsAtTarget > 12) {
				//break out of the loop
				gyroAtTarget = true;
			}
			else {
				repsAtTarget++;
			}
		}
		else {
			repsAtTarget = 0;
		}
	}
	SensorValue[encoder] = 0; //zero the value of the drive encoder
}

//called from the gyro turn task, makes sure it is a valid motor input and updates drive motors;
void driveL(int val){
	val = (abs(val) > 127)? 127 * val/abs(val) : val; //ensure val is under 127
	motor[rearLeftMotor] = motor[frontLeftMotor] = ((val) >= 0 ? 1 : -1)*TrueSpeed[min(fabs(val),127)];
}
// same thing as driveL, but with a different negative sign
void driveR(int val){
	val = abs(val) > 127 ? 127 * val/abs(val) : val; //ensure val is under 127
	motor[rearRightMotor] = motor[frontRightMotor] = ((val) >= 0 ? -1 : 1)*TrueSpeed[min(fabs(val),127)];
}

//returns the minimum of 2 parameters, used in the TrueSpeed call
int min(int a, int b){
	if(a>b)
		return b;
	return a;
}
