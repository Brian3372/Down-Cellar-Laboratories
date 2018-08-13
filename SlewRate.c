#include "SlewRate.h"

/**
* initialize Slew Rate structure, set parameters
*
* @param slew rate instance of the SlewRate struct
* @param desired maximum accleration
*/
void
slewRateInit (SlewRate slew, float acceleration) {
	slew.maxAccel = acceleration;
	slew.lastRate = 0;
	slew.lastTime = nPgmTime/1000.0;
}

/**
* calculate rate output
*
* @param reference slew rate struct
* @param desired ouput
*
* @return output value constrained from -127 to 127
*/
float
rateCalculate (SlewRate rate, float desiredRate) {
		float deltaTime = nPgmTime/1000.0-rate.lastTime;
		float desiredAccel = (desiredRate - rate.lastRate)/deltaTime;
		float addedRate;
		float newRate;

		if (abs(desiredAccel) < rate.maxAccel) {
		    addedRate = desiredAccel*deltaTime;
		    newRate = addedRate+rate.lastRate;
		}
		else {
		    addedRate = ((desiredAccel>0)? 1: -1)*rate.maxAccel*deltaTime;
        newRate = addedRate+rate.lastRate;
		}
		rate.lastTime = rate.lastTime+deltaTime;
		rate.lastRate = newRate;

		float returnVal = newRate;
		return returnVal;
}

void slewRateReset(SlewRate slew) {
	slew.lastRate = 0;
	slew.lastTime = nPgmTime/1000.0;
}
