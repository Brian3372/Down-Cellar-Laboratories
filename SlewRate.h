#ifndef SlewRate_h
#define SlewRate_h

typedef struct{
	float maxAccel;
	float lastRate;
	float lastTime;
} SlewRate;

#endif
