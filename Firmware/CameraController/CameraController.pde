#include "SonyFCB.h"

const uint8_t pinLED = 23;
const uint8_t pinCamEn = 13;
const uint8_t pinServoEn = 20;
const uint8_t pinPan = 18;
const uint8_t pinTilt = 19;
const uint8_t pinU1TX = 11;
const uint8_t pinU1RX = 12;
const uint8_t pinU2TX = 25;
const uint8_t pinu2RX = 24;

SonyFCB cam = SonyFCB(Serial1, 1);

void setup(void) {
	// Setup status
	pinMode(pinLED, OUTPUT);
	digitalWrite(pinLED, LOW);
	pinMode(pinCamEn, OUTPUT);
	digitalWrite(pinCamEn, HIGH);
	pinMode(pinServoEn, OUTPUT);
	digitalWrite(pinServoEn, HIGH);

	// Setup host serial
	pinMode(pinU1TX, OUTPUT);
	mapPps(pinU1TX, PPS_OUT_U1TX);
	pinMode(pinU1RX, INPUT);
	mapPps(pinU1RX, PPS_IN_U1RX);
	Serial.begin(9600);

	// Setup camera
	pinMode(pinu2RX, INPUT);
	mapPps(pinu2RX, PPS_IN_U2RX);
	pinMode(pinU2TX, OUTPUT);
	mapPps(pinU2TX, PPS_OUT_U2TX);
	Serial1.begin(9600);
	cam.init();

	// Setup servos
	pinMode(pinPan, OUTPUT);
	pinMode(pinTilt, OUTPUT);
}

void loop(void) {
}
