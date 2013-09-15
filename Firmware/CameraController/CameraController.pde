#include <SoftPWMServo.h>
#include "SonyFCB.h"

// Definitions
const uint8_t pinLED = 23;
const uint8_t pinCamEn = 13;
const uint8_t pinServoEn = 20;
const uint8_t pinPan = 18;
const uint8_t pinTilt = 19;
const uint8_t pinU1TX = 11;
const uint8_t pinU1RX = 12;
const uint8_t pinU2TX = 25;
const uint8_t pinu2RX = 24;

const uint16_t servoMinTime = 650;
const uint16_t servoMaxTime = 2350;
const uint16_t servoMaxAngle = 3600;
const uint16_t servoCenter = (servoMaxTime - servoMinTime) / 2;

// Global Objects
SonyFCB cam = SonyFCB(Serial1, 1);

// Global Variables
int16_t panTime = servoCenter;
int16_t tiltTime = servoCenter;
uint8_t bufPos = 0;
uint8_t buf[16];

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
	SoftPWMServoInit();
	SoftPWMServoSetFrameTime(usToTicks(4000));
	SoftPWMServoSetServoFrames(5);
}

void relativeServoUpdate(int16_t panChange, int16_t tiltChange) {
	panTime += panChange;
	tiltTime += tiltChange;
	updateServos();
}

void absoluteServoUpdate(int16_t panAngle, int16_t tiltAngle) {
	panTime = panAngle + servoCenter;
	tiltTime = tiltAngle + servoCenter;
	updateServos();
}

void updateServos(void) {
	constrain(panTime, servoMinTime, servoMaxTime);
	constrain(tiltTime, servoMinTime, servoMaxTime);
	SoftPWMServoServoWrite(pinPan, panTime);
	SoftPWMServoServoWrite(pinTilt, tiltTime);
	Serial.print("<SCP");
	Serial.write((panTime & 0xf0) >> 8);
	Serial.write(panTime & 0x0f);
	Serial.write((tiltTime & 0xf0) >> 8);
	Serial.write(tiltTime & 0x0f);
	Serial.println(">");
}

void loop(void) {
	if(Serial.available()) {
		uint8_t x = Serial.read();
	}
}
