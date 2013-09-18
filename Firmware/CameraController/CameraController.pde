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
uint8_t bufStatus = false;
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
	int16_t pan = panTime - servoCenter;
	int16_t tilt = tiltTime - servoCenter;
	Serial.write((uint8_t)(pan >> 8));
	Serial.write((uint8_t)(pan & 0xff));
	Serial.write((uint8_t)(tilt >> 8));
	Serial.write((uint8_t)(tilt & 0xff));
	Serial.println(">");
}

void parseCommand(void) {
	// Must be command packet
	if(buf[0] != 'C') return;
	// Must be camera command
	if(buf[1] != 'C') return;
	// Parse command
	if(buf[2] == 'P' || buf[2] == 'p') { // Position command
		// Ensure packet is long enough
		if(bufPos < 7) return;
		// Get pan value
		int16_t pan = (buf[3] << 8) | buf[4];
		// Get tilt value
		int16_t tilt = (buf[5] << 8) | buf[6];
		if(buf[2] == 'P') { // Absolute
			absoluteServoUpdate(pan, tilt);
		} else { // Relative
			relativeServoUpdate(pan, tilt);
		}
	}
	bufPos = 0;
	bufStatus = false;
}

void loop(void) {
	if(Serial.available()) {
		uint8_t x = Serial.read();
		if(bufStatus) {
			if(x == '>') {
				parseCommand();
			} else {
				buf[bufPos++] = x;
			}
		} else {
			if(x == '<') bufStatus = true;
		}
	}
}
