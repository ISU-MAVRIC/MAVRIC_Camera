#include <SoftPWMServo.h>
#include "SonyFCB.h"

/* Pin Assignments */
const uint8_t pinLED = PIN_LED1;///< LED indicator pin
const uint8_t pinCamEn = 13;    ///< Camera active pin
const uint8_t pinServoEn = 20;  ///< Servo active pin
const uint8_t pinPan = 18;      ///< Pan servo command pin
const uint8_t pinTilt = 19;     ///< Tilt servo command pin
const uint8_t pinU1TX = 11;     ///< Host serial output
const uint8_t pinU1RX = 12;     ///< Host serial input
const uint8_t pinU2TX = 25;     ///< Camera serial output
const uint8_t pinU2RX = 24;     ///< Camera serial input

/* Constant variables */
const uint16_t servoMinTime = 650;    ///< Servo min rotation time to update position, in ms
const uint16_t servoMaxTime = 2350;   ///< Servo max rotation time to update postition, in ms
const uint16_t servoMaxAngle = 3600;  ///< Servo max postion
const uint16_t servoCenter = (servoMaxTime + servoMinTime) / 2; ///< Servo center/default positon

/* Global Objects */
SonyFCB cam = SonyFCB(Serial1, 1);

/* Global Variables */
int16_t panTime = servoCenter;  ///< Pan position, in ms
int16_t tiltTime = servoCenter; ///< Tilt position, in ms
uint8_t buf[16];                ///< Serial buffer
uint8_t bufStatus = false;      ///< Buffer packet received flag
uint8_t bufPos = 0;             ///< Next buffer byte to write to
uint8_t byteRx = 0;             ///< Serial byte received

/**
 *  Initial setup.
 */
void setup() {
  /// Initialize pins
  pinMode(pinLED, OUTPUT);
  digitalWrite(pinLED, LOW);
  pinMode(pinCamEn, OUTPUT);
  digitalWrite(pinCamEn, HIGH);
  pinMode(pinServoEn, OUTPUT);
  digitalWrite(pinServoEn, HIGH);

  /// Setup host serial
  pinMode(pinU1TX, OUTPUT);
  mapPps(pinU1TX, PPS_OUT_U1TX);
  pinMode(pinU1RX, INPUT);
  mapPps(pinU1RX, PPS_IN_U1RX);
  Serial.begin(9600);


  /// Setup camera
  pinMode(pinU2RX, INPUT);
  mapPps(pinU2RX, PPS_IN_U2RX);
  pinMode(pinU2TX, OUTPUT);
  mapPps(pinU2TX, PPS_OUT_U2TX);
  Serial1.begin(9600);
  cam.init();

  /// Setup servos
  pinMode(pinPan, OUTPUT);
  pinMode(pinTilt, OUTPUT);
  SoftPWMServoInit();
  SoftPWMServoSetFrameTime(usToTicks(4000));
  SoftPWMServoSetServoFrames(5);
  
  Serial.println("Ready.");
}

/**
 *  Updates the servo positions relative to their current positions.
 *  @param panChange    ms to rotate the pan servo
 *  @param tiltChange   ms to rotate the tilt servo
 */
void relativeServoUpdate(int16_t panChange, int16_t tiltChange) {
  panTime += panChange;
  tiltTime += tiltChange;
  updateServos();
}

/**
 *  Updates the servo positions to a given positon.
 *  @param panAngle    Angle to set the pan servo postion
 *  @param tiltAngle    Angle to set the tilt servo postion
 */
void absoluteServoUpdate(int16_t panAngle, int16_t tiltAngle) {
  panTime = panAngle + servoCenter;
  tiltTime = tiltAngle + servoCenter;
  updateServos();
}

/**
 *  Sends servos updated position values.
 */
void updateServos() {
  /// Keep servo positions within bounds
  panTime = constrain(panTime, servoMinTime, servoMaxTime);
  tiltTime = constrain(tiltTime, servoMinTime, servoMaxTime);
  /// Send PWM position signals
  SoftPWMServoServoWrite(pinPan, panTime);
  SoftPWMServoServoWrite(pinTilt, tiltTime);
  /// Output command via serial
  Serial.print("<~Camera servo position update~ ");
  int16_t pan = panTime;// - servoCenter;
  int16_t tilt = tiltTime;// - servoCenter;
  int16_t percentTilt = (tilt - servoMinTime) * 100 / (servoMaxTime - servoMinTime);
  int16_t percentPan = (pan - servoMinTime) * 100 / (servoMaxTime - servoMinTime);

  /*
  Serial.write((uint8_t)(pan >> 8));    /// Output MSB of upated pan position
  Serial.write((uint8_t)(pan & 0xff));  /// Output LSB of upated pan position
  Serial.write((uint8_t)(tilt >> 8));   /// Output MSB of upated tilt position
  Serial.write((uint8_t)(tilt & 0xff)); /// Output LSB of upated tilt position
  */
  Serial.print(" Pan:");
  Serial.print(percentPan, DEC);
  Serial.print("%");
  Serial.print("  Tilt:");
  Serial.print(percentTilt, DEC);
  Serial.print("%");
  Serial.println(" >");
}

/**
 *  Parses the bytes received on the host serial and executes the associated command(s).
 *  Camera position update commands are formatted as follows:
 *    [0] - Type of packet being sent (Command)
 *    [1] - Specific system packet applies to (Camera)
 *    [2] - Purpose of the packet (Position)
 *    [3] - Byte 1 of packet parameters (Pan[0])
 *    [4] - Byte 2 of packet parameters (Pan[1])
 *    [5] - Byte 3 of packet parameters (Tilt[0])
 *    [6] - Byte 4 of packet parameters (Tilt[1])
 *  
 *  Servo position is 0 at center, and from range/2 to -(range/2).
 */
void parseCommand() {
  
  if(buf[0] != 'C') return;   /// Byte [0] must signal command packet
  if(buf[1] != 'C') return;   /// Byte [1] must signal camera command
  /// Packet is valid type for camera servo updates, parse command parameters
  if(buf[2] == 'P' || buf[2] == 'p') { /// Position command
    /// Ensure command packet is full length
    if(bufPos < 7) return;
    /// Byte [3,4] is pan value
    int16_t pan = (buf[3] << 8) | buf[4];
    /// Byte [5,6] is tilt value
    int16_t tilt = (buf[5] << 8) | buf[6];
    /// Check type of position update to apply (relative or absolute)
    if(buf[2] == 'P') { /// Absolute positions
      absoluteServoUpdate(pan, tilt);
    } else { /// Relative positions
      relativeServoUpdate(pan, tilt);
    }
  }
  /// Reset receive buffer status and position
  bufPos = 0;
  bufStatus = false;
}

/**
 *  Main loop
 */
void loop() {
  /// Check if serial byte received
  if(Serial.available() > 0) {
    byteRx = Serial.read();
    if(bufStatus) { /// If byte received is the start of a packet
      if(byteRx == '>') {  /// If byte received is the end of a packet
        parseCommand();
      } else {
        buf[bufPos++] = byteRx;  /// Add byte received to buffer, go to next postion in buffer
      }
    } else {  /// If first byte of a packet has not yet been received
      if(byteRx == '<') bufStatus = true;  /// Start of a packet received
    }    
  }
}
