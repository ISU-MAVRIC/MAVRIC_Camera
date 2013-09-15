#include "SonyFCB.h"

/******************/
/* Public Methods */
/******************/

SonyFCB::SonyFCB(Stream &serial, const uint8_t address) {
	port = &serial;
	addr = address;
	power = true;
	zoom = 0x0000;
}

/**
 * Initialize the camera.
 */
void SonyFCB::init(void) {
	setAddress(addr);
	clearBuffers();
	setRegister(REG_STABLEZOOM, STABLEZOOM_ON);
}

/**
 * Trigger processing of incoming serial data.
 */
void SonyFCB::update(void) {
	while(port->available()) {
		// TODO parse input
		port->read();
	}
}

/**
 * Set the address of the camera.
 * @param address Address of the camera (1-7)
 */
void SonyFCB::setAddress(const uint8_t address) {
	addr = address;
	uint8_t cmd[] = {0x30, addr};
	send(0x08, cmd, 2);
}

/**
 * Clear the camera's input buffers.
 */
void SonyFCB::clearBuffers(void) {
	uint8_t cmd[] = {0x01, 0x00, 0x01};
	send(0x08, cmd, 3);
}

/**
 * Set the power state of the camera.
 * @param on Power state boolean
 */
void SonyFCB::setPower(const uint8_t on) {
	uint8_t cmd[4];
	this->power = on;
	if(on) {
		cmd = {0x01, 0x04, 0x00, 0x02};
	} else {
		cmd = {0x01, 0x04, 0x00, 0x03};
	}
	send(addr, cmd, 4);
}

/**
 * Get the power state of the camera.
 * @return Power state boolean
 */
uint8_t SonyFCB::getPower(void) {
	return power;
}

/**
 * Set the zoom level of the camera.
 * @param z Zoom level (0x00 wide, 0xFFFF tele)
 */
void SonyFCB::setZoom(const uint16_t z) {
	uint8_t cmd[] = {0x01, 0x04, 0x47, 0x00, 0x00, 0x00, 0x00};
	zoom = z;
	cmd[3] = (z & 0xf000) >> 12;
	cmd[4] = (z & 0x0f00) >> 8;
	cmd[5] = (z & 0x00f0) >> 4;
	cmd[6] = (z & 0x000f);
	send(addr, cmd, 7);
}

/**
 * Get the zoom level of the camera.
 * @return 0x00 wide, 0xFFFF tele
 */
uint16_t SonyFCB::getZoom(void) {
	return zoom;
}

/**
 * Set a register value in the camera.
 * @param reg Register number
 * @param val Register value
 */
void SonyFCB::setRegister(const uint8_t reg, const uint8_t val) {
	uint8_t cmd[] = {0x01, 0x04, 0x24, 0x00, 0x00, 0x00};
	cmd[3] = reg;
	cmd[4] = (val & 0xf0) >> 4;
	cmd[5] = (val & 0x0f);
	send(addr, cmd, 6);
}

/**
 * Get a register value from the camera.
 * @param  reg Register number
 * @return Register value
 */
uint8_t SonyFCB::getRegister(const uint8_t reg) {
	// TODO getRegister
	return 0x00;
}

/*******************/
/* Private Methods */
/*******************/

/**
 * Frame and send a command to the specified camera.
 * @param addr 	Destination address (1-7 or 8 for broadcast)
 * @param buf 	Payload to send to camera
 * @param len	Length of the payload
 */
void SonyFCB::send(const uint8_t addr, const uint8_t * buf, const uint8_t len) {
	// Write header
	port->write(0x80 | addr);
	// Write payload
	port->write(buf, len);
	// Write terminator
	port->write(0xff);
}

/**
 * Convert ascii characters to the proper character codes.
 * @param  in 	ASCII character code input
 * @return	Converted character code
 */
uint8_t SonyFCB::convertCharacter(const uint8_t in) {
	// Default to slashed-zero
	uint8_t out = 72;
	// Handle numerals
	if(in >= 49 && in <= 57) out = in - 19;
	// Handle zero
	if(in == 48) out = 39;
	// Handle upper case
	if(in >= 65 && in <= 90) out = in - 65;
	// Handle lower case
	if(in >= 97 && in <= 122) out = in - 97;
	// TODO handle punctuation marks
	return out;
}
