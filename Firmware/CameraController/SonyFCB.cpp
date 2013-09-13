#include "SonyFCB.h"

SonyFCB::SonyFCB(Stream &serial, const uint8_t address) {
	port = &serial;
	addr = address;
}

void SonyFCB::init(void) {
	setAddress(addr);
	clearBuffers();
}

void SonyFCB::update(void) {
	while(port->available()) {
		// TODO parse input
		port->read();
	}
}

void SonyFCB::setAddress(const uint8_t address) {
	addr = address;
	uint8_t cmd[] = {0x30, addr};
	send(0x08, cmd, 2);
}

void SonyFCB::clearBuffers(void) {
	uint8_t cmd[] = {0x01, 0x00, 0x01};
	send(0x08, cmd, 3);
}

void SonyFCB::send(const uint8_t addr, const uint8_t * buf, const uint8_t len) {
	// Write header
	port->write(0x80 | addr);
	// Write payload
	port->write(buf, len);
	// Write terminator
	port->write(0xff);
}

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
