#ifndef SonyFCB_h
#define SonyFCB_h

#include <WProgram.h>
#include <Stream.h>

#define REG_BAUD 0x00
#define BAUD_9600 0x00
#define BAUD_19200 0x01
#define BAUD_38400 0x02
#define REG_STABLEZOOM 0x53
#define STABLEZOOM_OFF 0x00
#define STABLEZOOM_ON 0x01

class SonyFCB {
	public:
		SonyFCB(Stream &serial, const uint8_t address);

		void init(void);
		void update(void);

		void setAddress(const uint8_t addr);
		void clearBuffers(void);

		void setPower(const uint8_t on);
		uint8_t getPower(void);

		void setZoom(const uint16_t zoom);
		uint16_t getZoom(void);

		void setRegister(const uint8_t reg, const uint8_t val);
		uint8_t getRegister(const uint8_t reg);

	private:
		// Configuration values
		Stream *port;
		uint8_t addr;

		// Status values
		uint8_t power;
		uint16_t zoom;
		
		// Helper functions
		void send(const uint8_t addr, const uint8_t * buf, const uint8_t len);
		uint8_t convertCharacter(const uint8_t in);
};

#endif
