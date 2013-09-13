#ifndef SonyFCB_h
#define SonyFCB_h

#include <WProgram.h>
#include <Stream.h>

class SonyFCB {
	public:
		SonyFCB(Stream &serial, const uint8_t address);

		void init(void);
		void update(void);

		void setAddress(const uint8_t addr);
		void clearBuffers(void);

		void setPower(const bool on);
		bool getPower(void);

		void setZoom(const uint16_t zoom);
		uint16_t getZoom(void);

	private:
		Stream *port;
		uint8_t addr;

		void setRegister(const uint8_t reg, const uint8_t val);
		uint8_t getRegister(const uint8_t reg);
		
		void send(const uint8_t addr, const uint8_t * buf, const uint8_t len);
		uint8_t convertCharacter(const uint8_t in);
};

#endif
