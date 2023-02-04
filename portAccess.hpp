#pragma once

#include <Arduino.h>
#include <variant.h>

#define ARDUINO_PIN_TO_PORT_PIN(x) \
	((int[22]){ \
		11, 10, 14, 9, 8, 15, 20, 21, 6, 7, 18, 16, 19, 17, 2, 32 | 8, 32 | 9, 4, 5, 32 | 2, 22, 23 \
	}[x])

#define ARDUINO_PIN_TO_EIC(x) \
	((int[22]){ \
		11, 10, 14, 9, -1, 15, 4, 5, 6, 7, 2, 0, 3, 1, 2, 8, 9, 4, 5, 2, 6, 7 \
	}[x])

#define PBMask(_x) (1 << ((_x) & 0x1f))
#define PGrp(_x)   ((_x) >> 5)

#define OutMode(x) (PORT->Group[PGrp(x)].DIRSET.reg = PBMask(x))

#define OutSet(_x) (PORT_IOBUS->Group[PGrp(_x)].OUTSET.reg = PBMask(_x))
#define OutClr(_x) (PORT_IOBUS->Group[PGrp(_x)].OUTCLR.reg = PBMask(_x))
#define OutTgl(_x) (PORT_IOBUS->Group[PGrp(_x)].OUTTGL.reg = PBMask(_x))

#define OutPin(_x, _val) if (_val) OutSet(_x); else OutClr(_x)

#define disableInterrupt_impl(x) EIC->INTFLAG.reg |= (1 << (x));
#define disableInterrupt(pin) disableInterrupt_impl(ARDUINO_PIN_TO_EIC(pin))