#include "main.hpp"


void setup()
{
	SerialUSB.begin(DEFAULT_BAUDRATE);
	delay(10);
	SerialUSB.println("Serial connection initialized.");

	SerialUSB.println("Initialization done!");
}

void loop()
{
	SerialUSB.println("Testing...");
	delay(1000);
}



int SerialPrintf(const char * format, ...)
{
	va_list vp;
	char buf[SERIAL_PRINTF_BUFSIZE];
	buf[SERIAL_PRINTF_BUFSIZE - 1] = '\0';

	va_start(vp, format);
	auto nchars = std::vsnprintf(buf, SERIAL_PRINTF_BUFSIZE, format, vp);
	va_end(vp);

	SerialUSB.print(buf);
	return nchars;
}
