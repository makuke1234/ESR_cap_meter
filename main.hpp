#pragma once

// C++ includes
#include <cstdio>
#include <cstdarg>

// MCU includes
#include <Arduino.h>

// Constants

#define DEFAULT_BAUDRATE 115200

#define SERIAL_PRINTF_BUFSIZE 128


// Function declarations

int SerialPrintf(const char * format, ...);