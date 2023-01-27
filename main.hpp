#pragma once

// C++ includes
#include <cstdio>
#include <cstdarg>

// MCU includes
#include <Arduino.h>

// Constants

// Serial baudrate
#define DEFAULT_BAUDRATE 115200

// USB serial printf binding buffer length
#define SERIAL_PRINTF_BUFSIZE 128

// ADC config
#define ADC_RESOLUTION_BITS 12
#define ADC_REFERENCE       AR_INTERNAL1V0
#define ADC_REFERENCE_VOLTS 1.0


// Function declarations

int SerialPrintf(const char * format, ...);