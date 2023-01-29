#pragma once

// C++ includes
#include <cstdio>
#include <cstdarg>
#include <cstdint>

// MCU includes
#include <Arduino.h>

// Constants

// Pin configuration

#define ESR_OUT     A1  // 7
#define ESR_OUT_11X A2  // 8

#define LED_RED    6  // 29
#define LED_GREEN  7  // 30
#define LED_BLUE  20  // 31

#define PUSH_BTN 21  // 32
#define BTN_DEBOUNCE_THRESHOLD_MS 30

#define LCD_RS 4   // 13
#define LCD_EN 1   // 15
#define LCD_D4 17  // 9
#define LCD_D5 18  // 10
#define LCD_D6 8   // 11
#define LCD_D7 9   // 12
#define LCD_RW 3   // 14

#define LCD_WIDTH  16
#define LCD_HEIGHT  2

// Serial baudrate
#define DEFAULT_BAUDRATE 115200

// USB serial printf binding buffer length
#define SERIAL_PRINTF_BUFSIZE 128

// ADC config
#define ADC_RESOLUTION_BITS      16
#define ADC_OVERSAMPLING_SAMPLES  0
#define ADC_REFERENCE            AR_INTERNAL1V0
#define ADC_REFERENCE_VOLTS      1.0
#define ADC_CLK_DIV              ADC_CTRLB_PRESCALER_DIV64


// Function declarations
void btnISR();
void setrgb(std::uint8_t r, std::uint8_t g, std::uint8_t b);
int SerialPrintf(const char * format, ...);
