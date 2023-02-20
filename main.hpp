#pragma once

// C++ includes
#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cassert>
#include <limits>

// MCU includes
#include <Arduino.h>
#include "ADC.hpp"

// Constants

// Pin configuration

#define CAP_OUT       A11 // 16
#define CAP_RC_DETECT 2   // 23
#define ESR_OUT_11X   A1  // 7

#define ESR_PWM_OUT_LOW      PAPin(13)  // 22
#define ESR_PWM_OUT_HIGH     PAPin(12)  // 21
#define ESR_PWM_FUNCTION     PORT_PMUX_PMUXO_E_Val
#define ESR_PWM_LOW_CHANNEL  0
#define ESR_PWM_HIGH_CHANNEL 1
#define CAP_CHARGE_OUT       PBPin(11)  // PB11, 20
#define CAP_DISCHARGE_OUT    PBPin(10)  // PB10, 19

#define LED_RED    6  // 29
#define LED_GREEN  7  // 30
#define LED_BLUE  20  // 31

#define LED_RED_CC   2
#define LED_GREEN_CC 3
#define LED_BLUE_CC  0

#define LED_PWM_FREQUENCY 10000  // LED PWM frequency

#define PUSH_BTN  21  // 32
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
// Serial data receive buffer length
#define SERIAL_READ_BUFSIZE   32

// ADC config, ADC uses internal 8MHz OSC as clock source
#define ADC_RESOLUTION_BITS      16
#define ADC_OVERSAMPLING_SAMPLES 1024
#define ADC_SLOW_CLK_DIV         ADC_CTRLB_PRESCALER_DIV64  // 125 kHz slow conversion ADC clock
#define ADC_CLK_DIV              ADC_CTRLB_PRESCALER_DIV8   // 1 Mhz normal ADC clock


#define ESR_AMP_GAIN           10.2   // Gain resistors 33k & 3.3k
#define ESR_AMP_OFFSET         0.081  // Amplifier output voltage offset
#define ESR_R_OUT              10.0   // 10 ohms
#define ESR_CAP_BURDEN_VOLTAGE 0.3    // ESR & capacitance measuring burden voltage

#define ESR_DETECTOR_RDIV1 3300.0               // 3.3k
#define ESR_DETECTOR_RDIV2 (47000.0 + 10000.0)  // 47k + 10k

#define ESR_DEFAULT_FREQUENCY  100000  // 100 kHz

// Capacitance meter specific hardware config
#define CAP_R_SERIES             1000.0    // 1k series resistor value
#define CAP_TIME_CONSTANT_COEF   1.0       // Difference to real time constant due to Schmitt triger action
#define CAP_TIME_CONSTANT_OFFSET 10        // Capacitor RC time constant offset in timer ticks, due to delays in the microcontroller & comparator propagation delay
#define CAP_DISCHARGE_THRESHOLD  0.01      // Threshold, under which the capacitor is determined to be completely discharged
#define CAP_TIMEOUT_TICKS        48000000U  // Threshold, over which the measurement will time out

// Data structures
struct State
{
	bool debug:1;
	bool btnState:1;
	bool intOccur:1;

	constexpr State()
		: debug(false), btnState(false), intOccur(false)
	{}
};

// Function declarations
void btnISR();
void setrgb(std::uint8_t r, std::uint8_t g, std::uint8_t b);
int SerialPrintf(const char * format, ...);

std::uint32_t getSampleInterfaceEsr(bool precisemode);
std::uint32_t getSampleInterfaceCap(bool precisemode);
void setGainInterface(std::uint8_t gain);

bool comRead(char * buf, std::uint8_t & bufidx);
void communicationLoop();

// Printing function
enum class Info : std::uint8_t
{
	Temp,
	TempAcc,
	Ref,
	Supply,
	Gain_0x5,
	Gain_2x
};

void printInfo(Info type, std::uint8_t decPlaces = 0, bool hasData = false, float uData = 0.0f);
