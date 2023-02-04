#include "pwm.hpp"

std::uint8_t pwm::portPinNum[pwm::dutyCycleArrSize];

void pwm::init(std::uint32_t frequency)
{
	// Initialize TCC0 clocks
	PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_GEN_GCLK0 |  // 48 MHz main clock as clock source
		GCLK_CLKCTRL_ID_TCC0_TCC1;
	while (GCLK->STATUS.bit.SYNCBUSY);

	TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
	while (TCC0->SYNCBUSY.bit.WAVE);

	// Set frequency
	const auto ticks = 48000000U / frequency;
	TCC0->PER.reg = ticks;
	while (TCC0->SYNCBUSY.bit.PER);

	// Start timer
	TCC0->CTRLA.bit.ENABLE = 1;
	while (TCC0->SYNCBUSY.bit.ENABLE);
}

void pwm::add(std::uint8_t idx, std::uint8_t pin, std::uint8_t defDuty)
{
	pin = ARDUINO_PIN_TO_PORT_PIN(pin);
	const auto apin = pin;
	const auto grp = PGrp(pin);
	pin &= 0x1F;
	// Enable peripheral multiplexer
	PORT->Group[grp].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
	
	const auto pmuxIdx = pin / 2;

	/* Function E:
	 * PA04, PA05, PA08, PA09
	 */
	/* Function F:
	 * PA10, PA11, PB10, PB11, PA12, PA13, PA14, PA15, PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23
	 */

	// Select function
	auto function = (!grp && (pin <= 9) && (bool[10]){ 0, 0, 0, 0, 1, 1, 0, 0, 1, 1 }[pin]) ? PORT_PMUX_PMUXO_E_Val : PORT_PMUX_PMUXO_F_Val;

	// Even/odd function selection shifting
	function = (pin % 2) ? (function << PORT_PMUX_PMUXO_Pos) : (function << PORT_PMUX_PMUXE_Pos);

	// Set pin function
	PORT->Group[grp].PMUX[pmuxIdx].reg |= function;

	// CC0 -> WO[0], WO[4]
	// CC1 -> WO[1], WO[5]
	// CC2 -> WO[2], WO[6]
	// CC3 -> WO[3], WO[7]
	pwm::portPinNum[idx] = apin;
	pwm::duty(idx, defDuty);
}
void pwm::duty(std::uint8_t idx, std::uint8_t duty)
{
	TCC0->CCB[idx].reg = (TCC0->PER.reg * std::uint32_t(duty)) / 255U;
	while (TCC0->SYNCBUSY.vec.CCB);
}
void pwm::remove(std::uint8_t idx)
{
	const auto pin = pwm::portPinNum[idx];
	// Disable wave output on pin
	PORT->Group[PGrp(pin)].PINCFG[pin & 0x1F].reg &= ~PORT_PINCFG_PMUXEN;
}
