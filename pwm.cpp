#include "pwm.hpp"

volatile bool pwm::enabled[pwm::dutyCycleArrSize] = {};
std::uint8_t pwm::portPinNum[pwm::dutyCycleArrSize];

void TCC0_Handler()
{
	if (TCC0->INTFLAG.bit.OVF)
	{
		if (pwm::enabled[0]) OutSet(pwm::portPinNum[0]);
		if (pwm::enabled[1]) OutSet(pwm::portPinNum[1]);
		if (pwm::enabled[2]) OutSet(pwm::portPinNum[2]);
		if (pwm::enabled[3]) OutSet(pwm::portPinNum[3]);
		TCC0->INTFLAG.bit.OVF = 1;
	}
	else if (TCC0->INTFLAG.bit.MC0)
	{
		OutClr(pwm::portPinNum[0]);
		TCC0->INTFLAG.bit.MC0 = 1;
	}
	else if (TCC0->INTFLAG.bit.MC1)
	{
		OutClr(pwm::portPinNum[1]);
		TCC0->INTFLAG.bit.MC1 = 1;
	}
	else if (TCC0->INTFLAG.bit.MC2)
	{
		OutClr(pwm::portPinNum[2]);
		TCC0->INTFLAG.bit.MC2 = 1;
	}
	else if (TCC0->INTFLAG.bit.MC3)
	{
		OutClr(pwm::portPinNum[3]);
		TCC0->INTFLAG.bit.MC3 = 1;
	}
}

void pwm::init(std::uint32_t frequency)
{
	// Initialize TCC0 clocks
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_GEN_GCLK0 |  // 48 MHz main clock as clock source
		GCLK_CLKCTRL_ID_TCC0_TCC1;
	while (GCLK->STATUS.bit.SYNCBUSY);

	TCC0->INTENSET.reg = TCC_INTENSET_OVF;

	TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ;
	while (TCC0->SYNCBUSY.bit.WAVE);

	// Set the timer priority
	NVIC_SetPriority(TCC0_IRQn, 1);
	// Enable Timer0 interrupts
	NVIC_EnableIRQ(TCC0_IRQn);

	// Set frequency
	const auto ticks = 48000000U / frequency;
	TCC0->PER.reg = ticks - 1;
	while (TCC0->SYNCBUSY.bit.PER);
	
	TCC0->COUNT.reg = 0;
	while (TCC0->SYNCBUSY.bit.COUNT);

	// Start timer
	TCC0->CTRLA.bit.ENABLE = 1;
	while (TCC0->SYNCBUSY.bit.ENABLE);
}

void pwm::add(std::uint8_t idx, std::uint8_t pin, std::uint8_t defDuty)
{
	pwm::portPinNum[idx] = ARDUINO_PIN_TO_PORT_PIN(pin);
	pwm::duty(idx, defDuty);

	switch (idx)
	{
	case 0:
		TCC0->INTENSET.bit.MC0 = 1;
		break;
	case 1:
		TCC0->INTENSET.bit.MC1 = 1;
		break;
	case 2:
		TCC0->INTENSET.bit.MC2 = 1;
		break;
	case 3:
		TCC0->INTENSET.bit.MC3 = 1;
		break;
	}
	pwm::enabled[idx] = defDuty != 0;
}
void pwm::duty(std::uint8_t idx, std::uint8_t duty)
{
	TCC0->CC[idx].reg = (TCC0->PER.reg * std::uint32_t(duty)) / 255U;
	while (TCC0->SYNCBUSY.reg);

	pwm::enabled[idx] = duty != 0;
}
void pwm::remove(std::uint8_t idx)
{
	pwm::enabled[idx] = 0;
	switch (idx)
	{
	case 0:
		TCC0->INTENSET.bit.MC0 = 0;
		break;
	case 1:
		TCC0->INTENSET.bit.MC1 = 0;
		break;
	case 2:
		TCC0->INTENSET.bit.MC2 = 0;
		break;
	case 3:
		TCC0->INTENSET.bit.MC3 = 0;
		break;
	}
}
