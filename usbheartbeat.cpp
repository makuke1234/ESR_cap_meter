#include "usbheartbeat.hpp"

static bool s_isConnected = false;

void TC3_Handler()
{
	static std::uint32_t oldCount = 0;

	TC3->COUNT16.COUNT.reg = 0;

	const auto count = USB->DEVICE.FNUM.bit.FNUM;
	//const auto count = 0;
	s_isConnected = (count != oldCount);
	oldCount = count;

	// Clear overflow interrupt flag
	TC3->COUNT16.INTFLAG.bit.MC0 = 1;
}

void heartbeat::init()
{
	// Initialize timer TC3
	std::uint32_t tmrFreq = 96000000U;
	if (!clk::isInit(clk::tmr::tTC3))
	{
		clk::initTmr(clk::tmr::tTC3, GCLK_CLKCTRL_GEN_GCLK4_Val, tmrFreq);
	}
	else
	{
		tmrFreq = clk::tmrSpeed(clk::tmr::tTC3);
	}

	// Set up TC3 prescaler according to frequency, try to get frequency as close to 10000 Hz as possible
	// Also set TC3 into 16 bit mode
	std::uint32_t prescaler = tmrFreq / 10000U;
	auto prescaler_reg = 1;
	{
		auto temppre = prescaler;
		while (temppre >>= 1)
		{
			++prescaler_reg;
		}
	}
	prescaler_reg = (prescaler_reg > 4U) ? ((prescaler_reg - 4U) / 2U + 4U) : prescaler_reg;
	prescaler_reg = (prescaler_reg > 7U) ? 7U : prescaler_reg;
	prescaler = 1U << ((prescaler_reg > 4U) ? (prescaler_reg - 4U) * 2 + 4U : prescaler_reg);


	TC3->COUNT16.CTRLA.reg |=
		TC_CTRLA_PRESCALER(prescaler_reg) |
		TC_CTRLA_MODE_COUNT16;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

	// Set up TC3 period register according to frequency, get it to fire every ~5 milliseconds
	std::uint32_t period = tmrFreq / (prescaler * 200U);
	period = (period > 65535U) ? 65535U : period;

	TC3->COUNT16.CC[0].reg = std::uint16_t(period);
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

	// Enable TC3 interrupts
	NVIC_SetPriority(TC3_IRQn, 3);
	NVIC_EnableIRQ(TC3_IRQn);
	TC3->COUNT16.INTENSET.bit.MC0 = 1;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

	// Enable TC3
	TC3->COUNT16.CTRLA.bit.ENABLE = 1;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

	// Wait to get at least some pulses
	delay(7);
}
bool heartbeat::isConnected()
{
	return s_isConnected;
}
