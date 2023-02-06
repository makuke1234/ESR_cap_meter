#include "clocks.hpp"

bool clk::initPll96     = false;
bool clk::initTCC0_TCC1 = false,
	 clk::initTCC2_TC3  = false,
	 clk::initTC4_TC5   = false;
std::uint32_t clk::clkTCC0_TCC1{}, clk::clkTCC2_TC3{}, clk::clkTC4_TC5{};

void clk::pll96(std::uint8_t gclkid, std::uint32_t srcFreq)
{
	if (clk::isPll96())
	{
		return;
	}
	
	assert(gclkid  <= 0x08);
	assert(srcFreq <= 48000000U);
	
	const auto divFactor = (96000000U << 4U) / srcFreq;
	
	// Feed src to GCLK_DPLL
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_GEN(std::uint32_t(gclkid)) |
		GCLK_CLKCTRL_ID(1);
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	// Calculate division factor
	// Set DPLL ratio to 8 MHz * (11 + 1) = 96 MHz	
	SYSCTRL->DPLLRATIO.reg =
		SYSCTRL_DPLLRATIO_LDRFRAC(divFactor & 0x0F) |	// Fractional ratio
	    SYSCTRL_DPLLRATIO_LDR((divFactor >> 4U) - 1U);	// Integral ratio

	
	// Configure DPLL to disregard phase lock and select GCLK as source
	SYSCTRL->DPLLCTRLB.reg =
		SYSCTRL_DPLLCTRLB_LBYPASS |	// Bypass lock
		SYSCTRL_DPLLCTRLB_WUF |		// Wake up fast
		SYSCTRL_DPLLCTRLB_REFCLK(SYSCTRL_DPLLCTRLB_REFCLK_GCLK_Val);	// Select GCLK

	// Enable DPLL
	SYSCTRL->DPLLCTRLA.reg |= SYSCTRL_DPLLCTRLA_ENABLE;
	
	clk::initPll96 = true;
}
bool clk::isPll96()
{
	return clk::initPll96;
}

bool clk::isInit(tmr timer)
{
	switch (timer)
	{
	case tmr::tTCC0:
	case tmr::tTCC1:
		return clk::initTCC0_TCC1;
	case tmr::tTCC2:
	case tmr::tTC3:
		return clk::initTCC2_TC3;
	case tmr::tTC4:
	case tmr::tTC5:
		return clk::initTC4_TC5;
	default:
		assert(!"Impossible error to reach!");
	}
}
std::uint32_t clk::tmrSpeed(tmr timer)
{
	switch (timer)
	{
	case tmr::tTCC0:
	case tmr::tTCC1:
		return clk::clkTCC0_TCC1;
	case tmr::tTCC2:
	case tmr::tTC3:
		return clk::clkTCC2_TC3;
	case tmr::tTC4:
	case tmr::tTC5:
		return clk::clkTC4_TC5;
	default:
		assert(!"Impossible error to reach!");
	}
}

void clk::initGCLK(std::uint8_t gclkid, std::uint8_t src, std::uint32_t prescaler, bool expPrescaler)
{
	assert(gclkid <= 0x08);
	assert(src    <= 0x08);
	
	if ( ((prescaler >  1) && !expPrescaler) ||
		 ((prescaler >= 1) &&  expPrescaler) )
	{
		GCLK->GENDIV.reg =
			GCLK_GENDIV_DIV(prescaler - (expPrescaler != 0)) |
			GCLK_GENDIV_ID(std::uint32_t(gclkid));
		while (GCLK->STATUS.bit.SYNCBUSY);
	}
	else
	{
		expPrescaler = false;
	}
	
	GCLK->GENCTRL.reg =
		GCLK_GENCTRL_IDC |
		GCLK_GENCTRL_GENEN |
		GCLK_GENCTRL_SRC(std::uint32_t(src)) |
		(expPrescaler ? GCLK_GENCTRL_DIVSEL : 0UL) |
		GCLK_GENCTRL_ID(std::uint32_t(gclkid));
	while (GCLK->STATUS.bit.SYNCBUSY);
}
void clk::initTmr(tmr timer, std::uint8_t id, std::uint32_t srcFreq)
{
	if (isInit(timer))
	{
		return;
	}
	
	assert(id <= 0x08);
	
	std::uint32_t tmrId;
	
	// Enable timer peripheral
	switch (timer)
	{
	case tmr::tTCC0:
	case tmr::tTCC1:
		tmrId = GCLK_CLKCTRL_ID_TCC0_TCC1;
		PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1;
		break;
	case tmr::tTCC2:
	case tmr::tTC3:
		tmrId = GCLK_CLKCTRL_ID_TCC2_TC3;
		PM->APBCMASK.reg |= PM_APBCMASK_TCC2 | PM_APBCMASK_TC3;
		break;
	case tmr::tTC4:
	case tmr::tTC5:
		tmrId = GCLK_CLKCTRL_ID_TC4_TC5;
		PM->APBCMASK.reg |= PM_APBCMASK_TC4 | PM_APBCMASK_TC5;
		break;
	default:
		assert(!"Impossible error to reach!");
	}
	
	// Enable timer clock
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_GEN(std::uint32_t(id)) |
		tmrId;
	while (GCLK->STATUS.bit.SYNCBUSY);
	
	switch (timer)
	{
	case tmr::tTCC0:
	case tmr::tTCC1:
		clk::initTCC0_TCC1 = true;
		clk::clkTCC0_TCC1  = srcFreq;
		break;
	case tmr::tTCC2:
	case tmr::tTC3:
		clk::initTCC2_TC3  = true;
		clk::clkTCC2_TC3   = srcFreq;
		break;
	case tmr::tTC4:
	case tmr::tTC5:
		clk::initTC4_TC5   = true;
		clk::clkTC4_TC5    = srcFreq;
		break;
	default:
		assert(!"Impossible error to reach!");
	}
}
