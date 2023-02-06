#pragma once

#include <cstdint>
#include <cassert>

#include <Arduino.h>

#define GCLK_GENCTRL_SRC_DPLL96M_Val 0x8ul
#define GCLK_GENCTRL_SRC_DPLL96M (GCLK_GENCTRL_SRC_DPLL96M_Val << GCLK_GENCTRL_SRC_Pos)

namespace clk
{
	extern bool initPll96;
	extern bool initTCC0_TCC1, initTCC2_TC3, initTC4_TC5;
	extern std::uint32_t clkTCC0_TCC1, clkTCC2_TC3, clkTC4_TC5;
	
	void pll96(std::uint8_t gclkid, std::uint32_t srcFreq);
	bool isPll96();
	
	enum class tmr : std::uint8_t
	{
		tTCC0,
		tTCC1,
		tTCC2,
		tTC3,
		tTC4,
		tTC5
	};
	bool isInit(tmr timer);
	std::uint32_t tmrSpeed(tmr timer);
	
	void initGCLK(std::uint8_t gclkid, std::uint8_t src, std::uint32_t prescaler = 1, bool expPrescaler = false);
	void initTmr(tmr timer, std::uint8_t id, std::uint32_t srcFreq);
}