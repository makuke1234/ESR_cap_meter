#pragma once

#include "portAccess.hpp"

#include <vector>
#include <cstdint>

namespace pwm
{
	constexpr std::uint8_t dutyCycleArrSize = 4;

	extern std::uint8_t portPinNum[dutyCycleArrSize];

	void init(std::uint32_t frequency);

	void add(std::uint8_t idx, std::uint8_t pin, std::uint8_t defDuty = 0);
	void duty(std::uint8_t idx, std::uint8_t duty);
	void remove(std::uint8_t idx);
}
