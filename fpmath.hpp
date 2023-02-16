#pragma once

// Fixed-point math library for Cortex chips without an FPU

#include <cstdint>
#include <string>
#include <cassert>
#include <cstdlib>

#define FPD_FACTOR (1U << 20U)
#define FPD_FRAC   (FPD_FACTOR >> 1U)
#define FPD_MASK   (FPD_FACTOR - 1U)

namespace fp
{
	constexpr std::int32_t to(double num)
	{
		return std::int32_t(double(FPD_FACTOR) * num + 0.5);
	}
	constexpr std::int32_t to(std::int32_t num)
	{
		return FPD_FACTOR * num;
	}
	constexpr double fromD(std::int32_t num)
	{
		return double(num) / double(FPD_FACTOR);
	}
	constexpr std::int32_t fromI(std::int32_t num)
	{
		return (num + FPD_FRAC) / FPD_FACTOR;
	}

	constexpr std::int64_t mul(std::int32_t a, std::int32_t b)
	{
		return (std::int64_t(a) * std::int64_t(b) + std::int64_t(FPD_FRAC)) / std::int64_t(FPD_FACTOR);
	}
	constexpr std::int32_t div(std::int64_t a, std::int32_t b)
	{
		return std::int32_t( (a * std::int64_t(FPD_FACTOR) + (b >> 1)) / std::int64_t(b));
	}
	constexpr std::int32_t muldiv(std::int32_t a, std::int32_t b, std::int32_t c)
	{
		return std::int32_t( (std::int64_t(a) * std::int64_t(b) + (c >> 1)) / std::int64_t(c) );
	}

	constexpr std::int32_t dec(std::int32_t fpd)
	{
		return (fpd + FPD_FRAC) / FPD_FACTOR;
	}
	char * frac(std::int32_t fpd, std::uint8_t accuracy);
}
