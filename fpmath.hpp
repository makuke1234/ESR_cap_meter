#pragma once

// Fixed-point math library for Cortex chips without an FPU

#include <cstdint>
#include <string>
#include <cassert>
#include <cstdlib>

#define FPD_FACTOR (1U << 20U)
#define FPD_MASK   (FPD_FACTOR - 1U)

namespace fp
{
	constexpr std::int32_t to(double num) noexcept
	{
		return std::int32_t(double(FPD_FACTOR) * num);
	}
	constexpr std::int32_t to(std::int32_t num) noexcept
	{
		return FPD_FACTOR * num;
	}
	constexpr double fromD(std::int32_t num) noexcept
	{
		return double(num) / double(FPD_FACTOR);
	}
	constexpr std::int32_t fromI(std::int32_t num) noexcept
	{
		return num / FPD_FACTOR;
	}

	constexpr std::int64_t mul(std::int32_t a, std::int32_t b) noexcept
	{
		return (std::int64_t(a) * std::int64_t(b)) / std::int64_t(FPD_FACTOR);
	}
	constexpr std::int32_t div(std::int64_t a, std::int32_t b) noexcept
	{
		return std::int32_t((a * std::int64_t(FPD_FACTOR)) / std::int64_t(b));
	}

	constexpr std::int32_t dec(std::int32_t fpd) noexcept
	{
		return fpd / FPD_FACTOR;
	}
	char * frac(std::int32_t fpd, std::uint8_t accuracy) noexcept;
}
