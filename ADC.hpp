#pragma once

#include "main.hpp"

#include <cstdint>
#include <cassert>

// Macros

#define ADC_CALC_MAX_COUNTS(bits) ((1U << bits) - 1U)
#define ADC_MAX_COUNTS            ADC_CALC_MAX_COUNTS(ADC_RESOLUTION_BITS)
#define ADC_CALC_REFERENCE(gain)  (ADC_REFERENCE_VOLTS / gain)
#define ADC_FPD_FACTOR            131072
constexpr std::int32_t ADC_TOFPD(double num) noexcept
{
	return std::int32_t(double(ADC_FPD_FACTOR) * num);
}
constexpr std::int32_t ADC_TOFPD(std::int32_t num) noexcept
{
	return ADC_FPD_FACTOR * num;
}
constexpr double ADC_FROMFPD_D(std::int32_t num) noexcept
{
	return double(num) / double(ADC_FPD_FACTOR);
}
constexpr std::int32_t ADC_FROMFPD_I(std::int32_t num) noexcept
{
	return num / ADC_FPD_FACTOR;
}

/*
 * ADC pin definitions:
 *
 * AREF -> PA03 -> ADC1
 * A0  -> D14  -> PA02 -> ADC0
 * A1  -> D15  -> PB08 -> ADC2
 * A2  -> D16  -> PB09 -> ADC3
 * A3  -> D17  -> PA04 -> ADC4
 * A4  -> D18  -> PA05 -> ADC5
 * A5  -> D19  -> PB02 -> ADC10
 * A6  -> PA06 -> D8 -> ADC6
 * A7  -> PA07 -> D9 -> ADC7
 * A8  -> D4   -> PA08 -> ADC16
 * A9  -> D3   -> PA09 -> ADC17
 * A10 -> D1   -> PA10 -> ADC18
 * A11 -> D0   -> PA11 -> ADC19
 */

#define A6  8
#define A7  9
#define A8  4
#define A9  3
#define A10 2
#define A11 1

// Temperature
#define ADC_A_TEMP_SLOPE_MV_K    2.4
#define ADC_B_TEMP_SLOPE_MV_K    2.16
#define ADC_A_TEMP_SLOPE_V_K_FPD (ADC_TO_FPD(ADC_A_TEMP_SLOPE_MV_K) / 1000)
#define ADC_B_TEMP_SLOPE_V_K_FPD (ADC_TO_FPD(ADC_B_TEMP_SLOPE_MV_K) / 1000)

#define ADC_A_TEMP_V_25C     0.667
#define ADC_B_TEMP_V_25C     0.688
#define ADC_A_TEMP_V_25C_FPD ADC_TO_FPD(ADC_A_TEMP_V_25C)
#define ADC_B_TEMP_V_25C_FPD ADC_TO_FPD(ADC_B_TEMP_V_25C)

#define ADC_CAL_LOG_ROW_ADDRESS      0x00806030
#define ADC_CAL_LOG_ROW_READ(idx)    (reinterpret_cast<volatile std::uint32_t *>(ADC_CAL_LOG_ROW_ADDRESS)[idx])
#define ADC_CAL_EXTRACT_IMPL_(value, shift, mask) ((shift ? (value >> shift) : value) & mask)
#define ADC_CAL_EXTRACT(value, type) ADC_CAL_EXTRACT_IMPL_(value, type##_SHIFT, type##_MASK)

#define ADC_CAL_ROOM_TEMP_VAL_INT_SHIFT 0
#define ADC_CAL_ROOM_TEMP_VAL_INT_MASK  0x00FF
#define ADC_CAL_ROOM_TEMP_VAL_DEC_SHIFT 8
#define ADC_CAL_ROOM_TEMP_VAL_DEC_MASK  0x000F
#define ADC_CAL_HOT_TEMP_VAL_INT_SHIFT  12
#define ADC_CAL_HOT_TEMP_VAL_INT_MASK   0x00FF
#define ADC_CAL_HOT_TEMP_VAL_DEC_SHIFT  20
#define ADC_CAL_HOT_TEMP_VAL_DEC_MASK   0x000F
#define ADC_CAL_ROOM_INT1V_VAL_SHIFT    24
#define ADC_CAL_ROOM_INT1V_VAL_MASK     0x00FF
#define ADC_CAL_HOT_INT1V_VAL_SHIFT     (32 - 32)
#define ADC_CAL_HOT_INT1V_VAL_MASK      0x00FF
#define ADC_CAL_ROOM_ADC_VAL_SHIFT      (40 - 32)
#define ADC_CAL_ROOM_ADC_VAL_MASK       0x0FFF
#define ADC_CAL_HOT_ADC_VAL_SHIFT       (52 - 32)
#define ADC_CAL_HOT_ADC_VAL_MASK        0x0FFF

namespace adc
{
	template<typename T>
	constexpr T clamp(T v, T min, T max) noexcept
	{
		return (v < min) ? min : ((v > max) ? max : v);
	}

	std::uint16_t toCounts(std::uint16_t val, std::uint8_t resolution) noexcept;
	std::uint16_t fromCounts(std::uint16_t val, std::uint8_t resolution) noexcept;

	struct LogRow
	{
		constexpr static float maxCountsFloat = ADC_CALC_MAX_COUNTS(12);

		std::uint32_t hotAdcVal      : 12;
		std::uint32_t roomAdcVal     : 12;
		std::uint32_t hotInt1VVal    : 8;
		std::uint32_t roomInt1VVal   : 8;
		std::uint32_t hotTempValDec  : 4;
		std::uint32_t hotTempValInt  : 8;
		std::uint32_t roomTempValDec : 4;
		std::uint32_t roomTempValInt : 8;

		float INT1VH;
		float INT1VR;
		float tempH;
		float tempR;
		float VADCH;
		float VADCR;

		LogRow() noexcept;
	};

	enum class Gain : std::uint8_t
	{
		g0_5x,
		g1x,
		g2x,
		g4x,
		g8x,
		g16x
	};
	enum class Channel : std::uint8_t
	{
		Out,
		OutAmp,
		IntTemp
	};

	struct AdcCalData
	{
		adc::LogRow lr;

		float ref1VReal = 1.0f;
		std::int32_t ref1VReal_FPD = ADC_TOFPD(1.0);
		std::int32_t offsetCounts[6] = {};

		std::uint8_t gainIdx = std::uint8_t(Gain::g1x);
		std::uint8_t gainSetting = 0;
		float gainCal[6] = { 0.5f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f };
	};
	extern AdcCalData calData;

	/**
	  * @brief Initializes the ADC with desired resolution
	  * @param adcResolution ADC resolution - 8, 10, or 12 ... 16 bits
	  * @param overSampling Desired oversampling samples, valid for adcResolution >= 12,
	  * 0 by default for automatic oversampling modes
	  * @return Effective real ADC resolution achieved
	  */
	std::uint8_t init(std::uint8_t adcResolution, std::uint16_t overSamplingSamples = 0) noexcept;

	void startAdc() noexcept;
	void stopAdc() noexcept;

	constexpr std::uint8_t pinToMux(std::uint8_t pin) noexcept
	{
		return (pin <= 4) ? ((std::uint8_t[5]){ 19, 18, 0, 17, 16 }[pin]) : ((pin <= 7) ? (pin - 2) : ((std::uint8_t[6]){ 0, 2, 3, 4, 5, 10 }[pin - A0]) );
	}

	void setGain(Gain gainIdx) noexcept;
	/**
	  * @brief Sets the sampling time length for the ADC
	  * @param time Sampling time value in the range of 0 ... 63
	  */
	void setSamplingTime(std::uint8_t time) noexcept;
	std::uint16_t sample(Channel channel, bool preciseTemp = false) noexcept;
	double getVolts(std::uint16_t sample) noexcept;
	std::uint32_t getVolts_fpd(std::uint16_t sample) noexcept;
	void calibrate(std::uint16_t tempSample) noexcept;
	float getTemp(std::uint16_t tempSample) noexcept;
}