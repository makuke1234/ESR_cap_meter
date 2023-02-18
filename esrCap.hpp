#pragma once

#include "main.hpp"
#include "fpmath.hpp"
#include "portAccess.hpp"
#include "clocks.hpp"

#include <cassert>
#include <cstdint>

namespace esrcap
{
	using sampleFunc_t = std::uint32_t (*)(bool precisemode);
	using gainFunc_t   = void (*)(std::uint8_t gain);

	std::uint32_t autoScaleGetSample(
		sampleFunc_t sampleFunc,
		gainFunc_t gainFunc,
		std::uint8_t & gain,
		bool precisemode = false
	);
}

namespace esr
{

	struct MeterCalData
	{
		static constexpr std::int32_t gainOffset_FPD = fp::muldiv(
			fp::to(ESR_CAP_BURDEN_VOLTAGE),
			std::int32_t(ESR_DETECTOR_RDIV1 * 1000.0),
			std::int32_t( (ESR_DETECTOR_RDIV1 + ESR_DETECTOR_RDIV2) * 1000.0 )
		);
		static constexpr std::int32_t vdiv_FPD = fp::to(1.0);

		std::int32_t adcOffsetVolts_FPD[6] = { 0 };
		std::int32_t outputOffset_FPD = 0;

		esrcap::sampleFunc_t getSample = nullptr;
		esrcap::gainFunc_t   setGain   = nullptr;
		std::uint8_t gain = 1;

		bool precisemode = false;
	};
	extern MeterCalData calData;

	void init(esrcap::sampleFunc_t sampleFunc, esrcap::gainFunc_t gainFunc);

	void setFrequency(std::uint32_t frequency, bool enableOut = false);
	void outputEnable(bool enable = true);

	bool zeroReading();

	std::int32_t measureESR_fpd(bool & overload);
	float measureESR(bool & overload);

	std::int32_t calcVoltsPreDivider_fpd(std::int32_t voltage);
	std::int32_t calcDetectorAmplitude_fpd(std::int32_t preVoltage);
	std::int32_t calcESRDivider_fpd(std::int32_t voltage);
	std::int32_t calcESR_fpd(std::int32_t voltage);
}

namespace cap
{
	struct MeterCalData
	{
		std::uint32_t offsetTicks = CAP_TIME_CONSTANT_OFFSET;
		float capacitanceCoef     = CAP_TIME_CONSTANT_COEF;

		esrcap::sampleFunc_t getSample = nullptr;
		esrcap::gainFunc_t setGain = nullptr;
		std::uint8_t gain = 1;

		std::uint32_t measureTicks = 0;
		bool measureDone = false;
	};
	extern MeterCalData calData;

	void init(esrcap::sampleFunc_t sampleFunc, esrcap::gainFunc_t gainFunc);

	void startMeasureMent_async();
	std::uint32_t measureTicks(bool discharge = true, std::uint32_t timeoutTicks = CAP_TIMEOUT_TICKS);
	bool measureTicks_async(std::uint32_t & ticks, bool discharge = true);
	void stopMeasurement();
	bool isDischarged();
	void discharge();
	void stop();

	void zeroReading();

	// Calculate capacitance in nF
	std::int32_t calcCapacitance_fpd(std::int32_t ticks);
}
