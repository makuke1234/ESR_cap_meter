#pragma once

#include "main.hpp"

namespace esr
{
	struct MeterCalData
	{

	};
	extern MeterCalData calData;

	void zeroReading();

	float calcVoltsPre(float voltage);
	float calcDetectorAmplitude(float preVoltage);
	float calcESRDivider(float voltage);
	float calcESR(float voltage);
}

namespace cap
{
	struct MeterCalData
	{

	};
	extern MeterCalData calData;

}
