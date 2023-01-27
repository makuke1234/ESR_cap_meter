#include "ADC.hpp"

std::uint16_t adc::toCounts(std::uint16_t val, std::uint8_t resolution) noexcept
{
	const auto resDelta = std::int8_t(resolution) - ADC_RESOLUTION_BITS;
	if (!resDelta)
	{
		return val;
	}
	else if (resDelta < 0)
	{
		return (val >> (-resDelta));
	}
	else
	{
		return (val << resDelta);
	}
}
std::uint16_t adc::fromCounts(std::uint16_t val, std::uint8_t resolution) noexcept
{
	return adc::toCounts(val, 2 * ADC_RESOLUTION_BITS - resolution);
}

adc::LogRow::LogRow()
{
	const auto regVal = ADC_CAL_LOG_ROW_READ();

	this->roomTempValInt = ADC_CAL_EXTRACT(regVal, ADC_CAL_ROOM_TEMP_VAL_INT);
	this->roomTempValDec = ADC_CAL_EXTRACT(regVal, ADC_CAL_ROOM_TEMP_VAL_DEC);
	this->hotTempValInt  = ADC_CAL_EXTRACT(regVal, ADC_CAL_HOT_TEMP_VAL_INT);
	this->hotTempValDec  = ADC_CAL_EXTRACT(regVal, ADC_CAL_HOT_TEMP_VAL_DEC);
	this->roomInt1VVal   = ADC_CAL_EXTRACT(regVal, ADC_CAL_ROOM_INT1V_VAL);
	this->hotInt1VVal    = ADC_CAL_EXTRACT(regVal, ADC_CAL_HOT_INT1V_VAL);
	this->roomAdcVal     = ADC_CAL_EXTRACT(regVal, ADC_CAL_ROOM_ADC_VAL);
	this->hotAdcVal      = ADC_CAL_EXTRACT(regVal, ADC_CAL_HOT_ADC_VAL);
}
