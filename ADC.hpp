#pragma once

#include "main.hpp"

#include <cstdint>

// Macros

#define ADC_MAX_COUNTS           ((1U << ADC_RESOLUTION_BITS) - 1U)
#define ADC_CALC_REFERENCE(gain) (ADC_REFERENCE_VOLTS / gain)
#define ADC_FPD_FACTOR           1000000
#define ADC_TOFPD(num)           std::int32_t(double(ADC_FPD_FACTOR) * double(num))
#define ADC_FROMFPD(num)         double(num) / double(ADC_FPD_FACTOR)

// Temperature
#define ADC_A_TEMP_SLOPE_MV_K 2.4
#define ADC_B_TEMP_SLOPE_MV_K 2.16
#define ADC_A_TEMP_SLOPE_V_K_FPD (ADC_TO_FPD(ADC_A_TEMP_SLOPE_MV_K) / 1000)
#define ADC_B_TEMP_SLOPE_V_K_FPD (ADC_TO_FPD(ADC_B_TEMP_SLOPE_MV_K) / 1000)

#define ADC_A_TEMP_V_25C 0.667
#define ADC_B_TEMP_V_25C 0.688
#define ADC_A_TEMP_V_25C_FPD ADC_TO_FPD(ADC_A_TEMP_V_25C)
#define ADC_B_TEMP_V_25C_FPD ADC_TO_FPD(ADC_B_TEMP_V_25C)

#define ADC_CAL_LOG_ROW_ADDRESS 0x00806030
#define ADC_CAL_LOG_ROW_READ() (*reinterpret_cast<volatile std::uint64_t *>(ADC_CAL_LOG_ROW_ADDRESS))
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
#define ADC_CAL_HOT_INT1V_VAL_SHIFT     32
#define ADC_CAL_HOT_INT1V_VAL_MASK      0x00FF
#define ADC_CAL_ROOM_ADC_VAL_SHIFT      40
#define ADC_CAL_ROOM_ADC_VAL_MASK       0x0FFF
#define ADC_CAL_HOT_ADC_VAL_SHIFT       52
#define ADC_CAL_HOT_ADC_VAL_MASK        0x0FFF

namespace adc
{
	struct LogRow
	{
		std::uint64_t hotAdcVal     :12;
		std::uint64_t roomAdcVal    :12;
		std::uint64_t hotInt1VVal   :8;
		std::uint64_t roomInt1VVal  :8;
		std::uint64_t hotTempValDec :4;
		std::uint64_t hotTempValInt :8;
		std::uint64_t roomTempValDec:4;
		std::uint64_t roomTempValInt:8;

		LogRow() noexcept;
	};
}
