#include "ADC.hpp"

adc::AdcCalData adc::calData;

static void syncadc() noexcept
{
	while (ADC->STATUS.bit.SYNCBUSY);
}

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
	const auto regVal1 = ADC_CAL_LOG_ROW_READ(0), regVal2 = ADC_CAL_LOG_ROW_READ(1);

	this->roomTempValInt = ADC_CAL_EXTRACT(regVal1, ADC_CAL_ROOM_TEMP_VAL_INT);
	this->roomTempValDec = ADC_CAL_EXTRACT(regVal1, ADC_CAL_ROOM_TEMP_VAL_DEC);
	this->hotTempValInt  = ADC_CAL_EXTRACT(regVal1, ADC_CAL_HOT_TEMP_VAL_INT);
	this->hotTempValDec  = ADC_CAL_EXTRACT(regVal1, ADC_CAL_HOT_TEMP_VAL_DEC);
	this->roomInt1VVal   = ADC_CAL_EXTRACT(regVal1, ADC_CAL_ROOM_INT1V_VAL);
	this->hotInt1VVal    = ADC_CAL_EXTRACT(regVal2, ADC_CAL_HOT_INT1V_VAL);
	this->roomAdcVal     = ADC_CAL_EXTRACT(regVal2, ADC_CAL_ROOM_ADC_VAL);
	this->hotAdcVal      = ADC_CAL_EXTRACT(regVal2, ADC_CAL_HOT_ADC_VAL);

	const auto calcref = [](std::int8_t val) noexcept -> float
	{
		return 1.0f - float(val) / 1000.f;
	};

	// Calculate constants
	this->INT1VH = calcref(this->hotInt1VVal);
	this->INT1VR = calcref(this->roomInt1VVal);
	this->tempH  = float(this->hotTempValInt)  + float(this->hotTempValDec)  / 10.0f;
	this->tempR  = float(this->roomTempValInt) + float(this->roomTempValDec) / 10.0f;
	this->VADCH  = float(this->hotAdcVal)  / (this->INT1VH * this->maxCountsFloat);
	this->VADCR  = float(this->roomAdcVal) / (this->INT1VR * this->maxCountsFloat);
}

std::uint8_t adc::init(std::uint8_t adcResolution, std::uint16_t overSamplingSamples) noexcept
{
	std::uint8_t ret = adcResolution;

	// Select ADC reference
	analogReference(AR_INTERNAL1V0);

	uint32_t bias = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
	uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
	linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

	/* Wait for bus synchronization. */
	syncadc();

	/* Write the calibration data. */
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
	syncadc();

	// Configure ADC resolution
	std::uint32_t initialRes;

	switch (adcResolution)
	{
	case 8:
		initialRes = ADC_CTRLB_RESSEL_8BIT;
		break;
	case 10:
		initialRes = ADC_CTRLB_RESSEL_10BIT;
		break;
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
		initialRes = ADC_CTRLB_RESSEL_12BIT;
		break;
	default:
		assert(!"Invalid ADC resolution!");
	}

	ADC->CTRLB.reg = ADC_CLK_DIV | initialRes;	// Select initial ADC resolution & peripheral clock divider
	syncadc();

	if ((adcResolution > 12) || ((adcResolution == 12) && overSamplingSamples))
	{
		// Enable averaging mode
		std::uint32_t sNum;

		if (overSamplingSamples)
		{
			switch (overSamplingSamples)
			{
			case 1:
				sNum = ADC_AVGCTRL_SAMPLENUM_1;
				overSamplingSamples = 0;
				break;
			case 2:
				sNum = ADC_AVGCTRL_SAMPLENUM_2;
				overSamplingSamples = 1;
				break;
			case 4:
				sNum = ADC_AVGCTRL_SAMPLENUM_4;
				overSamplingSamples = 2;
				break;
			case 8:
				sNum = ADC_AVGCTRL_SAMPLENUM_8;
				overSamplingSamples = 3;
				break;
			case 16:
				sNum = ADC_AVGCTRL_SAMPLENUM_16;
				overSamplingSamples = 4;
				break;
			case 32:
				sNum = ADC_AVGCTRL_SAMPLENUM_32;
				overSamplingSamples = 5;
				break;
			case 64:
				sNum = ADC_AVGCTRL_SAMPLENUM_64;
				overSamplingSamples = 6;
				break;
			case 128:
				sNum = ADC_AVGCTRL_SAMPLENUM_128;
				overSamplingSamples = 7;
				break;
			case 256:
				sNum = ADC_AVGCTRL_SAMPLENUM_256;
				overSamplingSamples = 8;
				break;
			case 512:
				sNum = ADC_AVGCTRL_SAMPLENUM_512;
				overSamplingSamples = 9;
				break;
			case 1024:
				sNum = ADC_AVGCTRL_SAMPLENUM_1024;
				overSamplingSamples = 10;
				break;
			default:
				assert(!"Invalid number of oversampling samples!");
			}
		}
		else
		{
			switch (adcResolution)
			{
			case 12:
				sNum = ADC_AVGCTRL_SAMPLENUM_2;
				overSamplingSamples = 1;
				break;
			case 13:
				sNum = ADC_AVGCTRL_SAMPLENUM_4;
				overSamplingSamples = 2;
				break;
			case 14:
				sNum = ADC_AVGCTRL_SAMPLENUM_16;
				overSamplingSamples = 4;
				break;
			case 15:
				sNum = ADC_AVGCTRL_SAMPLENUM_64;
				overSamplingSamples = 6;
				break;
			case 16:
				sNum = ADC_AVGCTRL_SAMPLENUM_256;
				overSamplingSamples = 8;
				break;
			default:
				assert(!"This error is impossible to produce!");
			}
		}

		/*
			Oversampling ADJRES config table for 12 ... 16 bits output

			12 bits:
			0 -> 1 sample   -> 0
			1 -> 2 samples  -> 1
			2 -> 4 samples  -> 2
			3 -> 8 samples  -> 3
			4 -> 16 samples -> 4
			...

			13 bits:
			1 -> (2 samples)-> 0
			2 -> 4 samples  -> 1
			3 -> 8 samples  -> 2
			4 -> 16 samples -> 3
			5 -> 32 samples -> 3
			6 -> 64 samples -> 3
			...
			
			14 bits:
			2 -> (4 samples) -> 0
			3 -> (8 samples) -> 1
			4 -> 16 samples  -> 2
			5 -> 32 samples  -> 2
			6 -> 64 samples  -> 2
			7 -> 128 samples -> 2
			8 -> 256 samples -> 2
			...

			15 bits:
			5  -> (32 samples) -> 0
			6  -> 64  samples  -> 1
			7  -> 128 samples  -> 1
			8  -> 256 samples  -> 1
			9  -> 512 samples  -> 1
			10 -> 1024 samples -> 1

			16 bits:
			8  -> 256 samples  -> 0
			9  -> 512 samples  -> 0
			10 -> 1024 samples -> 0

		*/

		adcResolution -= 12;

		std::int8_t adjRes = adc::clamp<std::int8_t>(overSamplingSamples, 0, 4) - adcResolution;
		adjRes = adc::clamp<std::int8_t>(adjRes, 0, 4);
		// Calculate real achievable resolution
		const auto minOverSample = 2 * adcResolution;
		ret -= adc::clamp<std::int8_t>((minOverSample - overSamplingSamples) / 2, 0, 4);

		adcResolution += 12;


		ADC->AVGCTRL.reg = sNum | ADC_AVGCTRL_ADJRES(adjRes);	// Select averaging amount
		syncadc();
		ADC->CTRLB.reg |= ADC_CTRLB_RESSEL_16BIT;	// Activate averaging mode by selecting 16 bit resolution
		syncadc();
	}

	// Calibrate ADC only if setting up was successful
	if (ret == adcResolution)
	{
		adc::startAdc();
		adc::setSamplingTime(63);
		const auto tempSample = adc::sample(Channel::IntTemp, true);
		adc::setSamplingTime(0);
		adc::calibrate(tempSample, true);
		adc::getSupply(true);
	}

	return ret;
}

void adc::startAdc() noexcept
{
	// Enable ADC peripheral
	PM->APBCMASK.reg |= PM_APBCMASK_ADC;

	// Enable temperature sensor
	SYSCTRL->VREF.bit.TSEN = 1;
	
	// Enable ADC
	ADC->CTRLA.bit.ENABLE = 0x01;
	syncadc();
}
void adc::stopAdc() noexcept
{
	// Disable ADC
	ADC->CTRLA.bit.ENABLE = 0x00;
	syncadc();
	
	// Disable temp sensor
	SYSCTRL->VREF.bit.TSEN = 0;

	// Disable ADC peripheral to save power
	PM->APBCMASK.reg &= ~PM_APBCMASK_ADC;
}

void adc::setGain(Gain gainIdx) noexcept
{
	std::uint8_t gain;
	switch (gainIdx)
	{
	case Gain::g0_5x:
		gain = 0xF;
		break;
	case Gain::g1x:
		gain = 0x0;
		break;
	case Gain::g2x:
		gain = 0x1;
		break;
	case Gain::g4x:
		gain = 0x2;
		break;
	case Gain::g8x:
		gain = 0x3;
		break;
	case Gain::g16x:
		gain = 0x4;
		break;
	default:
		assert(!"Invalid gain setting!");
	}

	adc::calData.gainIdx     = std::uint8_t(gainIdx);
	adc::calData.gainSetting = gain;
}
void adc::setSamplingTime(std::uint8_t time) noexcept
{
	assert(time < 64);

	ADC->SAMPCTRL.reg = time;
	syncadc();
}
static std::uint16_t s_sample() noexcept
{
	ADC->SWTRIG.bit.START = 1;			// Initiate software trigger to start ADC conversion
	syncadc();
	while (!ADC->INTFLAG.bit.RESRDY);	// Wait for conversion
	ADC->INTFLAG.bit.RESRDY = 1;		// Clear interrupt flag
	syncadc();

	return ADC->RESULT.reg;
}
std::uint16_t adc::sample(Channel channel, bool preciseTemp, bool diffMode) noexcept
{
	preciseTemp &= (ADC_CLK_DIV != ADC_CTRLB_PRESCALER_DIV512);
	const auto prescaler = ADC->CTRLB.bit.PRESCALER;

	if (preciseTemp)
	{
		ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV512_Val;
		syncadc();
	}
	if (ADC->CTRLB.bit.DIFFMODE != diffMode)
	{
		ADC->CTRLB.bit.DIFFMODE = diffMode;
		syncadc();
	}

	std::uint8_t chPos, chNeg = 0x19;
	switch (channel)
	{
	case Channel::Out:
		chPos = adc::pinToMux(ESR_OUT);
		break;
	case Channel::OutAmp:
		chPos = adc::pinToMux(ESR_OUT_11X);
		break;
	case Channel::IntTemp:
		chPos = 0x18;
		break;
	case Channel::Cal0:
		chPos = 0x00;
		chNeg = 0x00;
		break;
	case Channel::CalRef:
		chPos = 0x19;
		break;
	case Channel::IOSupply_1_4:
		chPos = 0x1B;
		break;
	case Channel::CoreSupply_1_4:
		chPos = 0x1A;
		chNeg = 0x18;
		break;
	default:
		assert(!"Invalid adc channel!");
	}

	ADC->INPUTCTRL.bit.MUXPOS = chPos;	// Select MUX channel
	syncadc();
	if (diffMode)
	{
		ADC->INPUTCTRL.bit.MUXNEG = (channel == Channel::IntTemp) ? 0x18 : chNeg;	// Select I/O ground as negative input, internal ground for temperature sensing
		syncadc();
	}
	
	// Select proper gain, temperature gain must be 1x
	ADC->INPUTCTRL.bit.GAIN = (channel == Channel::IntTemp) ? 0x00 : adc::calData.gainSetting;
	syncadc();

	if (preciseTemp)
	{
		s_sample();
	}

	const auto result = s_sample();
	if (preciseTemp)
	{
		ADC->CTRLB.bit.PRESCALER = prescaler;
		syncadc();
	}

	return result;
}
float adc::getVolts(std::uint16_t sample) noexcept
{
	return float(ADC_FROMFPD_D(adc::getVolts_fpd(sample)));
}
std::uint32_t adc::getVolts_fpd(std::uint16_t sample) noexcept
{
	float maxCounts = float(ADC_MAX_COUNTS);
	if (adc::Gain(adc::calData.gainIdx) != adc::Gain::g1x)
	{
		maxCounts *= adc::calData.gainCal[adc::calData.gainIdx];
	}

	std::uint32_t volts = ADC_TOFPD( ( adc::calData.ref1VReal * float(sample) ) / maxCounts );

	return volts;
}
void adc::calibrate(std::uint16_t tempSample, bool fullCal) noexcept
{
	if (fullCal)
	{
		// Calibrate zero
		const auto oldGain = adc::calData.gainIdx;

		auto refMax    = adc::sample(Channel::IOSupply_1_4, true);
		adc::setGain(Gain::g0_5x);
		auto refCounts = adc::sample(Channel::IOSupply_1_4, true);

		adc::calData.gainCal[std::uint8_t(Gain::g0_5x)] = float(refCounts) / float(refMax);

		adc::setGain(Gain::g1x);
		refMax    = adc::sample(Channel::CoreSupply_1_4, true);
		adc::setGain(Gain::g2x);
		refCounts = adc::sample(Channel::CoreSupply_1_4, true);
		
		adc::calData.gainCal[std::uint8_t(Gain::g2x)] = float(refCounts) / float(refMax);

		adc::setGain(Gain(oldGain));
	}
	
	// Slightly modified code from Atmel application note AT11481: ADC Configurations with Examples

	float INT1VM;	/* Voltage calculation for reality INT1V value during the ADC conversion */

	const auto & lr = adc::calData.lr;
	const auto coarse_temp = adc::getTemp(tempSample);

	INT1VM = lr.INT1VR + (((lr.INT1VH - lr.INT1VR) * (coarse_temp - lr.tempR)) / (lr.tempH - lr.tempR));
	// Set new reference calibration value
	adc::calData.ref1VReal = INT1VM;
	adc::calData.ref1VReal_FPD = ADC_TOFPD(adc::calData.ref1VReal);
}
float adc::getTemp(std::uint16_t tempSample) noexcept
{
	const auto & lr = adc::calData.lr;
	return lr.tempR + (((lr.tempH - lr.tempR)/(lr.VADCH - lr.VADCR)) * (adc::getVolts(tempSample) - lr.VADCR));
}

float adc::getSupply(bool preciseMeas) noexcept
{
	const auto sample = adc::sample(Channel::IOSupply_1_4, preciseMeas);
	adc::calData.supplyVoltage_FPD = 4U * adc::getVolts_fpd(sample);
	adc::calData.supplyVoltage = ADC_FROMFPD_D(adc::calData.supplyVoltage_FPD);

	return adc::calData.supplyVoltage;
}
