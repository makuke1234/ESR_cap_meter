#include "esrCap.hpp"

esr::MeterCalData esr::calData;
cap::MeterCalData cap::calData;

std::uint32_t esrcap::autoScaleGetSample(
	esrcap::sampleFunc_t sampleFunc,
	esrcap::gainFunc_t gainFunc,
	std::uint8_t & oldgain,
	bool precisemode
) noexcept
{
	auto sample = sampleFunc(precisemode);

	std::int32_t gain = fp::div(fp::to(1.0), sample);
	gain -= fp::to(0.2);

	if (gain < fp::to(1.0))
	{
		// Set gain to 0.5
		gain = 0;
	}
	else
	{
		gain = fp::fromI(gain);
		// Crude logarithmic conversion
		std::int32_t bits = 0;
		while (gain >>= 1)
		{
			++bits;
		}
		// clamp to 5
		gain = (bits > 5) ? 5 : bits;
	}

	if (gain != oldgain)
	{
		oldgain = gain;
		gainFunc(oldgain);
		sample = sampleFunc(precisemode);
	}

	return sample;
}

void TC3_Handler()
{
	// Toggle the outputs
	if (TC3->COUNT16.INTFLAG.bit.MC0)
	{
		OutClr(ESR_PWM_OUT_LOW);
		OutSet(ESR_PWM_OUT_HIGH);
		TC3->COUNT16.INTFLAG.bit.MC0 = 1;
	}
	else if (TC3->COUNT16.INTFLAG.bit.MC1)
	{
		OutClr(ESR_PWM_OUT_HIGH);
		OutSet(ESR_PWM_OUT_LOW);
		TC3->COUNT16.COUNT.reg = 0;
		TC3->COUNT16.INTFLAG.bit.MC1 = 1;
	}
}

void esr::init(esrcap::sampleFunc_t sampleFunc, esrcap::gainFunc_t gainFunc) noexcept
{
	assert(sampleFunc != nullptr);
	assert(gainFunc != nullptr);

	esr::calData.getSample = sampleFunc;
	esr::calData.setGain   = gainFunc;

	pinMode(ESR_OUT_11X, INPUT);
	OutMode(ESR_PWM_OUT_LOW);
	OutMode(ESR_PWM_OUT_HIGH);

	// Initialize timer clocks
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_ID_TCC2_TC3 |
		GCLK_CLKCTRL_GEN_GCLK0 |
		GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY);	

	PM->APBCSEL.bit.APBCDIV = 0;  // No prescaler
	PM->APBCMASK.bit.TC3_   = 1;  // Enable TC3 interface

	TC3->COUNT16.CTRLA.reg =
		TC_CTRLA_PRESCALER_DIV1 |
		TC_CTRLA_MODE_COUNT16;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

	// Enable match interrupts
	TC3->COUNT16.INTENSET.bit.MC0 = 1;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
	TC3->COUNT16.INTENSET.bit.MC1 = 1;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

	// Set the timer to be the most prioritized thing running
	NVIC_SetPriority(TC3_IRQn, 0);
	// Enable Timer3 interrupts
	NVIC_EnableIRQ(TC3_IRQn);

	esr::setFrequency(ESR_DEFAULT_FREQUENCY);
}

void esr::setFrequency(std::uint32_t frequency) noexcept
{
	// Calculate ticks from frequency
	const auto ticks = 48000000U / frequency;

	// Change timer tickcount
	TC3->COUNT16.CC[1].reg = ticks;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
	TC3->COUNT16.CC[0].reg = ticks/2;
	while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
	TC3->COUNT16.COUNT.reg = 0;
}
void esr::outputEnable(bool enable) noexcept
{
	if (!enable)
	{
		// Disable timer
		TC3->COUNT16.CTRLA.bit.ENABLE = 0;
		while (TC3->COUNT16.STATUS.bit.SYNCBUSY);

		// set outputs to 0
		OutClr(ESR_PWM_OUT_LOW);
		OutClr(ESR_PWM_OUT_HIGH);
	}
	else
	{
		// Enable timer
		TC3->COUNT16.CTRLA.bit.ENABLE = 1;
		while (TC3->COUNT16.STATUS.bit.SYNCBUSY);
	}
}

bool esr::zeroReading() noexcept
{
	bool overload;
	esr::calData.outputOffset_FPD += esr::measureESR_fpd(overload);
	return !overload;
}

std::int32_t esr::measureESR_fpd(bool & overload) noexcept
{
	auto sample = esrcap::autoScaleGetSample(
		esr::calData.getSample,
		esr::calData.setGain,
		esr::calData.gain
	);
	auto offset = esr::calData.adcOffsetVolts_FPD[esr::calData.gain];
	sample -= offset;

	auto gain = esr::calData.gain;

	auto olvolts = (!gain) ? (sample >> 1) : ( (gain == 1) ? sample : (sample << (gain - 1)) );
	overload = (olvolts > fp::to(0.85));
	if (overload)
	{
		return INT32_MAX;
	}


	// Remove amplifier offset to the input voltage
	sample -= fp::to(ESR_AMP_OFFSET);

	return esr::calcESR_fpd(sample);
}
float esr::measureESR(bool & overload) noexcept
{
	return fp::fromD(esr::measureESR_fpd(overload));
}

std::int32_t esr::calcVoltsPreDivider_fpd(std::int32_t voltage) noexcept
{
	return fp::div(voltage, esr::calData.vdiv_FPD);
}
std::int32_t esr::calcDetectorAmplitude_fpd(std::int32_t preVoltage) noexcept
{
	std::int32_t ampl = esr::calcVoltsPreDivider_fpd(preVoltage);
	ampl = fp::div(ampl, fp::to(ESR_AMP_GAIN));
	ampl -= esr::calData.gainOffset_FPD;

	return 2 * ampl;
}
std::int32_t esr::calcESRDivider_fpd(std::int32_t voltage) noexcept
{
	std::int32_t ampl = esr::calcDetectorAmplitude_fpd(voltage);
	return (ampl) ? fp::div(fp::to(ESR_CAP_BURDEN_VOLTAGE), ampl) : INT32_MAX;
}
std::int32_t esr::calcESR_fpd(std::int32_t voltage) noexcept
{
	std::int32_t r = fp::div(fp::to(ESR_R_OUT), esr::calcESRDivider_fpd(voltage) - fp::to(1.0));
	r -= esr::calData.outputOffset_FPD;

	return r;
}


void cap::init(esrcap::sampleFunc_t sampleFunc, esrcap::gainFunc_t gainFunc) noexcept
{
	assert(sampleFunc != nullptr);
	assert(gainFunc != nullptr);

	cap::calData.getSample = sampleFunc;
	cap::calData.setGain   = gainFunc;

	pinMode(CAP_RC_DETECT, INPUT);
	pinMode(CAP_CHARGE_OUT, OUTPUT);
	pinMode(CAP_DISCHARGE_OUT, OUTPUT);

	// Configure timer clock source

	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_CLKEN |
		GCLK_CLKCTRL_GEN_GCLK0 |  // 48 MHz main clock as clock source
		GCLK_CLKCTRL_ID_TC4_TC5;
	while (GCLK->STATUS.bit.SYNCBUSY);

	TC4->COUNT32.CTRLA.reg |=
		TC_CTRLA_PRESCALER_DIV1 |
		TC_CTRLA_MODE_COUNT32;
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
}

static void capMeasurementISR() noexcept
{
	// Capture timer ticks
	const auto ticks = TC4->COUNT32.COUNT.reg;

	cap::stopMeasurement();

	cap::calData.measureTicks = ticks;
	// Mark as done
	cap::calData.measureDone = true;
	
	SerialUSB.println("ISR!!!");
}
void cap::startMeasureMent_async() noexcept
{
	// Disable discharge
	//digitalWrite(CAP_DISCHARGE_OUT, 0);

	cap::calData.measureDone = false;

	// Enable RC time constant interrupt
	attachInterrupt(digitalPinToInterrupt(CAP_RC_DETECT), &capMeasurementISR, FALLING);

	// Start timer
	TC4->COUNT32.COUNT.reg = 0;
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);

	TC4->COUNT32.READREQ.reg = TC_READREQ_RCONT | TC_READREQ_ADDR(0x10);
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);	

	TC4->COUNT32.CTRLA.bit.ENABLE = 1;
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);

	// Start charging
	//digitalWrite(CAP_CHARGE_OUT, 1);
}
std::uint32_t cap::measureTicks(bool discharge, std::uint32_t timeoutTicks) noexcept
{
	std::uint32_t ticks;

	cap::startMeasureMent_async();
	while (!cap::measureTicks_async(ticks, discharge))
	{
		if (TC4->COUNT32.COUNT.reg > timeoutTicks)
		{
			cap::stopMeasurement();
			return UINT32_MAX;
		}
	}

	return ticks;
}
bool cap::measureTicks_async(std::uint32_t & ticks, bool discharge) noexcept
{
	auto ret = cap::calData.measureDone;
	if (ret)
	{
		ticks = cap::calData.measureTicks;
	}

	if (ret && discharge)
	{
		cap::discharge();		
	}

	cap::calData.measureDone = false;

	return ret;
}
void cap::stopMeasurement() noexcept
{
	// Disable timer
	TC4->COUNT32.CTRLA.bit.ENABLE = 0;
	while (TC4->COUNT32.STATUS.bit.SYNCBUSY);
	// Disable RC interrupt
	disableInterrupt(digitalPinToInterrupt(CAP_RC_DETECT));
}
bool cap::isDischarged() noexcept
{
	const auto sample = esrcap::autoScaleGetSample(
		cap::calData.getSample,
		cap::calData.setGain,
		cap::calData.gain,
		true
	);
	return sample < fp::to(CAP_DISCHARGE_THRESHOLD);
}
void cap::discharge() noexcept
{
	digitalWrite(CAP_CHARGE_OUT,    0);
	digitalWrite(CAP_DISCHARGE_OUT, 1);
}

std::int32_t cap::calcCapacitance_fpd(std::int32_t ticks) noexcept
{
	ticks -= CAP_TIME_CONSTANT_OFFSET;
	ticks = fp::mul(ticks, fp::to(CAP_TIME_CONSTANT_COEF));

	// C = (ticks * 1e9) / (R * ticks_in_second)
	std::int32_t cap = fp::muldiv(ticks, 1000000000, std::int32_t(CAP_R_SERIES * 48000000U));
	return cap;
}
