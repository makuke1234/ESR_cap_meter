#include "main.hpp"
#include "ADC.hpp"
#include "display.hpp"
#include "esrCap.hpp"
#include "pwm.hpp"
#include "usbheartbeat.hpp"

static volatile State state;

void setup()
{
	clk::pll96(GCLK_CLKCTRL_GEN_GCLK3_Val, 8000000U);
	clk::initGCLK(GCLK_CLKCTRL_GEN_GCLK4_Val, GCLK_GENCTRL_SRC_DPLL96M_Val);

	heartbeat::init();

	pinMode(LED_RED,   OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE,  OUTPUT);

	pwm::init(LED_PWM_FREQUENCY);
	pwm::add(LED_RED_CC,   LED_RED);
	pwm::add(LED_GREEN_CC, LED_GREEN);
	pwm::add(LED_BLUE_CC,  LED_BLUE);

	pinMode(PUSH_BTN, INPUT);
	// Attach interrupt to the button
	attachInterrupt(PUSH_BTN, &btnISR, CHANGE);
	
	SerialUSB.begin(DEFAULT_BAUDRATE);
	bool connected = false;
	if (!digitalRead(PUSH_BTN))
	{
		const auto sm = millis() + 500;
		while (!heartbeat::isConnected() && (millis() < sm));
	}
	if (heartbeat::isConnected())
	{
		setrgb(0, 25, 127);
		const auto sm = millis() + 5000;
		while (!SerialUSB && (millis() < sm));
		connected = SerialUSB != 0;
	}

	if (connected)
	{
		SerialUSB.println("Serial connection initialized.");
		setrgb(0, 255, 10);
	}
	else
	{
		setrgb(255, 25, 0);
	}


	SerialUSB.print("Initializing display...");
	disp::init();
	SerialUSB.println(" OK");


	SerialUSB.print("Initializing ADC...");
	const auto realres = adc::init(ADC_RESOLUTION_BITS, ADC_OVERSAMPLING_SAMPLES);
	if (realres == ADC_RESOLUTION_BITS)
	{
		SerialUSB.println(" OK");
	}
	else
	{
		SerialUSB.print(" Fail! Expected resolution: ");
		SerialUSB.print(ADC_RESOLUTION_BITS);
		SerialUSB.print("; Got: ");
		SerialUSB.println(realres);

		switch (realres)
		{
		case 8:
			setrgb(255, 0, 0);
			break;
		case 10:
			setrgb(255, 0, 255);
			break;
		case 12:
			setrgb(255, 255, 0);
			break;
		case 13:
			setrgb(0, 255, 255);
			break;
		case 14:
			setrgb(0, 0, 255);
			break;
		case 15:
			setrgb(0, 255, 0);
			break;
		}
	}

	printInfo(Info::TempAcc);
	printInfo(Info::Ref);

	SerialUSB.print("Initializing ESR measuring...");
	esr::init(
		&getSampleInterfaceEsr,
		&setGainInterface
	);
	SerialUSB.println(" OK");

	SerialUSB.print("Initializing capacitance measuring...");
	cap::init(
		&getSampleInterfaceCap,
		&setGainInterface
	);
	SerialUSB.println(" OK");

	SerialUSB.println("Initialization done!");
}

void loop()
{
	static long lastDisplayMillis = 0, lastDebugMillis = 0, lastCapMillis = 0;

	static bool esrmode = true, capprevmeas = false;
	static std::uint8_t capConvNum = 0;

	if (state.intOccur)
	{
		static std::uint16_t presses = 0, releases = 0;

		const bool localPress = state.btnState;
		state.intOccur = false;
		
		// Interrupt handling code

		if (!localPress)
		{
			cap::zeroReading();
		}
	}

	// Check for esr overload
	if (esrmode)
	{
		esr::outputEnable();

		bool ol = false;
		auto esr = esr::measureESR(ol);
		ol |= (esr > 100.0) ? true : false;

		if (!ol)
		{
			if (!capprevmeas)
			{
				esr::outputEnable(false);

				esrmode = false;
				state.capIsInProgress = false;
				capConvNum = 0;
				lastCapMillis = 0;
			}
			else
			{
				state.esrValue = esr;
				state.esrIsOL = false;
			}
		}
		else
		{
			state.esrIsOL = true;
			capprevmeas = false;
			state.esrClear();
		}
	}
	if (!esrmode)
	{
		// Measure capacitance
		if (!state.capIsInProgress && (millis() > lastCapMillis))
		{
			// Check if capacitor is discharged
			if (cap::isDischarged())
			{
				cap::startMeasureMent_async();
				state.capIsInProgress = true;
				state.capIsOL = false;
			}
			else
			{
				cap::discharge();
				state.capIsOL = true;
			}
		}
		else if (state.capIsInProgress)
		{
			std::uint32_t ticks;
			bool done = false;
			if (cap::measureTicks_async(ticks))
			{
				auto cap = cap::calcCapacitance_fpd(ticks);
				auto fcap = double(cap) / 1000.0;
				fcap = (fcap < 0.0) ? 0.0 : fcap;
				state.capValue = fcap;
				state.capIsOL = false;
				state.capIsInProgress = false;

				++capConvNum;

				if ((fcap > 10.0) || (capConvNum > 1))
				{
					// No contact bouncing issues if over 10nF or
					// already done the second measurement
					done = true;
				}
				else
				{
					lastCapMillis = millis() + 100;
				}
			}
			else if (TC4->COUNT32.COUNT.reg > CAP_TIMEOUT_TICKS)
			{
				state.capIsOL = true;
				done = true;
			}

			if (done)
			{
				capprevmeas = true;
				state.capIsInProgress = false;
				esrmode = true;
				cap::stop();
			}
		}
	}

	if (state.debug && (millis() > lastDebugMillis))
	{
		lastDebugMillis = millis() + 1000;
		//const auto tempSample = adc::sample(adc::Channel::IntTemp, true);
		//adc::calibrate(tempSample, true);
		printInfo(Info::TempAcc);
		printInfo(Info::Ref);
		printInfo(Info::Supply);

		printInfo(Info::Gain_0x5);
		printInfo(Info::Gain_2x);
		Serial.println();
	}

	if (heartbeat::isConnected())
	{
		communicationLoop();
	}

	// Update display every 250ms
	if (millis() > lastDisplayMillis)
	{
		lastDisplayMillis = millis() + 250;

		disp::lcd.setCursor(0, 0);
		if (state.capIsOL || state.capIsInProgress)
		{
			disp::lcd.print(state.capIsOL ? "Cap: OL      " : "Cap: charging");
		}
		else
		{
			// Show conversion result
			auto fcap = state.capValue;
			disp::lcd.print("Cap: ");

			std::uint8_t range;
			if (fcap > 999999999.9)
			{
				range = 4;
				fcap /= 1000000000.0;
			}
			if (fcap > 999999.9)
			{
				range = 3;
				fcap /= 1000000.0;
			}
			else if (fcap > 999.9)
			{
				range = 2;
				fcap /= 1000.0;
			}
			else if (fcap > 0.999)
			{
				range = 1;
			}
			else
			{
				range = 0;
				fcap *= 1000.0;
			}

			disp::lcd.print(fcap, (range == 0) ? 0 : 1);

			static const char * ranges[] = {
				" pF",
				" nF",
				" uF",
				" mF",
				" F"
			};

			disp::lcd.print(ranges[range]);
			disp::lcd.print("       ");
		}

		disp::lcd.setCursor(0, 1);
		if (state.esrIsOL)
		{
			disp::lcd.print("ESR: OL         ");
		}
		else
		{
			// Show conversion result
			disp::lcd.print("ESR: ");
			disp::lcd.print(state.esrValue, 3);
			disp::lcd.print(" ohm  ");
		}
	}
}


void btnISR()
{
	static long lastmillis = 0;
	const bool state_ = digitalRead(PUSH_BTN);
	const auto mil = millis(), delta = mil - lastmillis;
	lastmillis = mil;

	if (!state_ && (delta < BTN_DEBOUNCE_THRESHOLD_MS))
	{
		return;
	}

	const bool val = state.btnState;
	state.btnState = !state_;
	state.intOccur = (state.btnState != val);
}

void setrgb(std::uint8_t r, std::uint8_t g, std::uint8_t b)
{
	pwm::duty(LED_RED_CC,   r);
	pwm::duty(LED_GREEN_CC, g);
	pwm::duty(LED_BLUE_CC,  b);
}

int SerialPrintf(const char * format, ...)
{
	va_list vp;
	char buf[SERIAL_PRINTF_BUFSIZE];
	buf[SERIAL_PRINTF_BUFSIZE - 1] = '\0';

	va_start(vp, format);
	auto nchars = std::vsnprintf(buf, SERIAL_PRINTF_BUFSIZE, format, vp);
	va_end(vp);

	SerialUSB.print(buf);
	return nchars;
}

std::uint32_t getSampleInterfaceEsr(bool precisemode, std::uint32_t & instantSample)
{
	const auto volts = adc::getVolts_fpd(adc::sample(adc::Channel::OutAmp, precisemode));
	instantSample = volts;
	state.esrSampleAvg -= state.esrFilled() ? state.esrSampleAvgArr[state.esrFirstIdx()] : 0;
	state.esrSampleAvg += volts;
	state.esrSampleAvgArr[state.esrSampleIdx] = volts;
	state.esrSampleIdx = ((state.esrSampleIdx + 1) < ESR_MAX_AVG_SAMPLES) ? (state.esrSampleIdx + 1) : 0;
	state.esrNumSamples += (state.esrNumSamples < ESR_MAX_AVG_SAMPLES) ? 1 : 0;

	return state.esrSampleAvg / state.esrNumSamples;
}
std::uint32_t getSampleInterfaceCap(bool precisemode, std::uint32_t & instantSample)
{
	const auto volts = adc::getVolts_fpd(adc::sample(adc::Channel::Out, precisemode));
	instantSample = volts;
	return volts;
}
void setGainInterface(std::uint8_t gain)
{
	adc::setGain(adc::Gain(gain));
}

bool comRead(char * buf, std::uint8_t & bufidx)
{
	if (!SerialUSB.available())
	{
		return false;
	}

	buf[bufidx] = SerialUSB.read();
	auto & ch = buf[bufidx];
	++bufidx;

	if (bufidx >= SERIAL_READ_BUFSIZE)
	{
		bufidx = 0;
	}

	if ((ch == '\n') || (ch == '\r') || (ch == '\0'))
	{
		ch = '\0';
		bufidx = 0;
		return (buf[0] != '\0');
	}

	buf[SERIAL_READ_BUFSIZE - 1] = '\0';
	return false;
}
void communicationLoop()
{
	static char buf[SERIAL_READ_BUFSIZE];
	static std::uint8_t bufidx = 0;

	// Try to receive data
	const auto read = comRead(buf, bufidx);
	if (!read)
	{
		return;
	}
	
	// parse contents
	if (!std::strcmp(buf, "echo"))
	{
		SerialUSB.println("Echo");
	}
	else if (!std::strcmp(buf, "debug"))
	{
		SerialUSB.println("Toggling debug mode...");
		state.debug ^= 1;
	}
	else if (!std::strcmp(buf, "temp"))
	{
		printInfo(Info::TempAcc);
	}
	else if (!std::strcmp(buf, "ref"))
	{
		printInfo(Info::Ref);
	}
	else if (!std::strcmp(buf, "supply"))
	{
		printInfo(Info::Supply);
	}
	else
	{
		SerialUSB.println("Unknown command!");
	}
}


void printInfo(Info type, std::uint8_t decPlaces, bool hasData, float uData)
{
	static const char * prelist[] = {
		"Temperature: ",
		"Temperature (slow): ",
		"Reference: ",
		"Supply: ",
		"Gain 0.5x: ",
		"Gain 2x: ",
	};
	static const char * postlist[] = {
		" deg. C",
		" deg. C",
		"V",
		"V",
		"",
		"",
	};

	const auto ptype = std::underlying_type<Info>::type(type);
	SerialUSB.print(prelist[ptype]);

	float data;
	std::uint16_t sample = 0;
	if (!hasData)
	{
		switch (type)
		{
		case Info::Temp:
		case Info::TempAcc:
			sample = adc::sample(adc::Channel::IntTemp, type == Info::TempAcc);
			data = adc::getTemp(sample);
			decPlaces = !decPlaces ? 2 : decPlaces;
			break;
		case Info::Ref:
			data = adc::calData.ref1VReal;
			decPlaces = !decPlaces ? 6 : decPlaces;
			break;
		case Info::Supply:
			data = adc::getSupply();
			decPlaces = !decPlaces ? 4 : decPlaces;
			break;
		case Info::Gain_0x5:
			data = fp::fromD(adc::calData.gainCal_FPD[std::uint8_t(adc::Gain::g0_5x)]);
			decPlaces = !decPlaces ? 4 : decPlaces;
			break;
		case Info::Gain_2x:
			data = fp::fromD(adc::calData.gainCal_FPD[std::uint8_t(adc::Gain::g2x)]);
			decPlaces = !decPlaces ? 4 : decPlaces;
			break;
		default:
			assert(!"Unknown type!");
		}
	}
	else
	{
		data = uData;
	}

	SerialUSB.print(data, decPlaces);
	SerialUSB.println(postlist[ptype]);
}
