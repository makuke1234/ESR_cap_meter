#include "main.hpp"
#include "ADC.hpp"
#include "display.hpp"
#include "esrCap.hpp"
#include "pwm.hpp"
#include "usbheartbeat.hpp"

static bool s_int = false, s_btnState = false;

void setup()
{
	clk::pll96(GCLK_CLKCTRL_GEN_GCLK3_Val, 8000000U);
	clk::initGCLK(GCLK_CLKCTRL_GEN_GCLK4_Val, GCLK_GENCTRL_SRC_DPLL96M_Val);

	//heartbeat::init();

	pinMode(LED_RED,   OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE,  OUTPUT);

	pwm::init(LED_PWM_FREQUENCY);
	pwm::add(LED_RED_CC,   LED_RED);
	pwm::add(LED_GREEN_CC, LED_GREEN);
	pwm::add(LED_BLUE_CC,  LED_BLUE);

	SerialUSB.begin(DEFAULT_BAUDRATE);
	bool connected = false;
	if (heartbeat::isConnected())
	{
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

	SerialUSB.print("Initializing button...");
	pinMode(PUSH_BTN, INPUT);
	// Attach interrupt to the button
	//attachInterrupt(PUSH_BTN, &btnISR, CHANGE);
	SerialUSB.println(" OK");

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

	const auto temp = adc::getTemp(adc::sample(adc::Channel::IntTemp, true));
	SerialUSB.print("Temperature: ");
	SerialUSB.print(temp);
	SerialUSB.println(" deg. C");

	SerialUSB.print("Reference voltage: ");
	SerialUSB.print(adc::calData.ref1VReal, 6);
	SerialUSB.println("V");

	SerialUSB.print("Initializing ESR measuring...");
	esr::init(
		&getSampleInterfaceEsr,
		&setGainInterface
	);
	SerialUSB.println(" OK");

	esr::outputEnable();
	esr::setFrequency(1000000);

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
	static long last = 0;
	static bool measure = false;

	if (s_int)
	{
		static std::uint16_t presses = 0, releases = 0;

		const bool localPress = s_btnState;
		s_int = false;
		
		// Interrupt handling code
	}
	if (!measure && !digitalRead(PUSH_BTN))
	{
		setrgb(255, 0, 255);
		delay(100);
		while (!digitalRead(PUSH_BTN));
		cap::startMeasureMent_async();
		measure = true;
	}

	static std::uint32_t ticks = 0;
	if (measure && cap::measureTicks_async(ticks, false))
	{
		setrgb(0, 255, 0);
		measure = false;
	}
	else if (measure)
	{
		ticks = TC4->COUNT32.COUNT.reg;
	}
	double fticks = double(ticks) / 48000000.0;
	disp::lcd.setCursor(0, 0);
	disp::lcd.print("Time: ");
	disp::lcd.print(fticks, 8);
	disp::lcd.print("          ");

	const auto tempSample = adc::sample(adc::Channel::IntTemp, true);
	//adc::calibrate(tempSample, true);
	const auto temp = adc::getTemp(tempSample);
	const auto supply = adc::getSupply();


	disp::lcd.setCursor(0, 1);
	disp::lcd.print("Temp: ");
	disp::lcd.print(temp, 2);
	disp::lcd.print(" deg. C");

	SerialUSB.print("Ref: ");
	SerialUSB.print(adc::calData.ref1VReal, 6);
	SerialUSB.print("V; Supply: ");
	SerialUSB.print(supply, 4);
	
	SerialUSB.print("V; Gain 0.5x: ");
	auto gain = adc::calData.gainCal_FPD[std::uint8_t(adc::Gain::g0_5x)];	
	SerialUSB.print(fp::fromD(gain), 5);

	SerialUSB.print("; Gain 2x: ");
	gain = adc::calData.gainCal_FPD[std::uint8_t(adc::Gain::g2x)];
	SerialUSB.print(fp::fromD(gain), 5);
	
	SerialUSB.println();

	while (!s_int && ((millis() - last) < 25))
	{
		if (!digitalRead(PUSH_BTN))
		{
			break;
		}
	}
	last = millis();
}


void btnISR()
{
	static long lastmillis = 0;
	const bool state = digitalRead(PUSH_BTN);
	const auto mil = millis(), delta = mil - lastmillis;
	lastmillis = mil;

	if (!state && (delta < BTN_DEBOUNCE_THRESHOLD_MS))
	{
		return;
	}

	const bool val = s_btnState;
	s_btnState = !state;
	s_int = (s_btnState != val);
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

std::uint32_t getSampleInterfaceEsr(bool precisemode)
{
	return adc::getVolts_fpd(adc::sample(adc::Channel::OutAmp, precisemode));
}
std::uint32_t getSampleInterfaceCap(bool precisemode)
{
	return adc::getVolts_fpd(adc::sample(adc::Channel::Out, precisemode));
}
void setGainInterface(std::uint8_t gain)
{
	adc::setGain(adc::Gain(gain));
}
