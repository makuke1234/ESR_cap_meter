#include "main.hpp"
#include "ADC.hpp"
#include "display.hpp"
#include "esrCap.hpp"

static bool s_int = false, s_btnState = false;

void setup()
{
	SerialUSB.begin(DEFAULT_BAUDRATE);
	delay(10);
	SerialUSB.println("Serial connection initialized.");

	SerialUSB.print("Initializing RGB LED outputs...");
	pinMode(LED_RED,   OUTPUT);
	pinMode(LED_GREEN, OUTPUT);
	pinMode(LED_BLUE,  OUTPUT);
	SerialUSB.println(" OK");

	SerialUSB.print("Initializing button...");
	pinMode(PUSH_BTN, INPUT);
	// Attach interrupt to the button
	attachInterrupt(PUSH_BTN, &btnISR, CHANGE);
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
			setrgb(1, 0, 0);
			break;
		case 10:
			setrgb(1, 0, 1);
			break;
		case 12:
			setrgb(1, 1, 0);
			break;
		case 13:
			setrgb(0, 1, 1);
			break;
		case 14:
			setrgb(0, 0, 1);
			break;
		case 15:
			setrgb(0, 1, 0);
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


	SerialUSB.println("Initialization done!");
}

void loop()
{
	static long last = 0;

	if (s_int)
	{
		static std::uint16_t presses = 0, releases = 0;

		const bool localPress = s_btnState;
		s_int = false;
		setrgb(localPress, 0, 0);

		if (!presses && !releases)
		{
			disp::lcd.clear();
			disp::lcd.setCursor(0, 0);
			disp::lcd.print("Presses:");
			disp::lcd.setCursor(0, 1);
			disp::lcd.print("Releases:");

		
		}
		if (localPress)
		{
			const auto tempSample = adc::sample(adc::Channel::IntTemp);
			const auto temp = adc::getTemp(tempSample);
			SerialUSB.print("Temperature: ");
			SerialUSB.print(temp);
			SerialUSB.print(" deg. C; Voltage: ");
			SerialUSB.print(adc::getVolts(tempSample), 4);
			SerialUSB.print("; sample: ");
			SerialUSB.print(tempSample);
			SerialUSB.println();

			
		}

		presses  += localPress == true;
		releases += localPress == false;

		//disp::lcd.setCursor(9, 0);
		//disp::lcd.print(presses);

		//disp::lcd.setCursor(10, 1);
		//disp::lcd.print(releases);
	}

	const auto tempSample = adc::sample(adc::Channel::IntTemp, true);
	adc::calibrate(tempSample, true);
	const auto temp = adc::getTemp(tempSample);
	disp::lcd.setCursor(0, 0);
	disp::lcd.print("Ref: ");
	disp::lcd.print(adc::calData.ref1VReal, 6);
	disp::lcd.print("V");

	disp::lcd.setCursor(0, 1);
	disp::lcd.print("Temp: ");
	disp::lcd.print(temp, 2);
	disp::lcd.print(" deg. C");

	SerialUSB.println("Testing...");

	//setrgb(1, 1, 0);
	while (!s_int && ((millis() - last) < 1000));
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
	digitalWrite(LED_RED,   r);
	digitalWrite(LED_GREEN, g);
	digitalWrite(LED_BLUE,  b);
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
