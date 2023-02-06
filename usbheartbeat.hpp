#pragma once

#include "clocks.hpp"

#include <Arduino.h>

namespace heartbeat
{
	void init();
	bool isConnected();
}
