#pragma once

#include "project_config.h"
#include "pin_setup.h"

class kicker
{
	public:
		kicker(pin &charge, pin &discharge);
	
	void check(uint32_t _actual_time);
	void keck();
	
	private:
		pin _charge;
		pin _discharge;
		bool ready;
		uint8_t state;
		uint32_t _tim;
		uint32_t _time;
};