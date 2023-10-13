#pragma once

#include "IMemoryUnit.h"

class APU : public IMemoryUnit
{
public:
	APU() {};

	~APU() {};

public:
	void step(unsigned long Cycles) {};

	uint8_t readByte(const uint16_t& Address) { return 0x00; };

	void writeByte(const uint16_t& Address, const uint8_t Value) {};

};

