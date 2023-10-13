#pragma once

#include <cstdint>

inline bool isBitSet(uint8_t BitArray, int BitNumber)
{
	if (((BitArray >> BitNumber) & 0x01) == 1)
		return true;

	return false;
}

inline void setBit(uint8_t& BitArray, int BitNumber)
{
	BitArray = (BitArray | (1 << BitNumber));
}

inline void clearBit(uint8_t& BitArray, int BitNumber)
{
	BitArray = (BitArray & ~(1 << BitNumber));
}