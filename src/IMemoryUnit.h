#pragma once

#include <cstdint>

class IMemoryUnit
{
public:
    virtual ~IMemoryUnit() {};

    virtual uint8_t readByte(const uint16_t& Address) = 0;

    virtual void writeByte(const uint16_t& Address, const uint8_t Value) = 0;
};

