#pragma once

#include "IMemoryUnit.h"

class Serial : public IMemoryUnit
{
public:
    Serial();

    ~Serial();

public:
    uint8_t readByte(const uint16_t& Address);
    
    void writeByte(const uint16_t& Address, const uint8_t Value);

private:
    uint8_t Data;
};

