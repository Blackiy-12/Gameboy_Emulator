#pragma once

#include "IMemoryUnit.h"

class CPU;

class Joypad : public IMemoryUnit
{
public:
    Joypad(CPU* CPUPtr);
   
    ~Joypad();

    void setInput(uint8_t Input, uint8_t Buttons);

    uint8_t readByte(const uint16_t& Address);

    void writeByte(const uint16_t& Address, const uint8_t Value);

private:
    CPU* CPUPtr;

    uint8_t SelectValues;
    
    uint8_t InputValues;
    
    uint8_t ButtonValues;
};
