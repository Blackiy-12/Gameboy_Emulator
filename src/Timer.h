#pragma once

#include <memory>

#include "IMemoryUnit.h"

class CPU;

class Counter
{
public:
    Counter(uint8_t Frequency);
    
    void step(unsigned int Cycles);

    bool hasOverflow();

    uint8_t getValue();
    
    void setValue(uint8_t Value);
    
    void setFrequency(uint8_t Frequency);
    
    void start();
    
    void stop();

private:
    bool Running;
    
    bool Overflow;

    uint8_t Value;
    
    uint8_t Frequency;
    
    int  Cycles;
};

class Timer : public IMemoryUnit
{
public:
    Timer(CPU* CPUPtr);

    ~Timer();

    void step(unsigned long Cycles);

    uint8_t readByte(const uint16_t& Address);

    void writeByte(const uint16_t& Address, const uint8_t Value);

private:
    CPU* CPUPtr;

    std::unique_ptr<Counter> DividerCounter;

    std::unique_ptr<Counter> TimerCounter;

    uint8_t TimerModulo;
    
    uint8_t TimerControl;
};


