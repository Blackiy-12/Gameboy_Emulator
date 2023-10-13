#include "Timer.h"

#include "CPU.h"
#include "Logger.h"
#include "BitOperation.h"

const uint16_t Divider_Address = 0xFF04;
const uint16_t TimerCounter_Address = 0xFF05;
const uint16_t TimerModulo_Address = 0xFF06;
const uint16_t TimerControl_Address = 0xFF07;

const int FrequencyCounts[]
{
    1024, 16, 64, 256
};

uint8_t Frequency4096       = 0x00;
uint8_t Frequency262144     = 0x01;
uint8_t Frequency65536      = 0x02;
uint8_t Frequency16384      = 0x03;

Counter::Counter(uint8_t Frequency)
    :   Running(true),
        Overflow(false),
        Value(0x00),
        Frequency(Frequency),
        Cycles(FrequencyCounts[Frequency])
{
}

void Counter::step(unsigned int Cycles)
{
    if (Running == false)
        return;

    this->Overflow = false;
    this->Cycles -= Cycles;
    while (this->Cycles <= 0)
    {
        
        this->Cycles += FrequencyCounts[Frequency];
        Value++;
        if (Value == 0x00)
        {
            this->Overflow = true;
            return;
        }
    }
}

bool Counter::hasOverflow()
{
    return this->Overflow;
}

uint8_t Counter::getValue()
{
    return this->Value;
}

void Counter::setValue(uint8_t Value)
{
    this->Value = Value;
}

void Counter::setFrequency(uint8_t Frequency)
{
    Frequency = Frequency;
    Cycles = FrequencyCounts[Frequency];
}

void Counter::start()
{
    Running = true;
}

void Counter::stop()
{
    Running = false;
}

Timer::Timer(CPU* CPUPtr)
    :   CPUPtr(CPUPtr),
        TimerModulo(0x00),
        TimerControl(0x00)
{
    DividerCounter = std::unique_ptr<Counter>(new Counter(Frequency16384));
    TimerCounter = std::unique_ptr<Counter>(new Counter(Frequency4096));
}

Timer::~Timer()
{
    DividerCounter.reset();
    TimerCounter.reset();

}

void Timer::step(unsigned long cycles)
{
    DividerCounter->step(cycles);

   
    TimerCounter->step(cycles);

    if (TimerCounter->hasOverflow())
    {
       
        TimerCounter->setValue(TimerModulo);

        if (CPUPtr != nullptr)
        {
            CPUPtr->triggerInterrupt(INT50);
        }
    }
}

uint8_t Timer::readByte(const uint16_t& Address)
{
    switch (Address)
    {
    case Divider_Address:
        return DividerCounter->getValue();
    case TimerCounter_Address:
        return TimerCounter->getValue();
    case TimerModulo_Address:
        return TimerModulo;
    case TimerControl_Address:
        return TimerControl;
    default:
        return 0x00;
    }
}

void Timer::writeByte(const uint16_t& Address, const uint8_t Value)
{
    switch (Address)
    {
    case Divider_Address:
        DividerCounter->setValue(0x00);
        break;

    case TimerCounter_Address:
        TimerCounter->setValue(Value);
        break;

    case TimerModulo_Address:
        TimerModulo = Value;

    case TimerControl_Address:
        if (isBitSet(Value, 2) == true)
            TimerCounter->start();

        else
            TimerCounter->stop();

        TimerCounter->setFrequency(Value & 0x03);
        break;

    default:
        Logger::log("Timer::ReadByte cannot write to address.");
        break;
    }
}
