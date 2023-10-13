#include "Joypad.h"

#include "CPU.h"
#include "BitOperation.h"

const uint16_t JoypadAddress = 0xFF00;

Joypad::Joypad(CPU* CPUPtr)
    :   CPUPtr(CPUPtr),
        SelectValues(0x00),
        InputValues(0x00),
        ButtonValues(0x00)
{
}

Joypad::~Joypad()
{
}

void Joypad::setInput(uint8_t Input, uint8_t Buttons)
{
    uint8_t InputChanges = !isBitSet(SelectValues, 4) ? (InputValues ^ Input) : 0x00;
    uint8_t buttonChanges = !isBitSet(SelectValues, 5) ? (ButtonValues ^ Buttons) : 0x00;

    InputValues = Input;
    ButtonValues = Buttons;


    if ((CPUPtr != nullptr) && ((InputChanges > 0x00) || (buttonChanges > 0x00)))
    {
        CPUPtr->triggerInterrupt(INT60);
    }
}

uint8_t Joypad::readByte(const uint16_t& Address)
{
    uint8_t Input = 0x00;

    switch (Address)
    {
    case JoypadAddress:

        if (isBitSet(SelectValues, 4) == false)
        {
            Input |= InputValues;
        }

        if (isBitSet(SelectValues, 5) == false)
        {
            Input |= ButtonValues;
        }

        return ((SelectValues | 0x0F) ^ Input) & 0x3F;
    default:
        return 0x00;
    }
}

void Joypad::writeByte(const uint16_t& Address, const uint8_t Value)
{
    switch (Address)
    {
    case JoypadAddress:
        SelectValues = (Value & 0x30);
        break;
    default:
        break;
    }
}
