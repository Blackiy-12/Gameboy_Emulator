#include "Serial.h"

const uint16_t SerialTransferDataAddress       = 0xFF01;
const uint16_t SerialTransferControlAddress = 0xFF02;

Serial::Serial()
    : Data(0x00)
{
}

Serial::~Serial()
{
}

uint8_t Serial::readByte(const uint16_t& Address)
{
    switch (Address)
    {
    case SerialTransferDataAddress:
        return Data;
    case SerialTransferControlAddress:
        return 0x00;
    default:
        return 0x00;
    }
}

void Serial::writeByte(const uint16_t& Address, const uint8_t Value)
{
    switch (Address)
    {
    case SerialTransferDataAddress:
        Data = Value;
        return;
    case SerialTransferControlAddress:
        return;
    default:
        return;
    }
}
