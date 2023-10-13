#include "MBC.h"

#include "Logger.h"

#include <cstring>
#include <algorithm>

uint8_t EnableRAM       = 0x0A;
uint8_t ROMBankMode     = 0x00;
uint8_t RAMBankMode     = 0x01;

MBC::MBC(uint8_t* ROMPtr, uint8_t* RAMPtr) 
    :   ROM(ROMPtr),
        RAM(RAMPtr),
        RAMEnabled(false)
{
}

MBC::~MBC()
{
}

ROMOnly_MBC::ROMOnly_MBC(uint8_t* ROMPtr, uint8_t* RAMPtr)
    : MBC(ROMPtr, RAMPtr)
{
}

ROMOnly_MBC::~ROMOnly_MBC()
{
}

uint8_t ROMOnly_MBC::readByte(const uint16_t& Address)
{
    if (Address <= 0x7FFF)
    {
        return ROM[Address];
    }
    else if (Address >= 0xA000 && Address <= 0xBFFF)
    {
        if (RAM == nullptr)
            return 0x00;

        return RAM[Address - 0xA000];
    }

    return 0x00;
}

void ROMOnly_MBC::writeByte(const uint16_t& Address, const uint8_t Value)
{
    if (Address >= 0xA000 && Address <= 0xBFFF)
        RAM[Address - 0xA000] = Value;
}

MBC1_MBC::MBC1_MBC(uint8_t* ROMPtr, uint8_t* RAMPtr)
    :   MBC(ROMPtr, RAMPtr),
        ROMBankLower(0x01),
        ROMRAMBankUpper(0x00),
        ROMRAMMode(ROMBankMode)
{
}

MBC1_MBC::~MBC1_MBC()
{
}

uint8_t MBC1_MBC::readByte(const uint16_t& Address)
{
    if (Address <= 0x3FFF)
        return ROM[Address];

    else if (Address <= 0x7FFF)
    {
        uint8_t TargetBank = ROMBankLower;
        if (ROMRAMMode == ROMBankMode)
            TargetBank |= (ROMRAMBankUpper << 4);

        unsigned int Target = (Address - 0x4000);
        Target += (0x4000 * TargetBank);
        return ROM[Target];
    }
    else if (Address >= 0xA000 && Address <= 0xBFFF)
    {
        
        if (RAMEnabled == false)
            return 0xFF;

        if (RAM == nullptr)
            return 0xFF;

        unsigned int Target = Address - 0xA000;
        if (ROMRAMMode == RAMBankMode)
            Target += (0x2000 * ROMRAMBankUpper);

        return RAM[Target];
    }

    return 0x00;
}

void MBC1_MBC::writeByte(const uint16_t& Address, const uint8_t Value)
{
    if (Address <= 0x1FFF)
        RAMEnabled = ((Value & EnableRAM) == EnableRAM);

    else if (Address <= 0x3FFF)
    {
        ROMBankLower = Value & 0x1F;

        if (ROMBankLower == 0x00)
            ROMBankLower = 0x01;
    }

    else if (Address <= 0x5FFF)
       ROMRAMBankUpper = Value & 0x03;

    else if (Address <= 0x7FFF)
        ROMRAMMode = Value & 0x01;

    else if (Address >= 0xA000 && Address <= 0xBFFF)
    {
        if (RAMEnabled == true)
            return;

        if (RAM == nullptr)
            return;

        
        unsigned int Target = Address - 0xA000;
        if (ROMRAMMode == RAMBankMode)
            Target += (0x2000 * ROMRAMBankUpper);

        RAM[Target] = Value;
    }
}

MBC2_MBC::MBC2_MBC(uint8_t* ROMPtr)
    :   MBC(ROMPtr, new uint8_t[0x1FF + 1]),
        ROMBank(0x01)
{
}

MBC2_MBC::~MBC2_MBC()
{
}

uint8_t MBC2_MBC::readByte(const uint16_t& Address)
{
    if (Address <= 0x3FFF)
        return ROM[Address];

    else if (Address <= 0x7FFF)
    {      
        unsigned int Target = (Address - 0x4000);
        Target += (0x4000 * ROMBank);
        return ROM[Target];
    }

    else if (Address >= 0xA000 && Address <= 0xA1FF)
    {
        if (RAMEnabled == false)
            return 0xFF;

        return RAM[Address - 0xA000];
    }

    return 0x00;
}

void MBC2_MBC::writeByte(const uint16_t& Address, const uint8_t Value)
{
    if (Address <= 0x1FFF)
    {
        if ((Address & 0x0100) == 0x0000)
            RAMEnabled = ((Value & EnableRAM) == EnableRAM);
    }
    else if (Address <= 0x3FFF)
    {
        if ((Address & 0x0100) == 0x0000)
            ROMBank = (Value & 0x0F);
    }
    else if (Address >= 0xA000 && Address <= 0xA1FF)
    {
        
        if (RAMEnabled == false)
            return;

        RAM[Address - 0xA000] = (Value & 0x0F);
    }
}

MBC3_MBC::MBC3_MBC(uint8_t* ROMPtr, uint8_t* RAMPtr)
    :   MBC(ROMPtr, RAMPtr),
        ROMBank(0x01),
        RAMBank(0x00)
{
    memset(RTCRegisters, 0x00, std::end(RTCRegisters) - std::begin(RTCRegisters));
}

MBC3_MBC::~MBC3_MBC()
{
}

uint8_t MBC3_MBC::readByte(const uint16_t& Address)
{

    if (Address <= 0x3FFF)
        return ROM[Address];

    else if (Address <= 0x7FFF)
    {
        unsigned int target = (Address - 0x4000);
        target += (0x4000 * ROMBank);
        return ROM[target];
    }

    else if (Address >= 0xA000 && Address <= 0xBFFF)
    {
        if (RAMEnabled == false)
            return 0xFF;

        if (RAM == nullptr)
            return 0xFF;

        if (RAMBank <= 0x03)
        {
            unsigned int target = Address - 0xA000;
            target += (0x2000 * RAMBank);
            return RAM[target];
        }

        else if (RAMBank >= 0x08 && RAMBank <= 0x0C)
            return RTCRegisters[RAMBank - 0x08];
    }

    return 0x00;
}

void MBC3_MBC::writeByte(const uint16_t& Address, const uint8_t Value)
{
    if (Address <= 0x1FFF)
        RAMEnabled = ((Value & EnableRAM) == EnableRAM);

    else if (Address <= 0x3FFF)
    {
        ROMBank = (Value & 0x7F);
        if (ROMBank == 0x00)
            ROMBank = 0x01;
    }

    else if (Address <= 0x5FFF)
        RAMBank = Value;

    else if (Address <= 0x7FFF)
        return;

    else if (Address >= 0xA000 && Address <= 0xBFFF)
    {
        if (!RAMEnabled)
            return;

        if (RAM == nullptr)
            return;

        if (RAMBank <= 0x03)
        {
            unsigned int target = Address - 0xA000;
            target += (0x2000 * RAMBank);
            RAM[target] = Value;
            return;
        }

        else if (RAMBank >= 0x08 && RAMBank <= 0x0C)
        {
            RTCRegisters[RAMBank - 0x08] = Value;
            return;
        }
    }
}

MBC5_MBC::MBC5_MBC(uint8_t* ROMPtr, uint8_t* RAMPtr)
    :   MBC(ROMPtr, RAMPtr),
        RAMG(0x00),
        ROMBank(0x0000),
        RAMBank(0x00)
{
}

MBC5_MBC::~MBC5_MBC()
{
}

uint8_t MBC5_MBC::readByte(const uint16_t& Address)
{
    if (Address <= 0x3FFF)
        return ROM[Address];

    else if (Address <= 0x7FFF)
    {
        unsigned int Target = (Address - 0x4000);
        Target += (0x4000 * ROMBank);
        return ROM[Target];
    }

    else if (Address >= 0xA000 && Address <= 0xBFFF)
    {
        unsigned int Target = Address - 0xA000;
        Target += (0x2000 * RAMBank);
        return RAM[Target];
    }

    return 0x00;
}

void MBC5_MBC::writeByte(const uint16_t& Address, const uint8_t Value)
{
    if (Address <= 0x1FFF)
        RAMG = Value;

    else if (Address <= 0x2FFF)
        ROMBank = (ROMBank & 0xFF00) | Value;

    else if (Address <= 0x3FFF)
    {
        uint16_t upper = (uint16_t)(Value & 0x01);
        ROMBank = (ROMBank & 0x00FF) | (upper << 8);
        return;
    }

    else if (Address <= 0x4FFF)
        RAMBank = (Value & 0x0F);

    else if (Address >= 0xA000 && Address <= 0xBFFF)
    {
        unsigned int Target = Address - 0xA000;
        Target += (0x2000 * RAMBank);
        RAM[Target] = Value;
    }
}
