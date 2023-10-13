#include "MMU.h"

#include "Logger.h"

#include <fstream>

MMU::MMU()
    : Booting(0xFF)
{
    this->registerMemoryUnit(0x0000, 0xFFFF, this);
}

MMU::~MMU()
{
}

void MMU::registerMemoryUnit(const uint16_t& StartRange, const uint16_t& EndRange, IMemoryUnit* Unit)
{
    for (int index = StartRange; index <= EndRange; index++)
    {
        MemoryUnits[index] = Unit;
    }
}

uint16_t MMU::read2Bytes(const uint16_t& Address)
{
    uint16_t val = read(Address + 1);
    val  = val << 8;
    val |= read(Address);
    return val;
}

void MMU::loadBootROM(const char* BootROMPath)
{
    if (BootROMPath == nullptr)
    {
        Booting = 0x01;
        return;
    }

    Booting = 0x00;
    std::streampos Size;

    std::ifstream File(BootROMPath, std::ios::in | std::ios::binary);

    if (File.is_open())
    {
        Size = File.tellg();
    
        int iSize = Size;

        if (iSize < 256)
        {
            Logger::log("Boot rom at is the wrong size!");
        }
        else
        {
            File.seekg(0, std::ios::beg);

            BIOS = std::unique_ptr<uint8_t>(new uint8_t[static_cast<unsigned int>(iSize)]);
            
            if (File.read(reinterpret_cast<char*>(BIOS.get()), iSize))
                Logger::log("Loaded boot rom");

            else
                Logger::log("Failed to load boot ROM ");
        }
    }

    File.close();
}

uint8_t MMU::read(const uint16_t& Address)
{
    if ((Booting == 0x00) && (Address <= 0x00FF))
    {
        if (BIOS == nullptr)
        {
            Logger::log("Access Violation! You can't read from the boot rom if it isn't loaded!");
            return 0x00;
        }

        return BIOS.get()[Address];
    }

    return MemoryUnits[Address]->readByte(Address);
}

void MMU::write(const uint16_t& Address, const uint8_t Value)
{
    if ((Booting == 0x00) && (Address <= 0x00FF))
    {
        Logger::log("Access Violation! You can't write to the boot ROM");
    }

    return MemoryUnits[Address]->writeByte(Address, Value);
}

uint8_t MMU::readByte(const uint16_t& Address)
{
    if (Address >= 0xC000 && Address <= 0xCFFF)
        return Bank0[Address - 0xC000];

    else if (Address >= 0xD000 && Address <= 0xDFFF)
        return Bank1[Address - 0xD000];
    
    else if (Address >= 0xE000 && Address <= 0xEFFF)
        return Bank0[Address - 0xE000];
    
    else if (Address >= 0xF000 && Address <= 0xFDFF)
        return Bank1[Address - 0xF000];
    
    else if (Address >= 0xFEA0 && Address <= 0xFEFF)
        return 0x00;
    
    else if (Address >= 0xFF80 && Address <= 0xFFFE)
        return HRAM[Address - 0xFF80];
    
    else if (Address == 0xFFFF)
        return IE;

    else if (Address == 0xFF0F)
        return IF;

    else if (Address == 0xFF50)
        return Booting;

    else if (Address == 0xFF4D)
        return Key1;
   
    return 0x00;
}

void MMU::writeByte(const uint16_t& Address, const uint8_t Value)
{

    if (Address >= 0xC000 && Address <= 0xCFFF)
        Bank0[Address - 0xC000] = Value;

    else if (Address >= 0xD000 && Address <= 0xDFFF)
        Bank1[Address - 0xD000] = Value;

    else if (Address >= 0xE000 && Address <= 0xEFFF)
        Bank0[Address - 0xE000] = Value;

    else if (Address >= 0xF000 && Address <= 0xFDFF)
        Bank1[Address - 0xF000] = Value;

    else if (Address >= 0xFF80 && Address <= 0xFFFE)
        HRAM[Address - 0xFF80] = Value;

    else if (Address == 0xFFFF)
        IE = Value;

    else if (Address == 0xFF0F)
        IF = Value;

    else if (Address == 0xFF50)
        Booting = Value;

    else if (Address == 0xFF4D)
        Key1 = Value;

}
