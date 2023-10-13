#pragma once

#include "IMemoryUnit.h"

#include <memory>

class MMU : IMemoryUnit
{
public:
    MMU();
    
    ~MMU();

public:
    void registerMemoryUnit(const uint16_t& StartRange, const uint16_t& EndRange, IMemoryUnit* Unit);
    
    uint16_t read2Bytes(const uint16_t& Address);
    
    void loadBootROM(const char* BootROMPath);

    uint8_t read(const uint16_t& Address);
    
    void write(const uint16_t& Address, const uint8_t Value);

    uint8_t readByte(const uint16_t& Address) override;

    void writeByte(const uint16_t& Address, const uint8_t Value) override;

private:
    // Booting
    uint8_t Booting;
    
    std::unique_ptr<uint8_t> BIOS;

    // Memory
    IMemoryUnit* MemoryUnits[0xFFFF + 1];
    
    uint8_t Bank0[0x0FFF + 1];   // 4k work RAM Bank 0
    
    uint8_t Bank1[0x0FFF + 1];   // 4k work RAM Bank 1
    
    uint8_t HRAM[0x007E + 1];    // HRAM

    /*
        Interrupts

        Bit     When 0          When 1
        0       VBlank off      VBlank on
        1       LCD stat off    LCD stat on
        2       Timer off       Timer on
        3       Serial off      Serial on
        4       Joypad off      Joypad on

    */

    uint8_t IE; // Interrupt enable register (0xFFFF)
    
    uint8_t IF; // Interrupt flag register (0xFF0F)
    
    uint8_t Key1;
};

