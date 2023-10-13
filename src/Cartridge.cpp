#include "Cartridge.h"

#include <fstream>

#include "MBC.h"
#include "Logger.h"

const uint16_t CartridgeTypeAddress = 0x0147;
const uint16_t ROMSizeAddress       = 0x0148;
const uint16_t RAMSizeAddress       = 0x0149;

Cartridge::Cartridge()
	: Type(CARTRIDGE_TYPE::ROM_ONLY)
{
}

Cartridge::~Cartridge()
{
    switch (Type)
    {
    case CARTRIDGE_TYPE::MBC1_RAM_BATTERY:
    case CARTRIDGE_TYPE::MBC3_TIMER_BATTERY:
    case CARTRIDGE_TYPE::MBC3_TIMER_RAM_BATTERY:
    case CARTRIDGE_TYPE::MBC3_RAM_BATTERY:
    case CARTRIDGE_TYPE::MBC5_RAM_BATTERY:
    case CARTRIDGE_TYPE::MBC5_RUMBLE_RAM_BATTERY:
        
        std::ofstream File(FilePath + "_RAM", std::ios::out | std::ios::binary | std::ios::trunc);

        if (File.is_open())
        {
            File.write( (char*) RAM.get(), RAMSize);
            File.close();
        }
        break;
    }

    MBC.reset();
    ROM.reset();
    RAM.reset();
}

void Cartridge::loadROM(const char* PathToROM)
{
	std::ifstream File(PathToROM, std::ios::in | std::ios::binary | std::ios::ate);

    std::streampos Size;

	if (File.is_open() == true)
	{
        Size = File.tellg();

        int iSize = Size;

        File.seekg(0, std::ios::beg);

        ROM = std::unique_ptr<uint8_t>(new uint8_t[static_cast<unsigned int>(iSize)]);

        if (File.read(reinterpret_cast<char*>(ROM.get()), Size))
        {
            Logger::log("Loaded game ROM");

            if (iSize < 0x014F)
                Logger::log("Cartridge doesn't have enough data!");

            else
                loadMBC(static_cast<unsigned int>(iSize));
        }
        else
        {
            Logger::log("Failed to load game rom");
        }
;
	}
    
    File.close();
}

uint8_t Cartridge::readByte(const uint16_t& Address)
{
    return MBC->readByte(Address);
}

void Cartridge::writeByte(const uint16_t& Address, const uint8_t Value)
{
    return MBC->writeByte(Address, Value);
}

void Cartridge::loadMBC(unsigned int ActualSize)
{
    Type = static_cast<CARTRIDGE_TYPE>(ROM.get()[CartridgeTypeAddress]);
    RomSize RomSizeFlag = static_cast<RomSize>(ROM.get()[ROMSizeAddress]);
    RamSize RamSizeFlag = static_cast<RamSize>(ROM.get()[RAMSizeAddress]);

    unsigned int RomSize = (32 * 1024) << static_cast<int>(RomSizeFlag);
    
    switch (RomSizeFlag)
    {
    case RomSize::ROM_1_1MB:
        RomSize = 1179648;
        break;
    case RomSize::ROM_1_2MB:
        RomSize = 1310720;
        break;
    case RomSize::ROM_1_5MB:
        RomSize = 1572864;
        break;
    }

    if (ActualSize != RomSize)
        Logger::log("Cartridge::LoadMBC - Unexpected ROM file size.");

    switch (RamSizeFlag)
    {
    case RamSize::RAM_None:
        RAMSize = 0;
        break;
    case RamSize::RAM_2KB:
        RAMSize = (1024 * 2);
        break;
    case RamSize::RAM_8KB:
        RAMSize = (1024 * 8);
        break;
    case RamSize::RAM_32KB:
        RAMSize = (1024 * 32);
        break;
    default:
        Logger::log("Cartridge::LoadMBC - Unexpected RAM size flag.");
        break;
    }

    if (RAMSize > 0)
    {
        std::string ramPath = FilePath + "_RAM";
        RAM = std::unique_ptr<uint8_t>(new uint8_t[RAMSize]);

        std::ifstream File(ramPath, std::ios::in | std::ios::binary | std::ios::ate);
        if (File.is_open())
        {
            std::streampos size = File.tellg();

            unsigned int iSize = size;

            File.seekg(0, std::ios::beg);

            if (iSize != RAMSize)
            {
                Logger::log("Cartridge::LoadMBC - Saved RAM was not the expected size.");
            }
            else if (File.read(reinterpret_cast<char*>(RAM.get()), size))
            {
                Logger::log("Loaded game RAM");
            }
        }
    }

    switch (Type)
    {
    case CARTRIDGE_TYPE::ROM_ONLY:
        MBC = std::make_unique<ROMOnly_MBC>(ROM.get(), RAM.get());
        break;
    case CARTRIDGE_TYPE::MBC1:
    case CARTRIDGE_TYPE::MBC1_RAM:
    case CARTRIDGE_TYPE::MBC1_RAM_BATTERY:
        MBC = std::make_unique<MBC1_MBC>(ROM.get(), RAM.get());
        break;
    case CARTRIDGE_TYPE::MBC2:
    case CARTRIDGE_TYPE::MBC2_BATTERY:
        RAM.reset();
        MBC = std::make_unique<MBC2_MBC>(ROM.get());
        break;
    case CARTRIDGE_TYPE::MBC3_TIMER_BATTERY:
    case CARTRIDGE_TYPE::MBC3_TIMER_RAM_BATTERY:
    case CARTRIDGE_TYPE::MBC3:
    case CARTRIDGE_TYPE::MBC3_RAM:
    case CARTRIDGE_TYPE::MBC3_RAM_BATTERY:
        MBC = std::make_unique<MBC3_MBC>(ROM.get(), RAM.get());
        break;
    case CARTRIDGE_TYPE::MBC5:
    case CARTRIDGE_TYPE::MBC5_RAM:
    case CARTRIDGE_TYPE::MBC5_RAM_BATTERY:
    case CARTRIDGE_TYPE::MBC5_RUMBLE:
    case CARTRIDGE_TYPE::MBC5_RUMBLE_RAM:
    case CARTRIDGE_TYPE::MBC5_RUMBLE_RAM_BATTERY:
        MBC = std::make_unique<MBC5_MBC>(ROM.get(), RAM.get());
        break;
    default:
        Logger::log("Unsupported Cartridge MBC type.");
        break;
    }
}
