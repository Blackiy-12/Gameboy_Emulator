#pragma once

#include <cstdint>
#include <memory>
#include <string>

#include "IMemoryUnit.h"


enum class CARTRIDGE_TYPE
{
	ROM_ONLY						= 0x00,

	MBC1							= 0x01,
	MBC1_RAM						= 0x02,
	MBC1_RAM_BATTERY				= 0x03,
	
	MBC2							= 0x05,
	MBC2_BATTERY					= 0x06,
	
	ROM_RAM							= 0x08,
	ROM_RAM_BATTERY					= 0x09,
	MMM0							= 0x0B,
	MMM01_RAM						= 0x0C,
	MMM01_RAM_BATTERY				= 0x0D,
	
	MBC3_TIMER_BATTERY				= 0x0F,
	MBC3_TIMER_RAM_BATTERY			= 0x10,
	MBC3							= 0x11,
	MBC3_RAM						= 0x12,
	MBC3_RAM_BATTERY				= 0x13,
	
	MBC5							= 0x19,
	MBC5_RAM						= 0x1A,
	MBC5_RAM_BATTERY				= 0x1B,
	MBC5_RUMBLE						= 0x1C,
	MBC5_RUMBLE_RAM					= 0x1D,
	MBC5_RUMBLE_RAM_BATTERY			= 0x1E,
	
	MBC6							= 0x20,
	MBC7_SENSOR_RUMBLE_RAM_BATTERY  = 0x22,
	POCKET_CAMERA					= 0xFC,
	BANDAI_TAMA5					= 0xFD,
	HuC3							= 0xFE,
	HuC1_RAM_BATTERY				= 0xFF
	
};

enum class RAM_SIZE
{
	NONE		= 0x00,
	UNUSED		= 0x01,
	BANKS_1		= 0x02,
	BANKS_4		= 0x03,
	BANKS_8		= 0x05,
	BANKS_16	= 0x04
};

enum class RomSize
{
 ROM_32KB    = 0x00,

 ROM_64KB    = 0x01,

 ROM_128KB   = 0x02,

 ROM_256KB   = 0x03,

 ROM_512KB   = 0x04,

 ROM_1MB     = 0x05,

 ROM_2MB     = 0x06,

 ROM_4MB     = 0x07,

 ROM_1_1MB   = 0x52,

 ROM_1_2MB   = 0x53,

 ROM_1_5MB   = 0x54
};

enum class RamSize
{
	RAM_None        = 0x00,

	RAM_2KB         = 0x01,

	RAM_8KB         = 0x02,

	RAM_32KB        = 0x03
};

class Cartridge : public IMemoryUnit
{
public:
	Cartridge();

	~Cartridge();

public:

	void loadROM(const char* PathToROM);

	uint8_t readByte(const uint16_t& Address);
	
	void writeByte(const uint16_t& Address, const uint8_t Value);

private:
	void loadMBC(unsigned int ActualSize);

private:
	std::string FilePath;

	CARTRIDGE_TYPE Type;

	unsigned int RAMSize;
	
	std::unique_ptr<uint8_t> ROM;
	
	std::unique_ptr<uint8_t> RAM;
	
	std::unique_ptr<IMemoryUnit> MBC;
};

