#include "GameboyEmu.h"


int main(int argc, char** argv)
{
	GameboyEmu Emu(2);

	Emu.start("D:/Projects/Gameboy_Emulator/res/MK.gb");

	return 0;
}