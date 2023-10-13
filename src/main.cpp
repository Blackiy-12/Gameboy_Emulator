#include "GameboyEmu.h"


int main(int argc, char** argv)
{
	GameboyEmu Emu(2);

	Emu.start("PATH");

	return 0;
}