#include "CPU.h"

#include "BitOperation.h"

#include <algorithm>

const uint8_t ZeroFlag			= 7;
const uint8_t SubtractFlag		= 6;
const uint8_t HalfCarryFlag		= 5;
const uint8_t CarryFlag			= 4;

CPU::CPU() 
	: 
	Cycles(0),
	Halted(false),
	//Register
	AF(0x0000), BC(0x0000), DE(0x0000), HL(0x0000), SP(0x0000), PC(0x0000),
	IME(0x00)
{
	for (unsigned int index = 0; index < std::end(InstructionMap) - std::begin(InstructionMap); index++)
		InstructionMap[index] = nullptr;

	for (unsigned int index = 0; index < std::end(InstructionCBMap) - std::begin(InstructionCBMap); index++)
		InstructionCBMap[index] = nullptr;


	this->setupInstructionMap();

	RegisterMap16Bit[0x00] = &BC;
	RegisterMap16Bit[0x01] = &DE;
	RegisterMap16Bit[0x02] = &HL;
	RegisterMap16Bit[0x03] = &SP;
}

CPU::~CPU()
{
	TimerPtr.reset();
	SerialPtr.reset();
	JoypadPtr.reset();
	GPUPtr.reset();
	CartridgePtr.reset();
	MMUPtr.reset();
}

void CPU::initialize()
{
	MMUPtr = std::make_unique<MMU>();

	CartridgePtr = std::make_unique<Cartridge>();

	GPUPtr = std::make_unique<GPU>(MMUPtr.get(), this);

	APUPtr = std::make_unique<APU>();

	JoypadPtr = std::make_unique<Joypad>(this);

	SerialPtr = std::make_unique<Serial>();

	TimerPtr = std::unique_ptr<Timer>(new Timer(this));

	MMUPtr->registerMemoryUnit(0x0000, 0x7FFF, CartridgePtr.get());

	MMUPtr->registerMemoryUnit(0x8000, 0x9FFF, GPUPtr.get());

	MMUPtr->registerMemoryUnit(0xA000, 0xBFFF, CartridgePtr.get());

	MMUPtr->registerMemoryUnit(0xFE00, 0xFE9F, GPUPtr.get());

	MMUPtr->registerMemoryUnit(0xFF00, 0xFF00, JoypadPtr.get());

	MMUPtr->registerMemoryUnit(0xFF01, 0xFF02, SerialPtr.get());

	MMUPtr->registerMemoryUnit(0xFF04, 0xFF07, TimerPtr.get());

	MMUPtr->registerMemoryUnit(0xFF10, 0xFF3F, APUPtr.get());

	MMUPtr->registerMemoryUnit(0xFF40, 0xFF4C, GPUPtr.get());
	MMUPtr->registerMemoryUnit(0xFF4E, 0xFF4F, GPUPtr.get());

	MMUPtr->registerMemoryUnit(0xFF51, 0xFF55, GPUPtr.get());
	MMUPtr->registerMemoryUnit(0xFF57, 0xFF6B, GPUPtr.get());
	MMUPtr->registerMemoryUnit(0xFF6D, 0xFF6F, GPUPtr.get());
}

void CPU::loadCartridge(const char* PathToCartridge, const char* BootPath)
{
	MMUPtr->loadBootROM(BootPath);

	if (MMUPtr->read(0xFF50) != 0x00)
	{
		PC = 0x0100;

		AF = 0x01B0;
		BC = 0x0013;
		DE = 0x00D8;
		HL = 0x014D;

		SP = 0xFFFE;

		MMUPtr->write(0xFF05, 0x00); 
		MMUPtr->write(0xFF06, 0x00); 
		MMUPtr->write(0xFF07, 0x00); 

		MMUPtr->write(0xFF10, 0x80); 
		MMUPtr->write(0xFF11, 0xBF); 
		MMUPtr->write(0xFF12, 0xF3); 
		MMUPtr->write(0xFF14, 0xBF); 
		MMUPtr->write(0xFF16, 0x3F); 
		MMUPtr->write(0xFF17, 0x00); 
		MMUPtr->write(0xFF19, 0xBF); 
		MMUPtr->write(0xFF1A, 0x7F); 
		MMUPtr->write(0xFF1B, 0xFF); 
		MMUPtr->write(0xFF1C, 0x9F); 
		MMUPtr->write(0xFF1E, 0xBF); 
		MMUPtr->write(0xFF20, 0xFF); 
		MMUPtr->write(0xFF21, 0x00); 
		MMUPtr->write(0xFF22, 0x00); 
		MMUPtr->write(0xFF23, 0xBF); 
		MMUPtr->write(0xFF24, 0x77); 
		MMUPtr->write(0xFF25, 0xF3); 
		MMUPtr->write(0xFF26, 0xF1); 

		MMUPtr->write(0xFF40, 0x91); 
		MMUPtr->write(0xFF42, 0x00); 
		MMUPtr->write(0xFF43, 0x00); 
		MMUPtr->write(0xFF45, 0x00); 
		MMUPtr->write(0xFF47, 0xFC); 
		MMUPtr->write(0xFF48, 0xFF); 
		MMUPtr->write(0xFF49, 0xFF); 
		MMUPtr->write(0xFF4A, 0x00); 
		MMUPtr->write(0xFF4B, 0x00); 

		MMUPtr->write(0xFFFF, 0x00); 

		GPUPtr->preBoot();
	}

	CartridgePtr->loadROM(PathToCartridge);
}

uint8_t CPU::step()
{
	unsigned long cycles = 0x00;

	if (Halted)
		cycles = NOP(0x00);

	else
	{
		uint16_t addr = PC;
		uint8_t Operand = readBytePC();
		Instruction instruction;

		if (Operand == 0xCB)
		{
			Operand = readBytePC();
			instruction = InstructionCBMap[Operand];
		}
		else
			instruction = InstructionMap[Operand];


		if (instruction != nullptr)
			cycles = (this->*instruction)(Operand);

		else
			HALT(0x76);
	}

	Cycles += cycles;

	if (GPUPtr != nullptr)
		GPUPtr->step(cycles);

	if (TimerPtr != nullptr)
		TimerPtr->step(cycles);

	if (APUPtr != nullptr)
		APUPtr->step(cycles);

	handleInterrupts();
	return cycles;
}

void CPU::triggerInterrupt(uint8_t Interrupt)
{
	uint8_t IF = MMUPtr->read(0xFF0F);

	if		(Interrupt == INT40) setBit(IF, 0);
	else if (Interrupt == INT48) setBit(IF, 1);
	else if (Interrupt == INT50) setBit(IF, 2);
	else if (Interrupt == INT58) setBit(IF, 3);
	else if (Interrupt == INT60) setBit(IF, 4);

	Halted = false;

	MMUPtr->write(0xFF0F, IF);
}

uint8_t* CPU::getCurrentFrame()
{
	return GPUPtr->getCurrentFrame();
}

void CPU::setInput(uint8_t Input, uint8_t Buttons)
{
	JoypadPtr->setInput(Input, Buttons);
}

void CPU::setVSyncCallback(std::function<void()> Callback)
{
	GPUPtr->setVSyncCallback(Callback);
}

uint8_t CPU::getHighByte(uint16_t Dest)
{
	return ((Dest >> 8) & 0xFF);
}

uint8_t CPU::getLowByte(uint16_t Dest)
{
	return (Dest & 0xFF);
}

uint8_t* CPU::getByteRegister(uint8_t Value)
{
	return ByteRegisterMap[Value & 0x07];
}

uint16_t* CPU::getUShortRegister(uint8_t Value, bool UseAF)
{
	if ((Value & 0x03) == 0x03)
		return UseAF ? &AF : RegisterMap16Bit[0x03];

	else
		return RegisterMap16Bit[Value & 0x03];
}

void CPU::setHighByte(uint16_t* Dest, uint8_t Value)
{
	uint8_t low = getLowByte(*Dest);
	*Dest = (Value << 8) | low;
}

void CPU::setLowByte(uint16_t* Dest, uint8_t Value)
{
	uint8_t high = getHighByte(*Dest);
	*Dest = ((uint16_t)high << 8) | Value;
}

void CPU::setFlag(uint8_t Flag)
{
	auto TMP = getLowByte(AF);
	setBit(TMP, Flag);
	setLowByte(&AF, TMP);
}

void CPU::clearFlag(uint8_t Flag)
{
	auto TMP = getLowByte(AF);
	clearBit(TMP, Flag);
	setLowByte(&AF, TMP);
}

bool CPU::isFlagSet(uint8_t Flag)
{
	return isBitSet(getLowByte(AF), Flag);
}

void CPU::pushByteToSP(uint8_t Value)
{
	SP--;
	MMUPtr->write(SP, Value);
}

void CPU::pushUShortToSP(uint16_t Value)
{
	pushByteToSP(getHighByte(Value));
	pushByteToSP(getLowByte(Value));

}

uint16_t CPU::popUShort()
{
	uint16_t Value = MMUPtr->read2Bytes(SP);
	SP += 2;
	return Value;
}

uint8_t CPU::popByte()
{
	uint8_t Value = MMUPtr->read(SP);
	SP++;
	return Value;
}

uint8_t CPU::readBytePC()
{
	uint8_t Value = MMUPtr->read(PC);
	PC++;
	return Value;
}

uint16_t CPU::readUShortPC()
{
	uint16_t Value = MMUPtr->read2Bytes(PC);
	PC += 2;
	return Value;
}

uint8_t CPU::addByte(uint8_t Left, uint8_t Right)
{
	uint8_t Value = Left + Right;

	clearFlag(SubtractFlag);
	(Value == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	(((Value ^ Right ^ Left) & 0x10) == 0x10) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);
	(Value < Left) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	return Value;
}

uint16_t CPU::addUShort(uint16_t Left, uint16_t Right)
{
	uint16_t result = Left + Right;

	clearFlag(SubtractFlag);
	(result < Left) ? setFlag(CarryFlag) : clearFlag(CarryFlag);
	((result ^ Left ^ Right) & 0x1000) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);

	return result;
}

void CPU::ADC(uint8_t Value)
{
	uint8_t A = getHighByte(AF);
	uint8_t C = (isFlagSet(CarryFlag)) ? 0x01 : 0x00;

	clearFlag(SubtractFlag);

	if (((int)(A & 0x0F) + (int)(Value & 0x0F) + (int)C) > 0x0F)
	{
		setFlag(HalfCarryFlag);
	}
	else
	{
		clearFlag(HalfCarryFlag);
	}

	if (((int)(A & 0xFF) + (int)(Value & 0xFF) + (int)C) > 0xFF)
	{
		setFlag(CarryFlag);
	}
	else
	{
		clearFlag(CarryFlag);
	}

	uint8_t result = A + Value + C;
	setHighByte(&AF, result);

	(result == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);

}

void CPU::SBC(uint8_t Value)
{
	int un = (int)Value & 0xFF;
	int tmpa = (int)getHighByte(AF) & 0xFF;
	int ua = tmpa;

	ua -= un;

	if (isFlagSet(CarryFlag))
	{
		ua -= 1;
	}

	setFlag(SubtractFlag);
	(ua < 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	ua &= 0xFF;

	(ua == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);

	if (((ua ^ un ^ tmpa) & 0x10) == 0x10)
		setFlag(HalfCarryFlag);

	else
		clearFlag(HalfCarryFlag);

	setHighByte(&AF, (uint8_t)ua);
}

void CPU::handleInterrupts()
{
	if (IME == 0x01)
	{
		uint8_t IE = MMUPtr->read(0xFFFF);
		uint8_t IF = MMUPtr->read(0xFF0F);


		uint8_t activeInterrupts = ((IE & IF) & 0x0F);
		if (activeInterrupts > 0x00)
		{
			IME = 0x00; 

			pushUShortToSP(PC);

			if (isBitSet(activeInterrupts, 0))
			{
				PC = INT40;
				clearBit(IF, 0);
			}
			
			else if (isBitSet(activeInterrupts, 1))
			{
				PC = INT48;
				clearBit(IF, 1);
			}
			
			else if (isBitSet(activeInterrupts, 2))
			{
				PC = INT50;
				clearBit(IF, 2);
			}

			else if (isBitSet(activeInterrupts, 3))
			{
				PC = INT58;
				clearBit(IF, 3);
			}
			
			else if (isBitSet(activeInterrupts, 4))
			{
				PC = INT60;
				clearBit(IF, 4);
			}

			MMUPtr->write(0xFF0F, IF);
		}
	}
}

uint8_t CPU::NOP(const uint8_t& Operand)
{
	return 4;
}

uint8_t CPU::LDrn(const uint8_t& Operand)
{
	uint8_t n = readBytePC();
	uint8_t* r = getByteRegister(Operand >> 3);
	(*r) = n;

	return 8;
}

uint8_t CPU::LDrR(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand >> 3);
	uint8_t* R = getByteRegister(Operand);
	(*r) = *R;

	return 4;
}

uint8_t CPU::LDrrnn(const uint8_t& Operand)
{
	uint16_t* rr = getUShortRegister(Operand >> 4, false);
	uint16_t nn = readUShortPC(); // read nn
	(*rr) = nn;

	return 12;
}

uint8_t CPU::INCr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand >> 3);
	bool isBit3Before = isBitSet(*r, 3);
	*r += 1;
	bool isBit3After = isBitSet(*r, 3);

	if (*r == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);

	if (isBit3Before && !isBit3After)
	{
		setFlag(HalfCarryFlag);
	}
	else
	{
		clearFlag(HalfCarryFlag);
	}

	return 4;
}

uint8_t CPU::INCrr(const uint8_t& Operand)
{
	uint16_t* rr = getUShortRegister(Operand >> 4, false);
	*rr += 1;

	return 8;
}

uint8_t CPU::DECrr(const uint8_t& Operand)
{
	uint16_t* rr = getUShortRegister(Operand >> 4, false);
	*rr -= 1;

	return 8;
}

uint8_t CPU::ORr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	setHighByte(&AF, *r | getHighByte(AF));

	// Affects Z and clears NHC
	if (getHighByte(AF) == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 4;
}

uint8_t CPU::XORr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	setHighByte(&AF, *r ^ getHighByte(AF));

	// Affects Z and clears NHC
	if (getHighByte(AF) == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 4;
}

uint8_t CPU::PUSHrr(const uint8_t& Operand)
{
	uint16_t* rr = getUShortRegister(Operand >> 4, true);
	pushUShortToSP(*rr);

	return 16;
}

uint8_t CPU::POPrr(const uint8_t& Operand)
{
	uint16_t* rr = getUShortRegister(Operand >> 4, true);
	(*rr) = popUShort();

	if (((Operand >> 4) & 0x03) == 0x03)
	{
		(*rr) &= 0xFFF0;
	}

	return 12;
}

uint8_t CPU::DECr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand >> 3);
	uint8_t calc = (*r - 1);

	setFlag(SubtractFlag);
	(calc == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);

	if (((calc ^ 0x01 ^ *r) & 0x10) == 0x10)
	{
		setFlag(HalfCarryFlag);
	}
	else
	{
		clearFlag(HalfCarryFlag);
	}

	*r = calc;

	return 4;
}

uint8_t CPU::SUBr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	uint8_t A = getHighByte(AF);
	uint8_t result = A - (*r);
	setHighByte(&AF, result);

	(result == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	setFlag(SubtractFlag);
	((A & 0x0F) < ((*r) & 0x0F)) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);
	((A & 0xFF) < ((*r) & 0xFF)) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	return 4;
}

uint8_t CPU::SBCAr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	SBC(*r);
	return 4;
}

uint8_t CPU::CALLccnn(const uint8_t& Operand)
{
	uint16_t nn = readUShortPC();

	bool check = false;
	switch ((Operand >> 3) & 0x03)
	{
	case 0x00:  // NZ
		check = !isFlagSet(ZeroFlag);
		break;
	case 0x01:  // Z
		check = isFlagSet(ZeroFlag);
		break;
	case 0x02:  // NC
		check = !isFlagSet(CarryFlag);
		break;
	case 0x03:  // C
		check = isFlagSet(CarryFlag);
		break;
	}

	if (check)
	{
		pushUShortToSP(PC);
		PC = nn;
		return 24;
	}
	else
	{
		return 12;
	}
}

uint8_t CPU::LDr_HL_(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand >> 3);

	(*r) = MMUPtr->read(HL);

	return 8;
}

uint8_t CPU::LD_HL_r(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	MMUPtr->write(HL, (*r)); // Load r into the address pointed at by HL.

	return 8;
}

uint8_t CPU::RETcc(const uint8_t& Operand)
{
	bool check = false;
	switch ((Operand >> 3) & 0x03)
	{
	case 0x00:  // NZ
		check = !isFlagSet(ZeroFlag);
		break;
	case 0x01:  // Z
		check = isFlagSet(ZeroFlag);
		break;
	case 0x02:  // NC
		check = !isFlagSet(CarryFlag);
		break;
	case 0x03:  // C
		check = isFlagSet(CarryFlag);
		break;
	}

	if (check)
	{
		PC = popUShort();
		return 20;
	}
	else
	{
		return 8;
	}
}

uint8_t CPU::ADDHLss(const uint8_t& Operand)
{
	uint16_t* ss = getUShortRegister(Operand >> 4, false);

	HL = addUShort(HL, *ss);

	return 8;
}

uint8_t CPU::JPccnn(const uint8_t& Operand)
{
	uint16_t nn = readUShortPC();

	bool check = false;
	switch ((Operand >> 3) & 0x03)
	{
	case 0x00:  // NZ
		check = !isFlagSet(ZeroFlag);
		break;
	case 0x01:  // Z
		check = isFlagSet(ZeroFlag);
		break;
	case 0x02:  // NC
		check = !isFlagSet(CarryFlag);
		break;
	case 0x03:  // C
		check = isFlagSet(CarryFlag);
		break;
	}

	if (check)
	{
		PC = nn;
		return 16;
	}
	else
	{
		return 12;
	}
}

uint8_t CPU::ADDAr(const uint8_t& Operand)
{
	uint8_t A = getHighByte(AF);
	uint8_t* r = getByteRegister(Operand);

	setHighByte(&AF, addByte(A, *r));

	return 4;
}

uint8_t CPU::ADCAr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	ADC(*r);
	return 4;
}

uint8_t CPU::JRcce(const uint8_t& Operand)
{
	int8_t arg = static_cast<int8_t>(readBytePC());

	bool check = false;
	switch ((Operand >> 3) & 0x03)
	{
	case 0x00:  // NZ
		check = !isFlagSet(ZeroFlag);
		break;
	case 0x01:  // Z
		check = isFlagSet(ZeroFlag);
		break;
	case 0x02:  // NC
		check = !isFlagSet(CarryFlag);
		break;
	case 0x03:  // C
		check = isFlagSet(CarryFlag);
		break;
	}

	if (check)
	{
		PC += arg;
		return 12;
	}
	else
	{
		return 8;
	}
}

uint8_t CPU::ANDr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	uint8_t result = (*r) & getHighByte(AF);
	setHighByte(&AF, result);

	if (result == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);
	setFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 4;
}

uint8_t CPU::CPr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	uint8_t A = getHighByte(AF);
	uint8_t result = A - (*r);

	(result == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	setFlag(SubtractFlag);
	((A & 0xFF) < ((*r) & 0xFF)) ? setFlag(CarryFlag) : clearFlag(CarryFlag);
	((A & 0x0F) < ((*r) & 0x0F)) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);

	return 4;
}

uint8_t CPU::RSTn(const uint8_t& Operand)
{
	uint8_t t = ((Operand >> 3) & 0x07);

	pushUShortToSP(PC);
	PC = (uint16_t)(t * 0x08);
	return 16;
}

uint8_t CPU::LD_BC_A(const uint8_t& Operand)
{
	MMUPtr->write(BC, getHighByte(AF));
	return 8;
}

uint8_t CPU::RLCA(const uint8_t& Operand)
{
	uint8_t r = getHighByte(AF);

	isBitSet(r, 7) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r left
	r = r << 1;

	// Set bit 0 of r to the old CarryFlag
	isFlagSet(CarryFlag) ? setBit(r, 0) : clearBit(r, 0);

	setHighByte(&AF, r);

	// Clear sZ, clears N, clears H, affects C
	clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 4;
}

uint8_t CPU::LD_nn_SP(const uint8_t& Operand)
{
	uint16_t nn = readUShortPC();

	// Load A into (nn)
	MMUPtr->write(nn + 1, getHighByte(SP));
	MMUPtr->write(nn, getLowByte(SP));

	return 20;
}

uint8_t CPU::LDA_BC_(const uint8_t& Operand)
{
	uint8_t val = MMUPtr->read(BC);
	setHighByte(&AF, val);

	return 8;
}

uint8_t CPU::RRCA(const uint8_t& Operand)
{
	uint8_t A = getHighByte(AF);
	bool carry = isBitSet(A, 0);

	A = A >> 1;

	if (carry)
	{
		setFlag(CarryFlag);
		setBit(A, 7);
	}
	else
	{
		clearFlag(CarryFlag);
		clearBit(A, 7);
	}

	setHighByte(&AF, A);

	clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 4;
}

uint8_t CPU::STOP(const uint8_t& Operand)
{
	return HALT(Operand);
}

uint8_t CPU::LD_DE_A(const uint8_t& Operand)
{
	MMUPtr->write(DE, getHighByte(AF));
	return 8;
}

uint8_t CPU::RLA(const uint8_t& Operand)
{
	bool carry = isFlagSet(CarryFlag);

	// Grab bit 7 and store it in the carryflag
	isBitSet(getHighByte(AF), 7) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift A left
	setHighByte(&AF, getHighByte(AF) << 1);

	// Set bit 0 of A to the old CarryFlag
	auto TMP = getHighByte(AF);
	carry ? setBit(TMP, 0) : clearBit(TMP, 0);
	setHighByte(&AF, TMP);

	// Clears Z, clears N, clears H, affects C
	clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 4;
}

uint8_t CPU::JRe(const uint8_t& Operand)
{
	int8_t e = static_cast<int8_t>(readBytePC());

	PC += e;
	return 12;
}

uint8_t CPU::LDA_DE_(const uint8_t& Operand)
{
	uint8_t val = MMUPtr->read(DE);
	setHighByte(&AF, val);
	return 8;
}

uint8_t CPU::RRA(const uint8_t& Operand)
{
	bool carry = isFlagSet(CarryFlag);

	// Grab bit 0 and store it in the carryflag
	isBitSet(getHighByte(AF), 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift A right
	setHighByte(&AF, getHighByte(AF) >> 1);

	// Set bit 7 of A to the old CarryFlag
	auto TMP = getHighByte(AF);
	carry ? setBit(TMP, 7) : clearBit(TMP, 7);
	setHighByte(&AF, TMP);

	// Affects Z, clears N, clears H, affects C
	clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 4;
}

uint8_t CPU::LDI_HL_A(const uint8_t& Operand)
{
	MMUPtr->write(HL, getHighByte(AF)); // Load A into the address pointed at by HL.

	HL++;

	return 8;
}

uint8_t CPU::DAA(const uint8_t& Operand)
{
	int aVal = getHighByte(AF);

	if (!isFlagSet(SubtractFlag))
	{
		if (isFlagSet(HalfCarryFlag) || (aVal & 0xF) > 9)
		{
			aVal += 0x06;
		}

		if (isFlagSet(CarryFlag) || (aVal > 0x9F))
		{
			aVal += 0x60;
		}
	}
	else
	{
		if (isFlagSet(HalfCarryFlag))
		{
			aVal = (aVal - 0x06) & 0xFF;
		}

		if (isFlagSet(CarryFlag))
		{
			aVal -= 0x60;
		}
	}

	clearFlag(HalfCarryFlag);

	if ((aVal & 0x100) == 0x100)
	{
		setFlag(CarryFlag);
	}

	aVal &= 0xFF;

	(aVal == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	setHighByte(&AF, (uint8_t)aVal);

	return 4;
}

uint8_t CPU::LDIA_HL_(const uint8_t& Operand)
{
	setHighByte(&AF, MMUPtr->read(HL));
	HL++;

	return 8;
}

uint8_t CPU::CPL(const uint8_t& Operand)
{
	uint8_t A = getHighByte(AF);
	uint8_t result = A ^ 0xFF;
	setHighByte(&AF, result);

	setFlag(SubtractFlag);
	setFlag(HalfCarryFlag);

	return 4;
}

uint8_t CPU::LDD_HL_A(const uint8_t& Operand)
{
	MMUPtr->write(HL, getHighByte(AF));

	HL--;
	return 8;
}

uint8_t CPU::INC_HL_(const uint8_t& Operand)
{
	uint8_t LocalHL = MMUPtr->read(HL);
	bool isBit3Before = isBitSet(LocalHL, 3);
	LocalHL += 1;
	bool isBit3After = isBitSet(LocalHL, 3);

	MMUPtr->write(HL, LocalHL);

	if (LocalHL == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);

	if (isBit3Before && !isBit3After)
	{
		setFlag(HalfCarryFlag);
	}
	else
	{
		clearFlag(HalfCarryFlag);
	}

	return 12;
}

uint8_t CPU::DEC_HL_(const uint8_t& Operand)
{
	uint8_t val = MMUPtr->read(HL);
	uint8_t calc = (val - 1);

	setFlag(SubtractFlag);
	(calc == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);

	if (((calc ^ 0x01 ^ val) & 0x10) == 0x10)
	{
		setFlag(HalfCarryFlag);
	}
	else
	{
		clearFlag(HalfCarryFlag);
	}

	MMUPtr->write(HL, calc);

	return 12;
}

uint8_t CPU::LD_HL_n(const uint8_t& Operand)
{
	uint8_t n = readBytePC();
	MMUPtr->write(HL, n); // Load n into the address pointed at by HL.

	return 12;
}

uint8_t CPU::SCF(const uint8_t& Operand)
{
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	setFlag(CarryFlag);

	return 4;
}

uint8_t CPU::LDDA_HL_(const uint8_t& Operand)
{
	uint8_t LocalHL = MMUPtr->read(HL);
	setHighByte(&AF, LocalHL);

	HL--;
	return 8;
}

uint8_t CPU::CCF(const uint8_t& Operand)
{
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	isFlagSet(CarryFlag) ? clearFlag(CarryFlag) : setFlag(CarryFlag);

	return 4;
}

uint8_t CPU::HALT(const uint8_t& Operand)
{
	Halted = true;
	return 0;
}

uint8_t CPU::ADDA_HL_(const uint8_t& Operand)
{
	uint8_t A = getHighByte(AF);
	uint8_t LocalHL = MMUPtr->read(HL);
	setHighByte(&AF, addByte(A, LocalHL));

	return 8;
}

uint8_t CPU::ADCA_HL_(const uint8_t& Operand)
{
	uint8_t LocalHL = MMUPtr->read(HL);
	ADC(LocalHL);
	return 8;
}

uint8_t CPU::SUB_HL_(const uint8_t& Operand)
{
	uint8_t A = getHighByte(AF);
	uint8_t LocalHL = MMUPtr->read(HL);
	uint8_t result = A - LocalHL;
	setHighByte(&AF, result);

	(result == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	setFlag(SubtractFlag);
	((A & 0x0F) < (result & 0x0F)) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);
	((A & 0xFF) < (result & 0xFF)) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::SBCA_HL_(const uint8_t& Operand)
{
	uint8_t LocalHL = MMUPtr->read(HL);
	SBC(LocalHL);
	return 8;
}

uint8_t CPU::AND_HL_(const uint8_t& Operand)
{
	uint8_t LocalHL = MMUPtr->read(HL);
	uint8_t result = LocalHL & getHighByte(AF);
	setHighByte(&AF, result);

	if (result == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);
	setFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::XOR_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);
	setHighByte(&AF, r ^ getHighByte(AF));

	// Affects Z and clears NHC
	if (getHighByte(AF) == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::OR_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);
	setHighByte(&AF, r | getHighByte(AF));

	// Affects Z and clears NHC
	if (getHighByte(AF) == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::CP_HL_(const uint8_t& Operand)
{
	uint8_t LocalHL = MMUPtr->read(HL);
	uint8_t A = getHighByte(AF);
	uint8_t result = A - LocalHL;

	(result == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	setFlag(SubtractFlag);
	((A & 0x0F) < (LocalHL & 0x0F)) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);
	((A & 0xFF) < (LocalHL & 0xFF)) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::JPnn(const uint8_t& Operand)
{
	uint16_t nn = readUShortPC();
	PC = nn;

	return 16;
}

uint8_t CPU::ADDAn(const uint8_t& Operand)
{
	uint8_t n = readBytePC();
	uint8_t A = getHighByte(AF);

	setHighByte(&AF, addByte(A, n));

	return 8;;
}

uint8_t CPU::RET(const uint8_t& Operand)
{
	PC = popUShort();

	return 16;
}

uint8_t CPU::CALLnn(const uint8_t& Operand)
{
	uint16_t nn = readUShortPC(); // read nn
	pushUShortToSP(PC); // Push PC to SP
	PC = nn; // Set the PC to the target address

	return 24;
}

uint8_t CPU::ADCAn(const uint8_t& Operand)
{
	ADC(readBytePC());
	return 8;
}

uint8_t CPU::SUBn(const uint8_t& Operand)
{
	uint8_t n = readBytePC();
	uint8_t A = getHighByte(AF);
	uint8_t result = A - n;
	setHighByte(&AF, result);

	(result == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	setFlag(SubtractFlag);
	((A & 0x0F) < (n & 0x0F)) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);
	((A & 0xFF) < (n & 0xFF)) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::RETI(const uint8_t& Operand)
{
	IME = 0x01; // Restore interrupts
	PC = popUShort(); // Return

	return 16;
}

uint8_t CPU::SBCAn(const uint8_t& Operand)
{
	uint8_t n = readBytePC();
	SBC(n);
	return 8;
}

uint8_t CPU::LD_0xFF00n_A(const uint8_t& Operand)
{
	uint8_t n = readBytePC(); // read n

	MMUPtr->write(0xFF00 + n, getHighByte(AF)); // Load A into 0xFF00 + n

	return 12;
}

uint8_t CPU::LD_0xFF00C_A(const uint8_t& Operand)
{
	MMUPtr->write(0xFF00 + getLowByte(BC), getHighByte(AF)); // Load A into 0xFF00 + C

	return 8;
}

uint8_t CPU::ANDn(const uint8_t& Operand)
{
	uint8_t n = readBytePC();

	uint8_t result = getHighByte(AF) & n;
	setHighByte(&AF, result);

	(result == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	setFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::ADDSPdd(const uint8_t& Operand)
{
	int8_t arg = static_cast<int8_t>(readBytePC());
	uint16_t result = (SP + arg);

	clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	((result & 0xF) < (SP & 0xF)) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);
	((result & 0xFF) < (SP & 0xFF)) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	SP = result;

	return 16;
}

uint8_t CPU::JP_HL_(const uint8_t& Operand)
{
	PC = HL;
	return 4;
}

uint8_t CPU::LD_nn_A(const uint8_t& Operand)
{
	uint16_t nn = readUShortPC();

	MMUPtr->write(nn, getHighByte(AF)); // Load A into (nn)

	return 16;
}

uint8_t CPU::XORn(const uint8_t& Operand)
{
	uint8_t n = readBytePC();
	setHighByte(&AF, n ^ getHighByte(AF));

	// Affects Z and clears NHC
	if (getHighByte(AF) == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::LDA_0xFF00n_(const uint8_t& Operand)
{
	uint8_t n = readBytePC(); // read n
	setHighByte(&AF, MMUPtr->read(0xFF00 + n));

	return 12;
}

uint8_t CPU::LDA_0xFF00C_(const uint8_t& Operand)
{
	setHighByte(&AF, MMUPtr->read(0xFF00 + getLowByte(BC)));

	return 8;
}

uint8_t CPU::DI(const uint8_t& Operand)
{
	IME = 0x00;

	return 4;
}

uint8_t CPU::ORn(const uint8_t& Operand)
{
	uint8_t n = readBytePC();
	setHighByte(&AF, n | getHighByte(AF));

	// Affects Z and clears NHC
	if (getHighByte(AF) == 0x00)
	{
		setFlag(ZeroFlag);
	}
	else
	{
		clearFlag(ZeroFlag);
	}

	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::LDHLSPe(const uint8_t& Operand)
{
	int8_t e = static_cast<int8_t>(readBytePC());

	uint16_t result = SP + e;

	uint16_t check = SP ^ e ^ ((SP + e) & 0xFFFF);

	((check & 0x100) == 0x100) ? setFlag(CarryFlag) : clearFlag(CarryFlag);
	((check & 0x10) == 0x10) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);

	clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);

	HL = result;

	return 12;
}

uint8_t CPU::LDSPHL(const uint8_t& Operand)
{
	SP = HL;

	return 8;
}

uint8_t CPU::LDA_nn_(const uint8_t& Operand)
{
	uint16_t nn = readUShortPC();
	setHighByte(&AF, MMUPtr->read(nn));

	return 16;
}

uint8_t CPU::EI(const uint8_t& Operand)
{
	IME = 0x01;

	return 4;
}

uint8_t CPU::CPn(const uint8_t& Operand)
{
	uint8_t n = readBytePC();
	uint8_t A = getHighByte(AF);
	uint8_t result = A - n;

	(result == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	((A & 0xFF) < (n & 0xFF)) ? setFlag(CarryFlag) : clearFlag(CarryFlag);
	((A & 0x0F) < (n & 0x0F)) ? setFlag(HalfCarryFlag) : clearFlag(HalfCarryFlag);
	setFlag(SubtractFlag);

	return 8;
}

uint8_t CPU::RLCr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);

	// Grab bit 7 and store it in the carryflag
	isBitSet(*r, 7) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r left
	(*r) = *r << 1;

	// Set bit 0 of r to the old CarryFlag
	isFlagSet(CarryFlag) ? setBit((*r), 0) : clearBit((*r), 0);

	// Affects Z, clears N, clears H, affects C
	(*r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 8;
}

uint8_t CPU::RLC_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);

	// Grab bit 7 and store it in the carryflag
	isBitSet(r, 7) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r left
	r <<= 1;

	// Set bit 0 of r to the old CarryFlag
	isFlagSet(CarryFlag) ? setBit((r), 0) : clearBit((r), 0);

	MMUPtr->write(HL, r);

	// Affects Z, clears N, clears H, affects C
	(r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 16;
}

uint8_t CPU::RRCr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);

	// Grab bit 0 and store it in the carryflag
	isBitSet(*r, 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r right
	(*r) = *r >> 1;

	// Set bit 0 of r to the old CarryFlag
	isFlagSet(CarryFlag) ? setBit((*r), 7) : clearBit((*r), 7);

	// Affects Z, clears N, clears H, affects C
	(*r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 8;
}

uint8_t CPU::RRC_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);

	// Grab bit 0 and store it in the carryflag
	isBitSet(r, 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r right
	r >>= 1;

	// Set bit 0 of r to the old CarryFlag
	isFlagSet(CarryFlag) ? setBit((r), 7) : clearBit((r), 7);

	MMUPtr->write(HL, r);

	// Affects Z, clears N, clears H, affects C
	(r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 16;
}

uint8_t CPU::RLr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);

	// Grab the current CarryFlag val
	bool carry = isFlagSet(CarryFlag);

	// Grab bit 7 and store it in the carryflag
	isBitSet(*r, 7) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r left
	(*r) = *r << 1;

	// Set bit 0 of r to the old CarryFlag
	carry ? setBit((*r), 0) : clearBit((*r), 0);

	// Affects Z, clears N, clears H, affects C
	(*r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 8;
}

uint8_t CPU::RL_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);

	// Grab the current CarryFlag val
	bool carry = isFlagSet(CarryFlag);

	// Grab bit 7 and store it in the carryflag
	isBitSet(r, 7) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r left
	r <<= 1;

	// Set bit 0 of r to the old CarryFlag
	carry ? setBit((r), 0) : clearBit((r), 0);

	MMUPtr->write(HL, r);

	// Affects Z, clears N, clears H, affects C
	(r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 16;
}

uint8_t CPU::RRr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);

	// Grab the current CarryFlag val
	bool carry = isFlagSet(CarryFlag);

	// Grab bit 0 and store it in the carryflag
	isBitSet(*r, 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r right
	(*r) = *r >> 1;

	// Set bit 7 of r to the old CarryFlag
	carry ? setBit((*r), 7) : clearBit((*r), 7);

	// Affects Z, clears N, clears H, affects C
	(*r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 8;
}

uint8_t CPU::RR_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);

	// Grab the current CarryFlag val
	bool carry = isFlagSet(CarryFlag);

	// Grab bit 0 and store it in the carryflag
	isBitSet(r, 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r right
	r >>= 1;

	// Set bit 7 of r to the old CarryFlag
	carry ? setBit((r), 7) : clearBit((r), 7);

	MMUPtr->write(HL, r);

	// Affects Z, clears N, clears H, affects C
	(r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 16;
}

uint8_t CPU::SLAr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);

	// Grab bit 7 and store it in the carryflag
	isBitSet(*r, 7) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r left
	(*r) = *r << 1;

	// Affects Z, clears N, clears H, affects C
	(*r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 8;
}

uint8_t CPU::SLA_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);

	// Grab bit 7 and store it in the carryflag
	isBitSet(r, 7) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r left
	r = r << 1;
	MMUPtr->write(HL, r);

	// Affects Z, clears N, clears H, affects C
	(r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 16;
}

uint8_t CPU::SRLr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);

	// Grab bit 0 and store it in the carryflag
	isBitSet(*r, 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r right
	(*r) = *r >> 1;
	clearBit(*r, 7);

	// Affects Z, clears N, clears H, affects C
	(*r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 8;
}

uint8_t CPU::SRL_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);

	// Grab bit 0 and store it in the carryflag
	isBitSet(r, 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r right
	r = r >> 1;
	clearBit(r, 7);
	MMUPtr->write(HL, r);

	// Affects Z, clears N, clears H, affects C
	(r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 16;
}

uint8_t CPU::SRAr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);

	// Grab bit 0 and store it in the carryflag
	isBitSet(*r, 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r right
	(*r) = (*r >> 1) | (*r & 0x80);

	// Affects Z, clears N, clears H, affects C
	(*r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 8;
}

uint8_t CPU::SRA_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);

	// Grab bit 0 and store it in the carryflag
	isBitSet(r, 0) ? setFlag(CarryFlag) : clearFlag(CarryFlag);

	// Shift r right
	r = (r >> 1) | (r & 0x80);
	MMUPtr->write(HL, r);

	// Affects Z, clears N, clears H, affects C
	(r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);

	return 16;
}

uint8_t CPU::BITbr(const uint8_t& Operand)
{
	uint8_t bit = (Operand >> 3) & 0x07;
	uint8_t* r = getByteRegister(Operand);

	// Test bit b in r
	(!isBitSet(*r, bit)) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);

	setFlag(HalfCarryFlag); // H is set
	clearFlag(SubtractFlag); // N is reset

	return 8;
}

uint8_t CPU::BITb_HL_(const uint8_t& Operand)
{
	uint8_t bit = (Operand >> 3) & 0x07;
	uint8_t r = MMUPtr->read(HL);

	// Test bit b in r
	(!isBitSet(r, bit)) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);

	setFlag(HalfCarryFlag); // H is set
	clearFlag(SubtractFlag); // N is reset

	return 12;
}

uint8_t CPU::RESbr(const uint8_t& Operand)
{

	uint8_t bit = (Operand >> 3) & 0x07;
	uint8_t* r = getByteRegister(Operand);
	clearBit(*r, bit);

	return 8;
}

uint8_t CPU::RESb_HL_(const uint8_t& Operand)
{
	uint8_t bit = (Operand >> 3) & 0x07;
	uint8_t r = MMUPtr->read(HL);
	auto TMP = r;
	clearBit(TMP, bit);
	MMUPtr->write(HL, TMP);

	return 16;
}

uint8_t CPU::SETbr(const uint8_t& Operand)
{
	uint8_t bit = (Operand >> 3) & 0x07;
	uint8_t* r = getByteRegister(Operand);
	setBit(*r, bit);

	return 8;
}

uint8_t CPU::SETb_HL_(const uint8_t& Operand)
{
	uint8_t bit = (Operand >> 3) & 0x07;
	uint8_t r = MMUPtr->read(HL);
	auto TMP = r;
	setBit(TMP, bit);
	MMUPtr->write(HL, TMP);

	return 16;
}

uint8_t CPU::SWAPr(const uint8_t& Operand)
{
	uint8_t* r = getByteRegister(Operand);
	uint8_t lowNibble = (*r & 0x0F);
	uint8_t highNibble = (*r & 0xF0);

	*r = (lowNibble << 4) | (highNibble >> 4);

	((*r) == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 8;
}

uint8_t CPU::SWAP_HL_(const uint8_t& Operand)
{
	uint8_t r = MMUPtr->read(HL);
	uint8_t lowNibble = (r & 0x0F);
	uint8_t highNibble = (r & 0xF0);

	MMUPtr->write(HL, (lowNibble << 4) | (highNibble >> 4));

	(r == 0x00) ? setFlag(ZeroFlag) : clearFlag(ZeroFlag);
	clearFlag(SubtractFlag);
	clearFlag(HalfCarryFlag);
	clearFlag(CarryFlag);

	return 16;
}

void CPU::setupInstructionMap()
{
	InstructionMap[0x00] = &CPU::NOP;
	InstructionMap[0x01] = &CPU::LDrrnn;
	InstructionMap[0x02] = &CPU::LD_BC_A;
	InstructionMap[0x03] = &CPU::INCrr;
	InstructionMap[0x04] = &CPU::INCr;
	InstructionMap[0x05] = &CPU::DECr;
	InstructionMap[0x06] = &CPU::LDrn;
	InstructionMap[0x07] = &CPU::RLCA;
	InstructionMap[0x08] = &CPU::LD_nn_SP;
	InstructionMap[0x09] = &CPU::ADDHLss;
	InstructionMap[0x0A] = &CPU::LDA_BC_;
	InstructionMap[0x0B] = &CPU::DECrr;
	InstructionMap[0x0C] = &CPU::INCr;
	InstructionMap[0x0D] = &CPU::DECr;
	InstructionMap[0x0E] = &CPU::LDrn;
	InstructionMap[0x0F] = &CPU::RRCA;
	InstructionMap[0x10] = &CPU::STOP;
	InstructionMap[0x11] = &CPU::LDrrnn;
	InstructionMap[0x12] = &CPU::LD_DE_A;
	InstructionMap[0x13] = &CPU::INCrr;
	InstructionMap[0x14] = &CPU::INCr;
	InstructionMap[0x15] = &CPU::DECr;
	InstructionMap[0x16] = &CPU::LDrn;
	InstructionMap[0x17] = &CPU::RLA;
	InstructionMap[0x18] = &CPU::JRe;
	InstructionMap[0x19] = &CPU::ADDHLss;
	InstructionMap[0x1A] = &CPU::LDA_DE_;
	InstructionMap[0x1B] = &CPU::DECrr;
	InstructionMap[0x1C] = &CPU::INCr;
	InstructionMap[0x1D] = &CPU::DECr;
	InstructionMap[0x1E] = &CPU::LDrn;
	InstructionMap[0x1F] = &CPU::RRA;
	InstructionMap[0x20] = &CPU::JRcce;
	InstructionMap[0x21] = &CPU::LDrrnn;
	InstructionMap[0x22] = &CPU::LDI_HL_A;
	InstructionMap[0x23] = &CPU::INCrr;
	InstructionMap[0x24] = &CPU::INCr;
	InstructionMap[0x25] = &CPU::DECr;
	InstructionMap[0x26] = &CPU::LDrn;
	InstructionMap[0x27] = &CPU::DAA;
	InstructionMap[0x28] = &CPU::JRcce;
	InstructionMap[0x29] = &CPU::ADDHLss;
	InstructionMap[0x2A] = &CPU::LDIA_HL_;
	InstructionMap[0x2B] = &CPU::DECrr;
	InstructionMap[0x2C] = &CPU::INCr;
	InstructionMap[0x2D] = &CPU::DECr;
	InstructionMap[0x2E] = &CPU::LDrn;
	InstructionMap[0x2F] = &CPU::CPL;
	InstructionMap[0x30] = &CPU::JRcce;
	InstructionMap[0x31] = &CPU::LDrrnn;
	InstructionMap[0x32] = &CPU::LDD_HL_A;
	InstructionMap[0x33] = &CPU::INCrr;
	InstructionMap[0x34] = &CPU::INC_HL_;
	InstructionMap[0x35] = &CPU::DEC_HL_;
	InstructionMap[0x36] = &CPU::LD_HL_n;
	InstructionMap[0x37] = &CPU::SCF;
	InstructionMap[0x38] = &CPU::JRcce;
	InstructionMap[0x39] = &CPU::ADDHLss;
	InstructionMap[0x3A] = &CPU::LDDA_HL_;
	InstructionMap[0x3B] = &CPU::DECrr;
	InstructionMap[0x3C] = &CPU::INCr;
	InstructionMap[0x3D] = &CPU::DECr;
	InstructionMap[0x3E] = &CPU::LDrn;
	InstructionMap[0x3F] = &CPU::CCF;
	InstructionMap[0x40] = &CPU::LDrR;
	InstructionMap[0x41] = &CPU::LDrR;
	InstructionMap[0x42] = &CPU::LDrR;
	InstructionMap[0x43] = &CPU::LDrR;
	InstructionMap[0x44] = &CPU::LDrR;
	InstructionMap[0x45] = &CPU::LDrR;
	InstructionMap[0x46] = &CPU::LDr_HL_;
	InstructionMap[0x47] = &CPU::LDrR;
	InstructionMap[0x48] = &CPU::LDrR;
	InstructionMap[0x49] = &CPU::LDrR;
	InstructionMap[0x4A] = &CPU::LDrR;
	InstructionMap[0x4B] = &CPU::LDrR;
	InstructionMap[0x4C] = &CPU::LDrR;
	InstructionMap[0x4D] = &CPU::LDrR;
	InstructionMap[0x4E] = &CPU::LDr_HL_;
	InstructionMap[0x4F] = &CPU::LDrR;
	InstructionMap[0x50] = &CPU::LDrR;
	InstructionMap[0x51] = &CPU::LDrR;
	InstructionMap[0x52] = &CPU::LDrR;
	InstructionMap[0x53] = &CPU::LDrR;
	InstructionMap[0x54] = &CPU::LDrR;
	InstructionMap[0x55] = &CPU::LDrR;
	InstructionMap[0x56] = &CPU::LDr_HL_;
	InstructionMap[0x57] = &CPU::LDrR;
	InstructionMap[0x58] = &CPU::LDrR;
	InstructionMap[0x59] = &CPU::LDrR;
	InstructionMap[0x5A] = &CPU::LDrR;
	InstructionMap[0x5B] = &CPU::LDrR;
	InstructionMap[0x5C] = &CPU::LDrR;
	InstructionMap[0x5D] = &CPU::LDrR;
	InstructionMap[0x5E] = &CPU::LDr_HL_;
	InstructionMap[0x5F] = &CPU::LDrR;
	InstructionMap[0x60] = &CPU::LDrR;
	InstructionMap[0x61] = &CPU::LDrR;
	InstructionMap[0x62] = &CPU::LDrR;
	InstructionMap[0x63] = &CPU::LDrR;
	InstructionMap[0x64] = &CPU::LDrR;
	InstructionMap[0x65] = &CPU::LDrR;
	InstructionMap[0x66] = &CPU::LDr_HL_;
	InstructionMap[0x67] = &CPU::LDrR;
	InstructionMap[0x68] = &CPU::LDrR;
	InstructionMap[0x69] = &CPU::LDrR;
	InstructionMap[0x6A] = &CPU::LDrR;
	InstructionMap[0x6B] = &CPU::LDrR;
	InstructionMap[0x6C] = &CPU::LDrR;
	InstructionMap[0x6D] = &CPU::LDrR;
	InstructionMap[0x6E] = &CPU::LDr_HL_;
	InstructionMap[0x6F] = &CPU::LDrR;
	InstructionMap[0x70] = &CPU::LD_HL_r;
	InstructionMap[0x71] = &CPU::LD_HL_r;
	InstructionMap[0x72] = &CPU::LD_HL_r;
	InstructionMap[0x73] = &CPU::LD_HL_r;
	InstructionMap[0x74] = &CPU::LD_HL_r;
	InstructionMap[0x75] = &CPU::LD_HL_r;
	InstructionMap[0x76] = &CPU::HALT;
	InstructionMap[0x77] = &CPU::LD_HL_r;
	InstructionMap[0x78] = &CPU::LDrR;
	InstructionMap[0x79] = &CPU::LDrR;
	InstructionMap[0x7A] = &CPU::LDrR;
	InstructionMap[0x7B] = &CPU::LDrR;
	InstructionMap[0x7C] = &CPU::LDrR;
	InstructionMap[0x7D] = &CPU::LDrR;
	InstructionMap[0x7E] = &CPU::LDr_HL_;
	InstructionMap[0x7F] = &CPU::LDrR;
	InstructionMap[0x80] = &CPU::ADDAr;
	InstructionMap[0x81] = &CPU::ADDAr;
	InstructionMap[0x82] = &CPU::ADDAr;
	InstructionMap[0x83] = &CPU::ADDAr;
	InstructionMap[0x84] = &CPU::ADDAr;
	InstructionMap[0x85] = &CPU::ADDAr;
	InstructionMap[0x86] = &CPU::ADDA_HL_;
	InstructionMap[0x87] = &CPU::ADDAr;
	InstructionMap[0x88] = &CPU::ADCAr;
	InstructionMap[0x89] = &CPU::ADCAr;
	InstructionMap[0x8A] = &CPU::ADCAr;
	InstructionMap[0x8B] = &CPU::ADCAr;
	InstructionMap[0x8C] = &CPU::ADCAr;
	InstructionMap[0x8D] = &CPU::ADCAr;
	InstructionMap[0x8E] = &CPU::ADCA_HL_;
	InstructionMap[0x8F] = &CPU::ADCAr;
	InstructionMap[0x90] = &CPU::SUBr;
	InstructionMap[0x91] = &CPU::SUBr;
	InstructionMap[0x92] = &CPU::SUBr;
	InstructionMap[0x93] = &CPU::SUBr;
	InstructionMap[0x94] = &CPU::SUBr;
	InstructionMap[0x95] = &CPU::SUBr;
	InstructionMap[0x96] = &CPU::SUB_HL_;
	InstructionMap[0x97] = &CPU::SUBr;
	InstructionMap[0x98] = &CPU::SBCAr;
	InstructionMap[0x99] = &CPU::SBCAr;
	InstructionMap[0x9A] = &CPU::SBCAr;
	InstructionMap[0x9B] = &CPU::SBCAr;
	InstructionMap[0x9C] = &CPU::SBCAr;
	InstructionMap[0x9D] = &CPU::SBCAr;
	InstructionMap[0x9E] = &CPU::SBCA_HL_;
	InstructionMap[0x9F] = &CPU::SBCAr;
	InstructionMap[0xA0] = &CPU::ANDr;
	InstructionMap[0xA1] = &CPU::ANDr;
	InstructionMap[0xA2] = &CPU::ANDr;
	InstructionMap[0xA3] = &CPU::ANDr;
	InstructionMap[0xA4] = &CPU::ANDr;
	InstructionMap[0xA5] = &CPU::ANDr;
	InstructionMap[0xA6] = &CPU::AND_HL_;
	InstructionMap[0xA7] = &CPU::ANDr;
	InstructionMap[0xA8] = &CPU::XORr;
	InstructionMap[0xA9] = &CPU::XORr;
	InstructionMap[0xAA] = &CPU::XORr;
	InstructionMap[0xAB] = &CPU::XORr;
	InstructionMap[0xAC] = &CPU::XORr;
	InstructionMap[0xAD] = &CPU::XORr;
	InstructionMap[0xAE] = &CPU::XOR_HL_;
	InstructionMap[0xAF] = &CPU::XORr;
	InstructionMap[0xB0] = &CPU::ORr;
	InstructionMap[0xB1] = &CPU::ORr;
	InstructionMap[0xB2] = &CPU::ORr;
	InstructionMap[0xB3] = &CPU::ORr;
	InstructionMap[0xB4] = &CPU::ORr;
	InstructionMap[0xB5] = &CPU::ORr;
	InstructionMap[0xB6] = &CPU::OR_HL_;
	InstructionMap[0xB7] = &CPU::ORr;
	InstructionMap[0xB8] = &CPU::CPr;
	InstructionMap[0xB9] = &CPU::CPr;
	InstructionMap[0xBA] = &CPU::CPr;
	InstructionMap[0xBB] = &CPU::CPr;
	InstructionMap[0xBC] = &CPU::CPr;
	InstructionMap[0xBD] = &CPU::CPr;
	InstructionMap[0xBE] = &CPU::CP_HL_;
	InstructionMap[0xBF] = &CPU::CPr;
	InstructionMap[0xC0] = &CPU::RETcc;
	InstructionMap[0xC1] = &CPU::POPrr;
	InstructionMap[0xC2] = &CPU::JPccnn;
	InstructionMap[0xC3] = &CPU::JPnn;
	InstructionMap[0xC4] = &CPU::CALLccnn;
	InstructionMap[0xC5] = &CPU::PUSHrr;
	InstructionMap[0xC6] = &CPU::ADDAn;
	InstructionMap[0xC7] = &CPU::RSTn;
	InstructionMap[0xC8] = &CPU::RETcc;
	InstructionMap[0xC9] = &CPU::RET;
	InstructionMap[0xCA] = &CPU::JPccnn;
	InstructionMap[0xCC] = &CPU::CALLccnn;
	InstructionMap[0xCD] = &CPU::CALLnn;
	InstructionMap[0xCE] = &CPU::ADCAn;
	InstructionMap[0xCF] = &CPU::RSTn;
	InstructionMap[0xD0] = &CPU::RETcc;
	InstructionMap[0xD1] = &CPU::POPrr;
	InstructionMap[0xD2] = &CPU::JPccnn;
	InstructionMap[0xD4] = &CPU::CALLccnn;
	InstructionMap[0xD5] = &CPU::PUSHrr;
	InstructionMap[0xD6] = &CPU::SUBn;
	InstructionMap[0xD7] = &CPU::RSTn;
	InstructionMap[0xD8] = &CPU::RETcc;
	InstructionMap[0xD9] = &CPU::RETI;
	InstructionMap[0xDA] = &CPU::JPccnn;
	InstructionMap[0xDC] = &CPU::CALLccnn;
	InstructionMap[0xDE] = &CPU::SBCAn;
	InstructionMap[0xDF] = &CPU::RSTn;
	InstructionMap[0xE0] = &CPU::LD_0xFF00n_A;
	InstructionMap[0xE1] = &CPU::POPrr;
	InstructionMap[0xE2] = &CPU::LD_0xFF00C_A;
	InstructionMap[0xE5] = &CPU::PUSHrr;
	InstructionMap[0xE6] = &CPU::ANDn;
	InstructionMap[0xE7] = &CPU::RSTn;
	InstructionMap[0xE8] = &CPU::ADDSPdd;
	InstructionMap[0xE9] = &CPU::JP_HL_;
	InstructionMap[0xEA] = &CPU::LD_nn_A;
	InstructionMap[0xEE] = &CPU::XORn;
	InstructionMap[0xEF] = &CPU::RSTn;
	InstructionMap[0xF0] = &CPU::LDA_0xFF00n_;
	InstructionMap[0xF1] = &CPU::POPrr;
	InstructionMap[0xF2] = &CPU::LDA_0xFF00C_;
	InstructionMap[0xF3] = &CPU::DI;
	InstructionMap[0xF5] = &CPU::PUSHrr;
	InstructionMap[0xF6] = &CPU::ORn;
	InstructionMap[0xF7] = &CPU::RSTn;
	InstructionMap[0xF8] = &CPU::LDHLSPe;
	InstructionMap[0xF9] = &CPU::LDSPHL;
	InstructionMap[0xFA] = &CPU::LDA_nn_;
	InstructionMap[0xFB] = &CPU::EI;
	InstructionMap[0xFE] = &CPU::CPn;
	InstructionMap[0xFF] = &CPU::RSTn;

	
	//Z80 Set CB
	
	InstructionCBMap[0x00] = &CPU::RLCr;
	InstructionCBMap[0x01] = &CPU::RLCr;
	InstructionCBMap[0x02] = &CPU::RLCr;
	InstructionCBMap[0x03] = &CPU::RLCr;
	InstructionCBMap[0x04] = &CPU::RLCr;
	InstructionCBMap[0x05] = &CPU::RLCr;
	InstructionCBMap[0x06] = &CPU::RLC_HL_;
	InstructionCBMap[0x07] = &CPU::RLCr;
	InstructionCBMap[0x08] = &CPU::RRCr;
	InstructionCBMap[0x09] = &CPU::RRCr;
	InstructionCBMap[0x0A] = &CPU::RRCr;
	InstructionCBMap[0x0B] = &CPU::RRCr;
	InstructionCBMap[0x0C] = &CPU::RRCr;
	InstructionCBMap[0x0D] = &CPU::RRCr;
	InstructionCBMap[0x0E] = &CPU::RRC_HL_;
	InstructionCBMap[0x0F] = &CPU::RRCr;
	InstructionCBMap[0x10] = &CPU::RLr;
	InstructionCBMap[0x11] = &CPU::RLr;
	InstructionCBMap[0x12] = &CPU::RLr;
	InstructionCBMap[0x13] = &CPU::RLr;
	InstructionCBMap[0x14] = &CPU::RLr;
	InstructionCBMap[0x15] = &CPU::RLr;
	InstructionCBMap[0x16] = &CPU::RL_HL_;
	InstructionCBMap[0x17] = &CPU::RLr;
	InstructionCBMap[0x18] = &CPU::RRr;
	InstructionCBMap[0x19] = &CPU::RRr;
	InstructionCBMap[0x1A] = &CPU::RRr;
	InstructionCBMap[0x1B] = &CPU::RRr;
	InstructionCBMap[0x1C] = &CPU::RRr;
	InstructionCBMap[0x1D] = &CPU::RRr;
	InstructionCBMap[0x1E] = &CPU::RR_HL_;
	InstructionCBMap[0x1F] = &CPU::RRr;
	InstructionCBMap[0x20] = &CPU::SLAr;
	InstructionCBMap[0x21] = &CPU::SLAr;
	InstructionCBMap[0x22] = &CPU::SLAr;
	InstructionCBMap[0x23] = &CPU::SLAr;
	InstructionCBMap[0x24] = &CPU::SLAr;
	InstructionCBMap[0x25] = &CPU::SLAr;
	InstructionCBMap[0x26] = &CPU::SLA_HL_;
	InstructionCBMap[0x27] = &CPU::SLAr;
	InstructionCBMap[0x28] = &CPU::SRAr;
	InstructionCBMap[0x29] = &CPU::SRAr;
	InstructionCBMap[0x2A] = &CPU::SRAr;
	InstructionCBMap[0x2B] = &CPU::SRAr;
	InstructionCBMap[0x2C] = &CPU::SRAr;
	InstructionCBMap[0x2D] = &CPU::SRAr;
	InstructionCBMap[0x2E] = &CPU::SRA_HL_;
	InstructionCBMap[0x2F] = &CPU::SRAr;
	InstructionCBMap[0x30] = &CPU::SWAPr;
	InstructionCBMap[0x31] = &CPU::SWAPr;
	InstructionCBMap[0x32] = &CPU::SWAPr;
	InstructionCBMap[0x33] = &CPU::SWAPr;
	InstructionCBMap[0x34] = &CPU::SWAPr;
	InstructionCBMap[0x35] = &CPU::SWAPr;
	InstructionCBMap[0x36] = &CPU::SWAP_HL_;
	InstructionCBMap[0x37] = &CPU::SWAPr;
	InstructionCBMap[0x38] = &CPU::SRLr;
	InstructionCBMap[0x39] = &CPU::SRLr;
	InstructionCBMap[0x3A] = &CPU::SRLr;
	InstructionCBMap[0x3B] = &CPU::SRLr;
	InstructionCBMap[0x3C] = &CPU::SRLr;
	InstructionCBMap[0x3D] = &CPU::SRLr;
	InstructionCBMap[0x3E] = &CPU::SRL_HL_;
	InstructionCBMap[0x3F] = &CPU::SRLr;
	InstructionCBMap[0x40] = &CPU::BITbr;
	InstructionCBMap[0x41] = &CPU::BITbr;
	InstructionCBMap[0x42] = &CPU::BITbr;
	InstructionCBMap[0x43] = &CPU::BITbr;
	InstructionCBMap[0x44] = &CPU::BITbr;
	InstructionCBMap[0x45] = &CPU::BITbr;
	InstructionCBMap[0x46] = &CPU::BITb_HL_;
	InstructionCBMap[0x47] = &CPU::BITbr;
	InstructionCBMap[0x48] = &CPU::BITbr;
	InstructionCBMap[0x49] = &CPU::BITbr;
	InstructionCBMap[0x4A] = &CPU::BITbr;
	InstructionCBMap[0x4B] = &CPU::BITbr;
	InstructionCBMap[0x4C] = &CPU::BITbr;
	InstructionCBMap[0x4D] = &CPU::BITbr;
	InstructionCBMap[0x4E] = &CPU::BITb_HL_;
	InstructionCBMap[0x4F] = &CPU::BITbr;
	InstructionCBMap[0x50] = &CPU::BITbr;
	InstructionCBMap[0x51] = &CPU::BITbr;
	InstructionCBMap[0x52] = &CPU::BITbr;
	InstructionCBMap[0x53] = &CPU::BITbr;
	InstructionCBMap[0x54] = &CPU::BITbr;
	InstructionCBMap[0x55] = &CPU::BITbr;
	InstructionCBMap[0x56] = &CPU::BITb_HL_;
	InstructionCBMap[0x57] = &CPU::BITbr;
	InstructionCBMap[0x58] = &CPU::BITbr;
	InstructionCBMap[0x59] = &CPU::BITbr;
	InstructionCBMap[0x5A] = &CPU::BITbr;
	InstructionCBMap[0x5B] = &CPU::BITbr;
	InstructionCBMap[0x5C] = &CPU::BITbr;
	InstructionCBMap[0x5D] = &CPU::BITbr;
	InstructionCBMap[0x5E] = &CPU::BITb_HL_;
	InstructionCBMap[0x5F] = &CPU::BITbr;
	InstructionCBMap[0x60] = &CPU::BITbr;
	InstructionCBMap[0x61] = &CPU::BITbr;
	InstructionCBMap[0x62] = &CPU::BITbr;
	InstructionCBMap[0x63] = &CPU::BITbr;
	InstructionCBMap[0x64] = &CPU::BITbr;
	InstructionCBMap[0x65] = &CPU::BITbr;
	InstructionCBMap[0x66] = &CPU::BITb_HL_;
	InstructionCBMap[0x67] = &CPU::BITbr;
	InstructionCBMap[0x68] = &CPU::BITbr;
	InstructionCBMap[0x69] = &CPU::BITbr;
	InstructionCBMap[0x6A] = &CPU::BITbr;
	InstructionCBMap[0x6B] = &CPU::BITbr;
	InstructionCBMap[0x6C] = &CPU::BITbr;
	InstructionCBMap[0x6D] = &CPU::BITbr;
	InstructionCBMap[0x6E] = &CPU::BITb_HL_;
	InstructionCBMap[0x6F] = &CPU::BITbr;
	InstructionCBMap[0x70] = &CPU::BITbr;
	InstructionCBMap[0x71] = &CPU::BITbr;
	InstructionCBMap[0x72] = &CPU::BITbr;
	InstructionCBMap[0x73] = &CPU::BITbr;
	InstructionCBMap[0x74] = &CPU::BITbr;
	InstructionCBMap[0x75] = &CPU::BITbr;
	InstructionCBMap[0x76] = &CPU::BITb_HL_;
	InstructionCBMap[0x77] = &CPU::BITbr;
	InstructionCBMap[0x78] = &CPU::BITbr;
	InstructionCBMap[0x79] = &CPU::BITbr;
	InstructionCBMap[0x7A] = &CPU::BITbr;
	InstructionCBMap[0x7B] = &CPU::BITbr;
	InstructionCBMap[0x7C] = &CPU::BITbr;
	InstructionCBMap[0x7D] = &CPU::BITbr;
	InstructionCBMap[0x7E] = &CPU::BITb_HL_;
	InstructionCBMap[0x7F] = &CPU::BITbr;
	InstructionCBMap[0x80] = &CPU::RESbr;
	InstructionCBMap[0x81] = &CPU::RESbr;
	InstructionCBMap[0x82] = &CPU::RESbr;
	InstructionCBMap[0x83] = &CPU::RESbr;
	InstructionCBMap[0x84] = &CPU::RESbr;
	InstructionCBMap[0x85] = &CPU::RESbr;
	InstructionCBMap[0x86] = &CPU::RESb_HL_;
	InstructionCBMap[0x87] = &CPU::RESbr;
	InstructionCBMap[0x88] = &CPU::RESbr;
	InstructionCBMap[0x89] = &CPU::RESbr;
	InstructionCBMap[0x8A] = &CPU::RESbr;
	InstructionCBMap[0x8B] = &CPU::RESbr;
	InstructionCBMap[0x8C] = &CPU::RESbr;
	InstructionCBMap[0x8D] = &CPU::RESbr;
	InstructionCBMap[0x8E] = &CPU::RESb_HL_;
	InstructionCBMap[0x8F] = &CPU::RESbr;
	InstructionCBMap[0x90] = &CPU::RESbr;
	InstructionCBMap[0x91] = &CPU::RESbr;
	InstructionCBMap[0x92] = &CPU::RESbr;
	InstructionCBMap[0x93] = &CPU::RESbr;
	InstructionCBMap[0x94] = &CPU::RESbr;
	InstructionCBMap[0x95] = &CPU::RESbr;
	InstructionCBMap[0x96] = &CPU::RESb_HL_;
	InstructionCBMap[0x97] = &CPU::RESbr;
	InstructionCBMap[0x98] = &CPU::RESbr;
	InstructionCBMap[0x99] = &CPU::RESbr;
	InstructionCBMap[0x9A] = &CPU::RESbr;
	InstructionCBMap[0x9B] = &CPU::RESbr;
	InstructionCBMap[0x9C] = &CPU::RESbr;
	InstructionCBMap[0x9D] = &CPU::RESbr;
	InstructionCBMap[0x9E] = &CPU::RESb_HL_;
	InstructionCBMap[0x9F] = &CPU::RESbr;
	InstructionCBMap[0xA0] = &CPU::RESbr;
	InstructionCBMap[0xA1] = &CPU::RESbr;
	InstructionCBMap[0xA2] = &CPU::RESbr;
	InstructionCBMap[0xA3] = &CPU::RESbr;
	InstructionCBMap[0xA4] = &CPU::RESbr;
	InstructionCBMap[0xA5] = &CPU::RESbr;
	InstructionCBMap[0xA6] = &CPU::RESb_HL_;
	InstructionCBMap[0xA7] = &CPU::RESbr;
	InstructionCBMap[0xA8] = &CPU::RESbr;
	InstructionCBMap[0xA9] = &CPU::RESbr;
	InstructionCBMap[0xAA] = &CPU::RESbr;
	InstructionCBMap[0xAB] = &CPU::RESbr;
	InstructionCBMap[0xAC] = &CPU::RESbr;
	InstructionCBMap[0xAD] = &CPU::RESbr;
	InstructionCBMap[0xAE] = &CPU::RESb_HL_;
	InstructionCBMap[0xAF] = &CPU::RESbr;
	InstructionCBMap[0xB0] = &CPU::RESbr;
	InstructionCBMap[0xB1] = &CPU::RESbr;
	InstructionCBMap[0xB2] = &CPU::RESbr;
	InstructionCBMap[0xB3] = &CPU::RESbr;
	InstructionCBMap[0xB4] = &CPU::RESbr;
	InstructionCBMap[0xB5] = &CPU::RESbr;
	InstructionCBMap[0xB6] = &CPU::RESb_HL_;
	InstructionCBMap[0xB7] = &CPU::RESbr;
	InstructionCBMap[0xB8] = &CPU::RESbr;
	InstructionCBMap[0xB9] = &CPU::RESbr;
	InstructionCBMap[0xBA] = &CPU::RESbr;
	InstructionCBMap[0xBB] = &CPU::RESbr;
	InstructionCBMap[0xBC] = &CPU::RESbr;
	InstructionCBMap[0xBD] = &CPU::RESbr;
	InstructionCBMap[0xBE] = &CPU::RESb_HL_;
	InstructionCBMap[0xBF] = &CPU::RESbr;
	InstructionCBMap[0xC0] = &CPU::SETbr;
	InstructionCBMap[0xC1] = &CPU::SETbr;
	InstructionCBMap[0xC2] = &CPU::SETbr;
	InstructionCBMap[0xC3] = &CPU::SETbr;
	InstructionCBMap[0xC4] = &CPU::SETbr;
	InstructionCBMap[0xC5] = &CPU::SETbr;
	InstructionCBMap[0xC6] = &CPU::SETb_HL_;
	InstructionCBMap[0xC7] = &CPU::SETbr;
	InstructionCBMap[0xC8] = &CPU::SETbr;
	InstructionCBMap[0xC9] = &CPU::SETbr;
	InstructionCBMap[0xCA] = &CPU::SETbr;
	InstructionCBMap[0xCB] = &CPU::SETbr;
	InstructionCBMap[0xCC] = &CPU::SETbr;
	InstructionCBMap[0xCD] = &CPU::SETbr;
	InstructionCBMap[0xCE] = &CPU::SETb_HL_;
	InstructionCBMap[0xCF] = &CPU::SETbr;
	InstructionCBMap[0xD0] = &CPU::SETbr;
	InstructionCBMap[0xD1] = &CPU::SETbr;
	InstructionCBMap[0xD2] = &CPU::SETbr;
	InstructionCBMap[0xD3] = &CPU::SETbr;
	InstructionCBMap[0xD4] = &CPU::SETbr;
	InstructionCBMap[0xD5] = &CPU::SETbr;
	InstructionCBMap[0xD6] = &CPU::SETb_HL_;
	InstructionCBMap[0xD7] = &CPU::SETbr;
	InstructionCBMap[0xD8] = &CPU::SETbr;
	InstructionCBMap[0xD9] = &CPU::SETbr;
	InstructionCBMap[0xDA] = &CPU::SETbr;
	InstructionCBMap[0xDB] = &CPU::SETbr;
	InstructionCBMap[0xDC] = &CPU::SETbr;
	InstructionCBMap[0xDD] = &CPU::SETbr;
	InstructionCBMap[0xDE] = &CPU::SETb_HL_;
	InstructionCBMap[0xDF] = &CPU::SETbr;
	InstructionCBMap[0xE0] = &CPU::SETbr;
	InstructionCBMap[0xE1] = &CPU::SETbr;
	InstructionCBMap[0xE2] = &CPU::SETbr;
	InstructionCBMap[0xE3] = &CPU::SETbr;
	InstructionCBMap[0xE4] = &CPU::SETbr;
	InstructionCBMap[0xE5] = &CPU::SETbr;
	InstructionCBMap[0xE6] = &CPU::SETb_HL_;
	InstructionCBMap[0xE7] = &CPU::SETbr;
	InstructionCBMap[0xE8] = &CPU::SETbr;
	InstructionCBMap[0xE9] = &CPU::SETbr;
	InstructionCBMap[0xEA] = &CPU::SETbr;
	InstructionCBMap[0xEB] = &CPU::SETbr;
	InstructionCBMap[0xEC] = &CPU::SETbr;
	InstructionCBMap[0xED] = &CPU::SETbr;
	InstructionCBMap[0xEE] = &CPU::SETb_HL_;
	InstructionCBMap[0xEF] = &CPU::SETbr;
	InstructionCBMap[0xF0] = &CPU::SETbr;
	InstructionCBMap[0xF1] = &CPU::SETbr;
	InstructionCBMap[0xF2] = &CPU::SETbr;
	InstructionCBMap[0xF3] = &CPU::SETbr;
	InstructionCBMap[0xF4] = &CPU::SETbr;
	InstructionCBMap[0xF5] = &CPU::SETbr;
	InstructionCBMap[0xF6] = &CPU::SETb_HL_;
	InstructionCBMap[0xF7] = &CPU::SETbr;
	InstructionCBMap[0xF8] = &CPU::SETbr;
	InstructionCBMap[0xF9] = &CPU::SETbr;
	InstructionCBMap[0xFA] = &CPU::SETbr;
	InstructionCBMap[0xFB] = &CPU::SETbr;
	InstructionCBMap[0xFC] = &CPU::SETbr;
	InstructionCBMap[0xFD] = &CPU::SETbr;
	InstructionCBMap[0xFE] = &CPU::SETb_HL_;
	InstructionCBMap[0xFF] = &CPU::SETbr;

	ByteRegisterMap[0x00] = reinterpret_cast<uint8_t*>(&BC) + 1;
	ByteRegisterMap[0x01] = reinterpret_cast<uint8_t*>(&BC);
	ByteRegisterMap[0x02] = reinterpret_cast<uint8_t*>(&DE) + 1;
	ByteRegisterMap[0x03] = reinterpret_cast<uint8_t*>(&DE);
	ByteRegisterMap[0x04] = reinterpret_cast<uint8_t*>(&HL) + 1;
	ByteRegisterMap[0x05] = reinterpret_cast<uint8_t*>(&HL);
	ByteRegisterMap[0x06] = reinterpret_cast<uint8_t*>(&AF);
	ByteRegisterMap[0x07] = reinterpret_cast<uint8_t*>(&AF) + 1;


	RegisterMap16Bit[0x00] = &BC;
	RegisterMap16Bit[0x01] = &DE;
	RegisterMap16Bit[0x02] = &HL;
	RegisterMap16Bit[0x03] = &SP;

}