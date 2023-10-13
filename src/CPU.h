#pragma once

#include <memory>
#include <functional>

#include "Cartridge.h"
#include "MMU.h"
#include "GPU.h"
#include "Joypad.h"
#include "Timer.h"
#include "Serial.h"
#include "MMU.h"
#include "APU.h"

#define INT40 0x40
#define INT48 0x48
#define INT50 0x50
#define INT58 0x58
#define INT60 0x60


class CPU
{
public:
	CPU();

	~CPU();

public:
    void initialize();

	void loadCartridge(const char* PathToCartridge, const char* BootPath);

    uint8_t step();

    void triggerInterrupt(uint8_t Interrupt);

    uint8_t* getCurrentFrame();
    
    void setInput(uint8_t Input, uint8_t Buttons);
    
    void setVSyncCallback(std::function<void()> Callback);


private:
    static uint8_t getHighByte(uint16_t Dest);
    
    static uint8_t getLowByte(uint16_t Dest);

    uint8_t* getByteRegister(uint8_t Value);
    
    uint16_t* getUShortRegister(uint8_t Value, bool UseAF);

    void setHighByte(uint16_t* Dest, uint8_t Value);
    
    void setLowByte(uint16_t* Dest, uint8_t Value);

    void setFlag(uint8_t Flag);
    
    void clearFlag(uint8_t Flag);
    
    bool isFlagSet(uint8_t Flag);

    void pushByteToSP(uint8_t Value);
    
    void pushUShortToSP(uint16_t Value);
    
    uint16_t popUShort();
    
    uint8_t popByte();
    
    uint8_t readBytePC();
    
    uint16_t readUShortPC();

    uint8_t addByte(uint8_t Left, uint8_t Right);
    
    uint16_t addUShort(uint16_t Left, uint16_t Right);
    
    void ADC(uint8_t Value);
    
    void SBC(uint8_t Value);

    void handleInterrupts();

private:

    // MMU (Memory Map Unit)
    std::unique_ptr<MMU> MMUPtr;

    // Cartridge
    std::unique_ptr<Cartridge> CartridgePtr;

    // GPU
    std::unique_ptr<GPU> GPUPtr;

    // APU
    std::unique_ptr<APU> APUPtr;

    // Joypad
    std::unique_ptr<Joypad> JoypadPtr;

    // Serial
    std::unique_ptr<Serial> SerialPtr;

    // Timer
    std::unique_ptr<Timer> TimerPtr;

    // Clock cycles
    unsigned long Cycles; // The current number of cycles

    bool Halted;

	//Registers

	uint16_t AF;
	
    uint16_t BC;
	
    uint16_t DE;
	
    uint16_t HL;
	
    uint16_t SP;
	
    uint16_t PC;

    uint8_t* ByteRegisterMap[0x07 + 1];
    
    uint16_t* RegisterMap16Bit[0x03 + 1];

    uint8_t IME;

private:
	//Instructions

	uint8_t NOP             (const uint8_t& Operand);
                            
    uint8_t LDrn            (const uint8_t& Operand);
                            
    uint8_t LDrR            (const uint8_t& Operand);
                            
    uint8_t LDrrnn          (const uint8_t& Operand);
                            
    uint8_t INCr            (const uint8_t& Operand);
                            
    uint8_t INCrr           (const uint8_t& Operand);
                            
    uint8_t DECrr           (const uint8_t& Operand);
                            
    uint8_t ORr             (const uint8_t& Operand);
                            
    uint8_t XORr            (const uint8_t& Operand);
                            
    uint8_t PUSHrr          (const uint8_t& Operand);
                            
    uint8_t POPrr           (const uint8_t& Operand);
                            
    uint8_t DECr            (const uint8_t& Operand);
                            
    uint8_t SUBr            (const uint8_t& Operand);
                            
    uint8_t SBCAr           (const uint8_t& Operand);
                            
    uint8_t CALLccnn        (const uint8_t& Operand);
                            
    uint8_t LDr_HL_         (const uint8_t& Operand);
                            
    uint8_t LD_HL_r         (const uint8_t& Operand);
                            
    uint8_t RETcc           (const uint8_t& Operand);
                            
    uint8_t ADDHLss         (const uint8_t& Operand);
                            
    uint8_t JPccnn          (const uint8_t& Operand);
                            
    uint8_t ADDAr           (const uint8_t& Operand);
                            
    uint8_t ADCAr           (const uint8_t& Operand);
                            
    uint8_t JRcce           (const uint8_t& Operand);
                            
    uint8_t ANDr            (const uint8_t& Operand);
                            
    uint8_t CPr             (const uint8_t& Operand);
                            
    uint8_t RSTn            (const uint8_t& Operand);

    uint8_t LD_BC_A             (const uint8_t& Operand);

    uint8_t RLCA                (const uint8_t& Operand);

    uint8_t LD_nn_SP            (const uint8_t& Operand);

    uint8_t LDA_BC_             (const uint8_t& Operand);

    uint8_t RRCA                (const uint8_t& Operand);

    uint8_t STOP                (const uint8_t& Operand);

    uint8_t LD_DE_A             (const uint8_t& Operand);

    uint8_t RLA                 (const uint8_t& Operand);

    uint8_t JRe                 (const uint8_t& Operand);

    uint8_t LDA_DE_             (const uint8_t& Operand);

    uint8_t RRA                 (const uint8_t& Operand);

    uint8_t LDI_HL_A            (const uint8_t& Operand);

    uint8_t DAA                 (const uint8_t& Operand);

    uint8_t LDIA_HL_            (const uint8_t& Operand);

    uint8_t CPL                 (const uint8_t& Operand);

    uint8_t LDD_HL_A            (const uint8_t& Operand);

    uint8_t INC_HL_             (const uint8_t& Operand);

    uint8_t DEC_HL_             (const uint8_t& Operand);

    uint8_t LD_HL_n             (const uint8_t& Operand);

    uint8_t SCF                 (const uint8_t& Operand);

    uint8_t LDDA_HL_            (const uint8_t& Operand);

    uint8_t CCF                 (const uint8_t& Operand);

    uint8_t HALT                (const uint8_t& Operand);

    uint8_t ADDA_HL_            (const uint8_t& Operand);

    uint8_t ADCA_HL_            (const uint8_t& Operand);

    uint8_t SUB_HL_             (const uint8_t& Operand);

    uint8_t SBCA_HL_            (const uint8_t& Operand);

    uint8_t AND_HL_             (const uint8_t& Operand);

    uint8_t XOR_HL_             (const uint8_t& Operand);

    uint8_t OR_HL_              (const uint8_t& Operand);

    uint8_t CP_HL_              (const uint8_t& Operand);

    uint8_t JPnn                (const uint8_t& Operand);

    uint8_t ADDAn               (const uint8_t& Operand);

    uint8_t RET                 (const uint8_t& Operand);

    uint8_t CALLnn              (const uint8_t& Operand);

    uint8_t ADCAn               (const uint8_t& Operand);

    uint8_t SUBn                (const uint8_t& Operand);

    uint8_t RETI                (const uint8_t& Operand);

    uint8_t SBCAn               (const uint8_t& Operand);

    uint8_t LD_0xFF00n_A        (const uint8_t& Operand);

    uint8_t LD_0xFF00C_A        (const uint8_t& Operand);

    uint8_t ANDn                (const uint8_t& Operand);

    uint8_t ADDSPdd             (const uint8_t& Operand);

    uint8_t JP_HL_              (const uint8_t& Operand);

    uint8_t LD_nn_A             (const uint8_t& Operand);

    uint8_t XORn                (const uint8_t& Operand);

    uint8_t LDA_0xFF00n_        (const uint8_t& Operand);

    uint8_t LDA_0xFF00C_        (const uint8_t& Operand);

    uint8_t DI                  (const uint8_t& Operand);

    uint8_t ORn                 (const uint8_t& Operand);

    uint8_t LDHLSPe             (const uint8_t& Operand);

    uint8_t LDSPHL              (const uint8_t& Operand);

    uint8_t LDA_nn_             (const uint8_t& Operand);

    uint8_t EI                  (const uint8_t& Operand);

    uint8_t CPn                 (const uint8_t& Operand);


    //Z80 - Set CB

    uint8_t RLCr            (const uint8_t& Operand);

    uint8_t RLC_HL_         (const uint8_t& Operand);

    uint8_t RRCr            (const uint8_t& Operand);

    uint8_t RRC_HL_         (const uint8_t& Operand);

    uint8_t RLr             (const uint8_t& Operand);

    uint8_t RL_HL_          (const uint8_t& Operand);

    uint8_t RRr             (const uint8_t& Operand);

    uint8_t RR_HL_          (const uint8_t& Operand);

    uint8_t SLAr            (const uint8_t& Operand);

    uint8_t SLA_HL_         (const uint8_t& Operand);

    uint8_t SRLr            (const uint8_t& Operand);

    uint8_t SRL_HL_         (const uint8_t& Operand);

    uint8_t SRAr            (const uint8_t& Operand);

    uint8_t SRA_HL_         (const uint8_t& Operand);

    uint8_t BITbr           (const uint8_t& Operand);

    uint8_t BITb_HL_        (const uint8_t& Operand);

    uint8_t RESbr           (const uint8_t& Operand);

    uint8_t RESb_HL_        (const uint8_t& Operand);

    uint8_t SETbr           (const uint8_t& Operand);

    uint8_t SETb_HL_        (const uint8_t& Operand);

    uint8_t SWAPr           (const uint8_t& Operand);

    uint8_t SWAP_HL_        (const uint8_t& Operand);


private:
	void setupInstructionMap();

	typedef uint8_t(CPU::* Instruction)(const uint8_t& Operand);
	
	Instruction InstructionMap[0xFF + 1];

	Instruction InstructionCBMap[0xFF + 1];

};

