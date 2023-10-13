#pragma once

#include "IMemoryUnit.h"

class MBC : public IMemoryUnit
{
public:
    MBC(uint8_t* ROMPtr, uint8_t* RAMPtr);
    ~MBC();

public:
    // IMemoryUnit
    virtual uint8_t readByte(const uint16_t& Address) = 0;
    virtual void writeByte(const uint16_t& Address, const uint8_t Value) = 0;

protected:
    uint8_t* ROM;
    uint8_t* RAM;
    bool RAMEnabled;
};

class ROMOnly_MBC : public MBC
{
public:
    ROMOnly_MBC(uint8_t* ROMPtr, uint8_t* RAMPtr);
    ~ROMOnly_MBC();

    // IMemoryUnit
    uint8_t readByte(const uint16_t& Address);
    void writeByte(const uint16_t& Address, const uint8_t Value);
};

class MBC1_MBC : public MBC
{
public:
    MBC1_MBC(uint8_t* ROMPtr, uint8_t* RAMPtr);
    ~MBC1_MBC();

    // IMemoryUnit
    uint8_t readByte(const uint16_t& Address);
    void writeByte(const uint16_t& Address, const uint8_t Value);

private:
    uint8_t ROMBankLower;
    uint8_t ROMRAMBankUpper;
    uint8_t ROMRAMMode;
};

class MBC2_MBC : public MBC
{
public:
    MBC2_MBC(uint8_t* ROMPtr);
    ~MBC2_MBC();

    // IMemoryUnit
    uint8_t readByte(const uint16_t& Address);
    void writeByte(const uint16_t& Address, const uint8_t Value);

private:
    uint8_t ROMBank;
};

class MBC3_MBC : public MBC
{
public:
    MBC3_MBC(uint8_t* ROMPtr, uint8_t* RAMPtr);
    ~MBC3_MBC();

    // IMemoryUnit
    uint8_t readByte(const uint16_t& Address);
    void writeByte(const uint16_t& Address, const uint8_t Value);

private:
    uint8_t ROMBank;
    uint8_t RAMBank;
    uint8_t RTCRegisters[0x05];
};

class MBC5_MBC : public MBC
{
public:
    MBC5_MBC(uint8_t* ROMPtr, uint8_t* RAMPtr);
    ~MBC5_MBC();

    // IMemoryUnit
    uint8_t readByte(const uint16_t& Address);
    void writeByte(const uint16_t& Address, const uint8_t Value);

private:
    uint8_t RAMG;

    uint16_t ROMBank;
    uint8_t RAMBank;
};