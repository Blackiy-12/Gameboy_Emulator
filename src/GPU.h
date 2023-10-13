#pragma once

#include "IMemoryUnit.h"

#include <functional>

class MMU;

class CPU;

enum class ControllerMode;

class GPU : public IMemoryUnit
{
public:
    GPU(MMU* MMUPtr, CPU* CPUPtr);
    
    ~GPU();

    void step(unsigned long Cycles);
    
    uint8_t* getCurrentFrame();

    uint8_t readByte(const uint16_t& Address);
    
    void writeByte(const uint16_t& Address, const uint8_t Value);
    
    void setVSyncCallback(std::function<void()> Callback);
    
    void preBoot();

private:
    
    inline void setControllerMode(ControllerMode Mode);

    inline ControllerMode getContrellerMode();

    inline bool isLCDDisplayEnabled();

    inline bool isWindowTileMapDisplaySelect();

    inline bool isWindowDisplayEnable();

    inline bool isBGWindowTileDataSelect();

    inline bool isBGTileMapDisplaySelect();

    inline bool isOBJSize();

    inline bool isOBJDisplayEnable();

    inline bool isBGDisplayEnable();

    inline bool isHBlankInterrupt();

    inline bool isOAMInterrupt();

    inline bool isVBlankInterrupt();

    inline bool isLYCoincidenceInterrupt();

    void launchDMATransfer(const uint8_t Address);
    
    void renderScanline();
         
    void renderImage();
         
    void renderBackgroundScanline();
         
    void renderWindowScanline();
         
    void renderOBJScanline();

private:
    MMU* MMUPtr;
    
    CPU* CPUPtr;
    
    uint8_t VRAM[0x1FFF + 1];
    
    uint8_t OAM[0x009F + 1];
    
    uint8_t bgPixels[160 * 144 * 4];
    
    uint8_t DisplayPixels[160 * 144 * 4];

    unsigned long ModeClock;
    
    int DMAClocksRemaining;
    
    std::function<void()> Callback;

    uint8_t LCDControl;
    
    uint8_t LCDControllerStatus;
    
    uint8_t ScrollY;
    
    uint8_t ScrollX;
    
    uint8_t LCDControllerYCoordinate;
    
    uint8_t LYCompare;
    
    uint8_t WindowYPosition;
    
    uint8_t WindowXPositionMinus7;
    
    uint8_t BGPaletteData;
    
    uint8_t ObjectPalette0Data;
    
    uint8_t ObjectPalette1Data;
};

