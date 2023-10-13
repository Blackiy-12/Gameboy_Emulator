#include "GPU.h"

#include "CPU.h"
#include "MMU.h"
#include "BitOperation.h"
#include "Logger.h"

#include <algorithm>

#define TINT 0

const int VBlankCycles         = 456;
const int HBlankCycles         = 204;
const int ReadingOAMCycles     =  80;
const int ReadingOAMVRAMCycles = 172;

const uint16_t LCDControlAddress                = 0xFF40;
const uint16_t LCDControllerStatusAddress       = 0xFF41;
const uint16_t ScrollYAddress                   = 0xFF42;
const uint16_t ScrollXAddress                   = 0xFF43;
const uint16_t LCDControllerYCoordinateAddress  = 0xFF44;
const uint16_t LYCompareAddress                 = 0xFF45;
const uint16_t WindowYPositionAddress           = 0xFF4A;
const uint16_t WindowXPositionMinus7Address     = 0xFF4B;
const uint16_t BGPaletteDataAddress             = 0xFF47;
const uint16_t ObjectPalette0DataAddress        = 0xFF48;
const uint16_t ObjectPalette1DataAddress        = 0xFF49;
const uint16_t DMATransferAndStartAddress       = 0xFF46;

enum class ControllerMode
{
    ModeHBlank          = 0,
    ModeVBlank          = 1,
    ModeReadingOAM      = 2,
    ModeReadingOAMVRAM  = 3
};

const uint8_t GBColors[] = {
    0xEB, 
    0xC4,
    0x60,
    0x00
};


GPU::GPU(MMU* MMUPtr, CPU* CPUPtr)
    :   MMUPtr(MMUPtr),
        CPUPtr(CPUPtr),
        ModeClock(VBlankCycles),
        DMAClocksRemaining(0),
        Callback(nullptr),
        LCDControl(0x00),
        ScrollY(0x00),
        ScrollX(0x00),
        LCDControllerYCoordinate(153),
        LYCompare(0x00),
        WindowYPosition(0x00),
        WindowXPositionMinus7(0x00),
        BGPaletteData(0x00),
        ObjectPalette0Data(0x00),
        ObjectPalette1Data(0x00)
{
    setControllerMode(ControllerMode::ModeVBlank);
    memset(DisplayPixels, 0x00, std::end(DisplayPixels) - std::begin(DisplayPixels));
}

GPU::~GPU()
{
}

void GPU::step(unsigned long Cycles)
{
    if (DMAClocksRemaining > 0)
        DMAClocksRemaining -= Cycles;

    if (isLCDDisplayEnabled() == false)
        return;

    ModeClock += Cycles;

    switch (getContrellerMode())
    {
    case ControllerMode::ModeReadingOAM:
        if (ModeClock >= ReadingOAMCycles)
        {
            ModeClock -= ReadingOAMCycles;
            setControllerMode(ControllerMode::ModeReadingOAMVRAM);
        }
        break;

    case ControllerMode::ModeReadingOAMVRAM:
        if (ModeClock >= ReadingOAMVRAMCycles)
        {
            ModeClock -= ReadingOAMVRAMCycles;

            renderScanline();

            setControllerMode(ControllerMode::ModeHBlank);

            if (isHBlankInterrupt() && (CPUPtr != nullptr))
                CPUPtr->triggerInterrupt(INT48);
        }
        break;

    case ControllerMode::ModeHBlank:
        if (ModeClock >= HBlankCycles)
        {
            ModeClock -= HBlankCycles;

            LCDControllerYCoordinate++;
            if (LCDControllerYCoordinate == 144)
            {
                setControllerMode(ControllerMode::ModeVBlank);
                renderImage();

                if (CPUPtr != nullptr)
                {
                    CPUPtr->triggerInterrupt(INT40);
                    
                    if (isVBlankInterrupt())
                        CPUPtr->triggerInterrupt(INT48);
                }
            }
            else
            {
                setControllerMode(ControllerMode::ModeReadingOAM);

                if (isOAMInterrupt() && (CPUPtr != nullptr))
                    CPUPtr->triggerInterrupt(INT48);
            }
        }
        break;
    case ControllerMode::ModeVBlank:
        if (ModeClock >= VBlankCycles)
        {
            ModeClock -= VBlankCycles;

            LCDControllerYCoordinate++;
            if (LCDControllerYCoordinate == 154)
            {
                setControllerMode(ControllerMode::ModeReadingOAM);

                LCDControllerYCoordinate = 0x00;

                if (isOAMInterrupt() && (CPUPtr != nullptr))
                    CPUPtr->triggerInterrupt(INT48);
            }
        }
        break;
    }

    if (LYCompare == LCDControllerYCoordinate)
    {
        setBit(LCDControllerStatus, 2);
        
        if (isLYCoincidenceInterrupt() && (CPUPtr != nullptr))
            CPUPtr->triggerInterrupt(INT48);

    }
    else
        clearBit(LCDControllerStatus, 2);
}

uint8_t* GPU::getCurrentFrame()
{
    return DisplayPixels;
}

uint8_t GPU::readByte(const uint16_t& Address)
{
    if (Address >= 0x8000 && Address <= 0x9FFF)
        return VRAM[Address - 0x8000];

    else if (Address >= 0xFE00 && Address <= 0xFE9F)
        return OAM[Address - 0xFE00];

    switch (Address)
    {
    
    case LCDControlAddress:
        return LCDControl;
    
    case LCDControllerStatusAddress:
        return LCDControllerStatus;
    
    case ScrollYAddress:
        return ScrollY;
    
    case ScrollXAddress:
        return ScrollX;
    
    case LCDControllerYCoordinateAddress:
        return LCDControllerYCoordinate;
    
    case LYCompareAddress:
        return LYCompare;
    
    case WindowYPositionAddress:
        return WindowYPosition;
    
    case WindowXPositionMinus7Address:
        return WindowXPositionMinus7;
    
    case BGPaletteDataAddress:
        return BGPaletteDataAddress;
    
    case ObjectPalette0DataAddress:
        return ObjectPalette0Data;
    
    case ObjectPalette1DataAddress:
        return ObjectPalette1Data;
    
    case DMATransferAndStartAddress:
        return 0x00;
    
    default:
        return 0x00;
    }
}

void GPU::writeByte(const uint16_t& Address, const uint8_t Value)
{
    if (Address >= 0x8000 && Address <= 0x9FFF)
    {
        VRAM[Address - 0x8000] = Value;
        return;
    }

    else if (Address >= 0xFE00 && Address <= 0xFE9F)
    {
        OAM[Address - 0xFE00] = Value;
        return;
    }

    switch (Address)
    {
    case LCDControlAddress:
    {
        bool isOn = isLCDDisplayEnabled();
        LCDControl = Value;

        if (isOn && !isLCDDisplayEnabled())
        {
            if (getContrellerMode() != ControllerMode::ModeVBlank)
                Logger::log("The LCD should not be turned off while not in VBlank.");

            memset(DisplayPixels, GBColors[0], std::end(DisplayPixels) - std::begin(DisplayPixels));

            for (unsigned int a = 0; a < std::end(DisplayPixels) - std::begin(DisplayPixels); a += 4)
                DisplayPixels[a] = 0xFF;

            LCDControllerYCoordinate = 153;
            ModeClock = VBlankCycles;
            setControllerMode(ControllerMode::ModeVBlank);
        }
        break;
    }
    case LCDControllerStatusAddress:
        LCDControllerStatus = (Value & 0xF8) | (LCDControllerStatus & 0x07);
        break;

    case ScrollYAddress:
        ScrollY = Value;
        break;

    case ScrollXAddress:
        ScrollX = Value;
        break;

    case LCDControllerYCoordinateAddress:
        LCDControllerYCoordinate = 0;
        break;

    case LYCompareAddress:
        LYCompare = Value;
        break;

    case WindowYPositionAddress:
        WindowYPosition = Value;
        break;

    case WindowXPositionMinus7Address:
        WindowXPositionMinus7 = Value;
        break;

    case BGPaletteDataAddress:
        BGPaletteData = Value;
        break;

    case ObjectPalette0DataAddress:
        ObjectPalette0Data = Value;
        break;

    case ObjectPalette1DataAddress:
        ObjectPalette1Data = Value;
        break;

    case DMATransferAndStartAddress:
        launchDMATransfer(Value);
        break;

    default:
        Logger::log("GPU::WriteByte cannot write to address.");
        break;
    }
}

void GPU::setVSyncCallback(std::function<void()> Callback)
{
    this->Callback = Callback;
}

void GPU::preBoot()
{
    LCDControllerYCoordinate = 0x91;
    ScrollY = 0x00;
    ScrollX = 0x00;
    LYCompare = 0x00;
    BGPaletteData = 0xFC;
    ObjectPalette0Data = 0xFF;
    ObjectPalette1Data = 0xFF;
    WindowYPosition = 0x00;
    WindowXPositionMinus7 = 0x00;

    memset(DisplayPixels, GBColors[0], std::end(DisplayPixels) - std::begin(DisplayPixels));
    
    for (unsigned int a = 0; a < std::end(DisplayPixels) - std::begin(DisplayPixels); a += 4)
        DisplayPixels[a] = 0xFF; 
}

inline void GPU::setControllerMode(ControllerMode Mode)
{
    LCDControllerStatus = ((LCDControllerStatus & ~0x03) | static_cast<uint8_t>(Mode));
}

inline ControllerMode GPU::getContrellerMode()
{
    return static_cast<ControllerMode>(LCDControllerStatus & 0x03);
}

inline bool GPU::isLCDDisplayEnabled()
{
    return isBitSet(LCDControl, 7);
}

inline bool GPU::isWindowTileMapDisplaySelect()
{
    return isBitSet(LCDControl, 6);
}

inline bool GPU::isWindowDisplayEnable()
{
    return isBitSet(LCDControl, 5);
}

inline bool GPU::isBGWindowTileDataSelect()
{
    return isBitSet(LCDControl, 4);
}

inline bool GPU::isBGTileMapDisplaySelect()
{
    return isBitSet(LCDControl, 3);
}

inline bool GPU::isOBJSize()
{
    return isBitSet(LCDControl, 2);
}

inline bool GPU::isOBJDisplayEnable()
{
    return isBitSet(LCDControl, 1);
}

inline bool GPU::isBGDisplayEnable()
{
    return isBitSet(LCDControl,0);
}

inline bool GPU::isHBlankInterrupt()
{
    return isBitSet(LCDControllerStatus, 3);
}

inline bool GPU::isOAMInterrupt()
{
    return isBitSet(LCDControllerStatus, 5);
}

inline bool GPU::isVBlankInterrupt()
{
    return isBitSet(LCDControllerStatus, 4);
}

inline bool GPU::isLYCoincidenceInterrupt()
{
    return isBitSet(LCDControllerStatus, 6);
}

void GPU::launchDMATransfer(const uint8_t Address)
{
    DMAClocksRemaining = 752;

    uint16_t source = (static_cast<uint16_t>(Address) * 0x0100);

    for (uint8_t offset = 0x00; offset <= 0x9F; offset++)
        OAM[offset] = MMUPtr->read(source | offset);
}

void GPU::renderScanline()
{
    renderBackgroundScanline();

    if (isWindowDisplayEnable())
        renderWindowScanline();


    
    memcpy( DisplayPixels + (LCDControllerYCoordinate * 160 * 4),
            bgPixels + (LCDControllerYCoordinate * 160 * 4),
            160 * 4);

    if (isOBJDisplayEnable())
        renderOBJScanline();
}

void GPU::renderImage()
{
    if (Callback != nullptr)
        Callback();
}

void GPU::renderBackgroundScanline()
{
    if (isBGDisplayEnable() == false)
    {
        for (int x = 0; x < 160; x++)
        {
            int index = ((LCDControllerYCoordinate * 160) + x) * 4;
            bgPixels[index + 3] = GBColors[0];   // R
            bgPixels[index + 2] = GBColors[0];   // G
            bgPixels[index + 1] = GBColors[0];   // B
            bgPixels[index + 0] = 0xFF;          // A
        }

        return;
    }

    const uint8_t palette[]
    {   
        GBColors[BGPaletteData & 0x03],
        GBColors[(BGPaletteData >> 2) & 0x03],
        GBColors[(BGPaletteData >> 4) & 0x03],
        GBColors[(BGPaletteData >> 6) & 0x03],
    };


    uint16_t tileNumberMap = isBGTileMapDisplaySelect() ? 0x9C00 : 0x9800;
    tileNumberMap -= 0x8000;

    
    uint16_t tileData = isBGWindowTileDataSelect() ? 0x8000 : 0x9000;
    tileData -= 0x8000;

    
    uint8_t tileY = (uint8_t)(((LCDControllerYCoordinate + ScrollY) / 8) % 32);

    uint8_t tileYOffset = (uint8_t)((LCDControllerYCoordinate + ScrollY) % 8);


    for (uint8_t x = 0; x < 160; x++)
    {
        uint8_t tileX = (uint8_t)(((ScrollX + x) / 8) % 32);

        uint8_t tileNumber = VRAM[(uint16_t)(tileNumberMap + (tileY * 32) + tileX)];

        uint16_t tileDataPtr = 0;
        if (isBGWindowTileDataSelect())
            tileDataPtr = (uint16_t)(tileData + tileNumber * 0x10);

        else
            tileDataPtr = (uint16_t)(tileData + static_cast<int8_t>(tileNumber) * 0x10);

        tileDataPtr += (uint16_t)(tileYOffset * 2);

        uint8_t b1 = VRAM[tileDataPtr];
        uint8_t b2 = VRAM[(uint16_t)(tileDataPtr + 1)];

        uint8_t bit = (uint8_t)(7 - ((ScrollX + x) % 8));
        uint8_t pLo = isBitSet(b1, bit) ? 0x01 : 0x00;
        uint8_t pHi = isBitSet(b2, bit) ? 0x02 : 0x00;

        uint8_t color = palette[pLo + pHi];

        int index = ((LCDControllerYCoordinate * 160) + x) * 4;
        bgPixels[index + 3] = color; // R
#if TINT
        if (bgPixels[index + 3] == 0x00) bgPixels[index + 3] = 0x30;
        bgPixels[index + 2] = 0x00; // G
        bgPixels[index + 1] = 0x00; // B
#else
        bgPixels[index + 2] = color; // G
        bgPixels[index + 1] = color; // B
#endif
        bgPixels[index + 0] = 0xFF;  // A
    }
}

void GPU::renderWindowScanline()
{
    int winY = LCDControllerYCoordinate - WindowYPosition;

    if (winY < 0)
        return;

    const uint8_t palette[]
    {
        GBColors[BGPaletteData & 0x03],
        GBColors[(BGPaletteData >> 2) & 0x03],
        GBColors[(BGPaletteData >> 4) & 0x03],
        GBColors[(BGPaletteData >> 6) & 0x03],
    };

    uint16_t tileNumberMap = isWindowTileMapDisplaySelect() ? 0x9C00 : 0x9800;
    tileNumberMap -= 0x8000;

    uint16_t tileData = isBGWindowTileDataSelect() ? 0x8000 : 0x9000;
    tileData -= 0x8000;

    uint8_t tileY = (uint8_t)(winY / 8);
    uint8_t tileYOffset = (uint8_t)(winY % 8);

    int winX = WindowXPositionMinus7 - 7;
    for (int x = 0; x < 160; x++)
    {
        if (x < winX)
            continue;

        uint8_t tileX = (uint8_t)((x - winX) / 8);

        uint8_t tileNumber = VRAM[(uint16_t)(tileNumberMap + (tileY * 32) + tileX)];

        uint16_t tileDataPtr = 0;
        if (isBGWindowTileDataSelect())
            tileDataPtr = (uint16_t)(tileData + tileNumber * 0x10);

        else
            tileDataPtr = (uint16_t)(tileData + static_cast<int8_t>(tileNumber) * 0x10);

        tileDataPtr += (uint16_t)(tileYOffset * 2);

        uint8_t b1 = VRAM[tileDataPtr];
        uint8_t b2 = VRAM[(uint16_t)(tileDataPtr + 1)];

        uint8_t bit = (uint8_t)(7 - x % 8);
        uint8_t pLo = isBitSet(b1, bit) ? 0x01 : 0x00;
        uint8_t pHi = isBitSet(b2, bit) ? 0x02 : 0x00;
        uint8_t color = palette[pLo + pHi];

        int index = ((LCDControllerYCoordinate * 160) + x) * 4;
        bgPixels[index + 1] = color; // B
#if TINT
        if (bgPixels[index + 1] == 0x00) bgPixels[index + 1] = 0x30;
        bgPixels[index + 2] = 0x00; // G
        bgPixels[index + 3] = 0x00; // R
#else
        bgPixels[index + 2] = color; // G
        bgPixels[index + 3] = color; // R
#endif
        bgPixels[index + 0] = 0xFF;  // A
    }
}

void GPU::renderOBJScanline()
{
    const uint8_t SPRITESIZEINBYTES = 16;
    const uint8_t bgPalette[]
    {
        GBColors[BGPaletteData & 0x03],
        GBColors[(BGPaletteData >> 2) & 0x03],
        GBColors[(BGPaletteData >> 4) & 0x03],
        GBColors[(BGPaletteData >> 6) & 0x03],
    };

    for (int i = 156; i >= 0; i -= 4)
    {
        uint8_t objY = OAM[i];
        uint8_t spriteSize = isOBJSize() ? 0x10 : 0x08;
        int height = spriteSize;

        int y = objY - 16;


        if ((y <= LCDControllerYCoordinate) && ((y + height) > LCDControllerYCoordinate))
        {
            uint8_t objX = OAM[i + 1];               
            uint8_t spriteTileNumber = OAM[i + 2];   
            uint8_t spriteFlags = OAM[i + 3];

            if (spriteSize == 0x10)
                spriteTileNumber &= 0xFE;

            uint8_t paletteNumber = isBitSet(spriteFlags, 4) ? 0x01 : 0x00;

            int x = objX - 8;
            
            const uint16_t tileData = 0x0000;

            uint8_t palette[]
            {
                0x00,
                GBColors[(paletteNumber == 0x00) ? (ObjectPalette0Data >> 2 & 0x03) : (ObjectPalette1Data >> 2 & 0x03)],
                GBColors[(paletteNumber == 0x00) ? (ObjectPalette0Data >> 4 & 0x03) : (ObjectPalette1Data >> 4 & 0x03)],
                GBColors[(paletteNumber == 0x00) ? (ObjectPalette0Data >> 6 & 0x03) : (ObjectPalette1Data >> 6 & 0x03)]
            };

            uint16_t tilePointer = tileData + (spriteTileNumber * SPRITESIZEINBYTES);
            uint8_t tileYOffset = isBitSet(spriteFlags, 6) ? ((height - 1) - (LCDControllerYCoordinate - y)) : (LCDControllerYCoordinate - y);
            tilePointer += (tileYOffset * 2);

            uint8_t low = VRAM[tilePointer];
            uint8_t high = VRAM[(uint16_t)(tilePointer + 1)];

            for (int indexX = 0; indexX < 8; indexX++)
            {
                int pixelX = x + indexX;

                if (pixelX >= 0 && pixelX < 160)
                {
                    uint8_t bit = isBitSet(spriteFlags, 5) ? indexX : 7 - indexX;
                    uint8_t pixelVal = 0x00;
                    if (isBitSet(high, bit)) pixelVal |= 0x02;
                    if (isBitSet(low, bit)) pixelVal |= 0x01;
                    uint8_t color = palette[pixelVal];

                    if (pixelVal != 0x00)
                    {
                        int index = ((LCDControllerYCoordinate * 160) + pixelX) * 4;

                        if (!isBitSet(spriteFlags, 7) || (bgPixels[index + 3] == bgPalette[0x00]))
                        {
                            DisplayPixels[index + 2] = color; // G
#if TINT    
                            if (DisplayPixels[index + 2] == 0x00) DisplayPixels[index + 2] = 0x30;
                            DisplayPixels[index + 3] = 0x00; // R
                            DisplayPixels[index + 1] = 0x00; // B
#else   
                            DisplayPixels[index + 3] = color; // R
                            DisplayPixels[index + 1] = color; // B
#endif  
                            DisplayPixels[index + 0] = 0xFF;  // A
                        }
                    }
                }
            }
        }
    }
}
