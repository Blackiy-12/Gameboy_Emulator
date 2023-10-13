#include "GameboyEmu.h"

#include <functional>

const int OriginalWidth = 160;
const int OriginalHeight = 144;

// 60 FPS
const double TimePerFrame = 1000.0 / 60.0;

// The number of CPU cycles per frame
const unsigned int CyclesPerFrame = 70224;

#define JOYPAD_NONE             0

#define JOYPAD_INPUT_DOWN       1 << 3
#define JOYPAD_INPUT_UP         1 << 2
#define JOYPAD_INPUT_LEFT       1 << 1
#define JOYPAD_INPUT_RIGHT      1 << 0

#define JOYPAD_BUTTONS_START    1 << 3
#define JOYPAD_BUTTONS_SELECT   1 << 2
#define JOYPAD_BUTTONS_B        1 << 1
#define JOYPAD_BUTTONS_A        1 << 0

GameboyEmu::GameboyEmu(int WindwoScale)
	:   WindowScale(WindwoScale),
        Running(true)

{
}

GameboyEmu::~GameboyEmu()
{

    CPUPtr.reset();
    
    SDL_DestroyTexture(WindowSpace);
    SDL_DestroyRenderer(RendererPtr);
    SDL_DestroyWindow(WindowPtr);

    SDL_Quit();
}

void GameboyEmu::start(std::string CartPath)
{
    this->CartPath = CartPath;

	initSDL();

	initEmu();

    loop();
}

void GameboyEmu::initSDL()
{
	SDL_Init(SDL_INIT_EVERYTHING);

	WindowPtr = SDL_CreateWindow("Gameboy", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, OriginalWidth * WindowScale, OriginalHeight * WindowScale, SDL_WINDOW_SHOWN);

	RendererPtr = SDL_CreateRenderer(WindowPtr, -1, SDL_RENDERER_ACCELERATED);
	
	WindowSpace = SDL_CreateTexture(RendererPtr, SDL_PIXELFORMAT_RGBA8888, SDL_TEXTUREACCESS_STREAMING, OriginalWidth, OriginalHeight);
}

void GameboyEmu::initEmu()
{
	this->CPUPtr = std::make_unique<CPU>();

	this->CPUPtr->initialize();

	if (BootPath.empty() == true)
		this->CPUPtr->loadCartridge(CartPath.c_str(), nullptr);
	else	
		this->CPUPtr->loadCartridge(CartPath.c_str(), BootPath.c_str());


}

void GameboyEmu::loop()
{

    CPUPtr->setVSyncCallback(std::bind(&GameboyEmu::render, this));

    unsigned int cycles = 0;
    

    while (Running)
    {

        Uint64 frameStart = SDL_GetTicks();
        SDL_Event Event;
        while (SDL_PollEvent(&Event) != 0)
        {
            if (Event.type == SDL_QUIT)
            {
                Running = false;
                CPUPtr->setVSyncCallback(nullptr);
                return;
            }
        }

        processInput();

        while (cycles < CyclesPerFrame)
        {
            cycles += CPUPtr->step();
        }

        cycles -= CyclesPerFrame;

        Uint64 frameEnd = SDL_GetTicks();

        if (frameEnd - frameStart < TimePerFrame)
        {
            SDL_Delay(TimePerFrame - frameEnd + frameStart);
        }
    }
}
void GameboyEmu::render()
{
    // Clear window
    SDL_SetRenderDrawColor(RendererPtr, 0xFF, 0xFF, 0xFF, 0xFF);
    SDL_RenderClear(RendererPtr);

    uint8_t* Pixels;
    int Pitch = 0;
    SDL_LockTexture(WindowSpace, nullptr, (void**)&Pixels, &Pitch);

    // Render Game
    uint8_t* Data = CPUPtr->getCurrentFrame();
    memcpy(Pixels, Data, 160 * 144 * 4);

    SDL_UnlockTexture(WindowSpace);

    SDL_RenderCopy(RendererPtr, WindowSpace, NULL, NULL);

    // Update window
    SDL_RenderPresent(RendererPtr);
}

void GameboyEmu::processInput()
{
    SDL_PumpEvents();
    const Uint8* keys = SDL_GetKeyboardState(NULL);
    uint8_t input = JOYPAD_NONE;
    uint8_t buttons = JOYPAD_NONE;

    if (keys[SDL_SCANCODE_W])
    {
        input |= JOYPAD_INPUT_UP;
    }

    if (keys[SDL_SCANCODE_A])
    {
        input |= JOYPAD_INPUT_LEFT;
    }

    if (keys[SDL_SCANCODE_S])
    {
        input |= JOYPAD_INPUT_DOWN;
    }

    if (keys[SDL_SCANCODE_D])
    {
        input |= JOYPAD_INPUT_RIGHT;
    }

    if (keys[SDL_SCANCODE_K])
    {
        buttons |= JOYPAD_BUTTONS_A;
    }

    if (keys[SDL_SCANCODE_L])
    {
        buttons |= JOYPAD_BUTTONS_B;
    }

    if (keys[SDL_SCANCODE_N])
    {
        buttons |= JOYPAD_BUTTONS_START;
    }

    if (keys[SDL_SCANCODE_M])
    {
        buttons |= JOYPAD_BUTTONS_SELECT;
    }

    CPUPtr->setInput(input, buttons);

}
