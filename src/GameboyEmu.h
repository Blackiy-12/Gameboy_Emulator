#pragma once

#include "CPU.h"

#include <SDL.h>
#include <string>

class GameboyEmu
{
public:
	GameboyEmu(int WindwoScale);

	~GameboyEmu();

public:
	void start(std::string CartPath);

private:
	SDL_Window* WindowPtr;

	SDL_Renderer* RendererPtr;

	SDL_Texture* WindowSpace;

	std::unique_ptr<CPU> CPUPtr;

	std::string CartPath;

	std::string BootPath;

	int GameboyX;

	int GameboyY;

	int WindowScale;

	bool Running;

private:

	void initSDL();

	void initEmu();

	void loop();

	void render();

	void processInput();
};

