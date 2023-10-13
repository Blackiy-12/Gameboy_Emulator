#pragma once

#include <string>

class Logger
{
public:
	static void log(std::string& Message);

	static void log(const char* Message);
};

