#include "Logger.h"

#include <iostream>

void Logger::log(std::string& Message)
{
    std::cout << Message << std::endl;
}

void Logger::log(const char* Message)
{
    std::cout << Message << std::endl;
}
