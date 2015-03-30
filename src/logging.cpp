#include "ed/logging.h"

#include <iostream>
#include <pthread.h>

namespace
{
    std::string prefix()
    {
        std::string ps = "[ED] ";

        // Add thread name
        char name[1000];
        size_t name_size = 1000;
        if (pthread_getname_np(pthread_self(), name, name_size) == 0)
            ps += "(" + std::string(name) + ") ";

        return ps;
    }
}

namespace ed
{

namespace log
{

// ----------------------------------------------------------------------------------------------------

std::ostream& info()
{
    std::cout << "\e[1;37m" << prefix() << " \e[0m";
    return std::cout;
}

// ----------------------------------------------------------------------------------------------------

void info(const char* str)
{
    std::cout << "\e[1;37m" << prefix() << str << " \e[0m" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

std::ostream& warning()
{
    std::cout << "\e[1;33m" << prefix() << "Warning: \e[0m";
    return std::cout;
}

// ----------------------------------------------------------------------------------------------------

void warning(const char* str)
{
    std::cout << "\e[1;33m" << prefix() << "Warning: \e[0m" << str << std::endl;
}

// ----------------------------------------------------------------------------------------------------

std::ostream& error()
{
    std::cout << "\e[1;31m" << prefix() << "Error: \e[0m";
    return std::cout;
}

// ----------------------------------------------------------------------------------------------------

void error(const char* str)
{
    std::cout << "\e[1;31m" << prefix() << "Error: \e[0m" << str << std::endl;
}

}

}
