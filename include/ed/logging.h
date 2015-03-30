#ifndef ED_LOGGING_H_
#define ED_LOGGING_H_

#include <ostream>

namespace ed
{

namespace log
{

std::ostream& info();

void info(const char* str);

std::ostream& warning();

void warning(const char* str);

std::ostream& error();

void error(const char* str);

}

}

#endif
