#ifndef ED_LOGGING_H_
#define ED_LOGGING_H_

#include <ostream>
#include <string>

namespace ed
{

namespace log
{

std::ostream& info();

void info(const char* str);

void info(const std::string& str);

std::ostream& warning();

void warning(const char* str);

void warning(const std::string& str);

std::ostream& error();

void error(const char* str);

void error(const std::string& str);

}

}

#endif
