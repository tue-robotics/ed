#ifndef ED_IO_FILESYSTEM_READ_H_
#define ED_IO_FILESYSTEM_READ_H_

#include <string>

namespace ed
{

class Measurement;

bool read(const std::string& filename, Measurement& msr);

}

#endif
