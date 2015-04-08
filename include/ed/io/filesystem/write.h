#ifndef ED_IO_FILESYSTEM_WRITE_H_
#define ED_IO_FILESYSTEM_WRITE_H_

#include <string>

namespace ed
{

class Measurement;
class Entity;

bool write(const std::string& filename, const Measurement& msr);

bool write(const std::string &filename, const Entity& e);

}

#endif
