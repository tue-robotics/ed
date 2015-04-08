#ifndef ED_IO_FILESYSTEM_READ_H_
#define ED_IO_FILESYSTEM_READ_H_

#include <string>

namespace ed
{

class Measurement;
class UpdateRequest;

bool read(const std::string& filename, Measurement& msr);

bool readEntity(const std::string& filename, UpdateRequest& req);

}

#endif
