#ifndef ED_IO_READER_H_
#define ED_IO_READER_H_

#include <string>
#include <vector>

#include <istream>

namespace ed
{

namespace io
{

class Reader
{

public:

    Reader() {}

    virtual ~Reader() {}

    virtual bool readGroup(const std::string& name) = 0;
    virtual bool endGroup() = 0;

    virtual bool readArray(const std::string& name) = 0;
    virtual bool endArray() = 0;

    virtual bool nextArrayItem() = 0;

    virtual bool readValue(const std::string&, float& f) = 0;
    virtual bool readValue(const std::string&, double& d) = 0;
    virtual bool readValue(const std::string&, int& i) = 0;
    virtual bool readValue(const std::string&, std::string& s) = 0;

    virtual bool ok() = 0;

    virtual std::string error() = 0;

};

}

} // end namespace era

#endif

