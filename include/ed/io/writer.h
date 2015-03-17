#ifndef ED_IO_WRITER_H_
#define ED_IO_WRITER_H_

#include <string>
#include <vector>

//#include "ed/io/data.h"

#include <ostream>

namespace ed
{

namespace io
{

class Writer
{

public:

    Writer(std::ostream& out) : out_(out) {}

    virtual ~Writer() {}

    virtual void writeGroup(const std::string& name) = 0;
    virtual void endGroup() = 0;

    virtual void writeValue(const std::string& key, float f) = 0;
    virtual void writeValue(const std::string& key, double d) = 0;
    virtual void writeValue(const std::string& key, int i) = 0;
    virtual void writeValue(const std::string& key, const std::string& s) = 0;

    virtual void writeValue(const std::string& key, const float* fs, std::size_t size) = 0;
    virtual void writeValue(const std::string& key, const int* is, std::size_t size) = 0;
    virtual void writeValue(const std::string& key, const std::string* ss, std::size_t size) = 0;

    virtual void writeValue(const std::string& key, const std::vector<float>& fs) { writeValue(key, &fs[0], fs.size()); }
    virtual void writeValue(const std::string& key, const std::vector<int>& is) { writeValue(key, &is[0], is.size()); }
    virtual void writeValue(const std::string& key, const std::vector<std::string>& ss) { writeValue(key, &ss[0], ss.size()); }

    virtual void writeArray(const std::string& key) = 0;
    virtual void addArrayItem() = 0;
    virtual void endArrayItem() = 0;
    virtual void endArray() = 0;

    virtual void finish() {}

protected:

    std::ostream& out_;

};

}

} // end namespace era

#endif

