#ifndef ED_IO_JSON_READER_H_
#define ED_IO_JSON_READER_H_

#include "ed/io/reader.h"
#include "ed/io/data.h"

#include <sstream>

namespace ed
{

namespace io
{

class JSONReader : public Reader
{

public:

    JSONReader(const char* s);

    virtual ~JSONReader();

    bool readGroup(const std::string& name);
    bool endGroup();

    bool readArray(const std::string& name);
    bool endArray();

    bool nextArrayItem();

    bool readValue(const std::string&, float& f);
    bool readValue(const std::string&, double& d);
    bool readValue(const std::string&, int& i);
    bool readValue(const std::string&, std::string& s);

    bool ok() { return error_.empty(); }

    std::string error() { return error_; }

private:

    Data data_;
    Node n_current_;
    std::vector<unsigned int> array_index_stack_;

    std::string error_;

    template<typename T>
    bool value(const std::string& key, T& value) const
    {
        const std::map<std::string, Node>& map = data_.maps[n_current_.idx];
        std::map<std::string, Node>::const_iterator it = map.find(key);
        if (it == map.end())
            return false;

        const Variant& v = data_.values[it->second.idx];

        if (!v.getValue(value))
            return false;

        return true;
    }

};

}

}

#endif
