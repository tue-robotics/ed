#ifndef ED_IO_JSON_READER_H_
#define ED_IO_JSON_READER_H_

#include "ed/io/data.h"
#include "ed/io/reader.h"

#include <sstream>
#include <vector>

namespace ed::io
{

class JSONReader : public ed::io::Reader
{

public:
    JSONReader(const char* s);

    ~JSONReader() override;

    bool readGroup(const std::string& name) override;
    bool endGroup() override;

    bool readArray(const std::string& name) override;
    bool endArray() override;

    bool nextArrayItem() override;

    bool readValue(const std::string&, float& f) override;
    bool readValue(const std::string&, double& d) override;
    bool readValue(const std::string&, int& i) override;
    bool readValue(const std::string&, std::string& s) override;

    bool ok() override { return error_.empty(); }

    std::string error() override { return error_; }

private:
    Data data_;
    Node n_current_;
    std::vector<unsigned int> array_index_stack_;

    std::string error_;

    template <typename T> bool value(const std::string& key, T& value) const
    {
        const std::map<std::string, Node>& map = data_.maps[n_current_.idx];
        auto const it = map.find(key);
        if (it == map.end())
            return false;

        const Variant& v = data_.values[it->second.idx];

        return static_cast<bool>(v.getValue(value));
    }
};

} // namespace ed::io

#endif
