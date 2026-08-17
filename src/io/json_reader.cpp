#include "ed/io/json_reader.h"

#include "ed/io/data.h"
#include "ed/io/data_writer.h"
#include "rapidjson/rapidjson.h"
#include "rapidjson/reader.h"
#include <cstdint>
#include <map>
#include <vector>

namespace ed::io
{

// ----------------------------------------------------------------------------------------------------

// The member names below are dictated by rapidjson's SAX Handler concept and cannot be renamed.
// NOLINTBEGIN(readability-identifier-naming)
struct MyHandler
{

    explicit MyHandler(ed::io::DataWriter& w_) : w(w_) {}

    static bool Null() { return true; }

    bool Bool(bool b)
    {
        int const i = b;
        w.setValue(key, i);
        ;
        return true;
    }

    bool Int(int i)
    {
        w.setValue(key, i);
        return true;
    }

    bool Uint(unsigned u)
    {
        w.setValue(key, static_cast<int>(u));
        return true;
    }

    bool Int64(int64_t i)
    {
        w.setValue(key, static_cast<int>(i));
        return true;
    }

    bool Uint64(uint64_t u)
    {
        w.setValue(key, static_cast<int>(u));
        return true;
    }

    bool Double(double d)
    {
        w.setValue(key, d);
        return true;
    }

    bool RawNumber(const char* str, rapidjson::SizeType /*len*/, bool /*copy*/)
    {
        w.setValue(key, str);
        return true;
    }
    bool String(const char* str, rapidjson::SizeType /*length*/, bool /*copy*/)
    {
        w.setValue(key, str);
        return true;
    }
    bool StartObject()
    {
        if (stack.empty())
        {
            stack.push_back('g');
            return true;
        }

        if (stack.back() == 'a')
        {
            stack.push_back('i');
            w.addArrayItem();
        }
        else
        {
            stack.push_back('g');
            w.writeGroup(key);
        }

        return true;
    }

    bool Key(const char* str, rapidjson::SizeType /*length*/, bool /*copy*/)
    {
        key = str;
        return true;
    }

    bool EndObject(rapidjson::SizeType /*memberCount*/)
    {
        if (stack.empty())
            return true;

        if (stack.back() == 'i')
            w.endArrayItem();
        else
            w.endGroup();

        stack.pop_back();
        return true;
    }

    bool StartArray()
    {
        w.writeArray(key);
        stack.push_back('a');
        return true;
    }

    bool EndArray(rapidjson::SizeType /*elementCount*/)
    {
        w.endArray();
        stack.pop_back();
        return true;
    }

    // The SAX handler writes into a writer owned by the caller.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    ed::io::DataWriter& w;
    std::string key;
    std::vector<unsigned char> stack;
};
// NOLINTEND(readability-identifier-naming)

// ----------------------------------------------------------------------------------------------------

JSONReader::JSONReader(const char* s) : n_current_(Node(0, NodeType::MAP))
{
    ed::io::DataWriter w(data_);
    MyHandler handler(w);
    rapidjson::StringStream ss(s);

    rapidjson::Reader reader;
    reader.Parse(ss, handler);

    if (reader.HasParseError())
    {
        error_ = "Could not parse string";
    }
}

// ----------------------------------------------------------------------------------------------------

JSONReader::~JSONReader() = default;

// ----------------------------------------------------------------------------------------------------

bool JSONReader::readGroup(const std::string& key)
{
    std::map<std::string, Node>& map = data_.maps[n_current_.idx];
    auto const it = map.find(key);
    if (it == map.end())
        return false;

    n_current_ = it->second;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::endGroup()
{
    n_current_.idx = data_.map_parents[n_current_.idx];
    n_current_.type = NodeType::MAP;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::readArray(const std::string& key)
{
    std::map<std::string, Node>& map = data_.maps[n_current_.idx];
    auto const it = map.find(key);
    if (it == map.end())
        return false;

    n_current_ = it->second;
    array_index_stack_.push_back(0);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::endArray()
{
    if (array_index_stack_.empty())
        return false;

    unsigned int const& i_next_array_item = array_index_stack_.back();
    array_index_stack_.pop_back();

    if (n_current_.type != NodeType::ARRAY && i_next_array_item > 0)
        n_current_.idx = data_.array_parents[data_.map_parents[n_current_.idx]];
    else
        n_current_.idx = data_.array_parents[n_current_.idx];

    n_current_.type = NodeType::MAP;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::nextArrayItem()
{
    if (array_index_stack_.empty())
        return false;

    unsigned int& i_next_array_item = array_index_stack_.back();

    if (n_current_.type != NodeType::ARRAY)
    {
        if (i_next_array_item == 0)
            return false;

        n_current_.idx = data_.map_parents[n_current_.idx];
        n_current_.type = NodeType::ARRAY;
    }

    std::vector<Node>& array = data_.arrays[n_current_.idx];

    if (i_next_array_item >= array.size())
        return false;

    n_current_ = array[i_next_array_item];
    ++i_next_array_item;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::readValue(const std::string& key, float& f)
{
    return value<float>(key, f);
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::readValue(const std::string& key, double& d)
{
    return value<double>(key, d);
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::readValue(const std::string& key, int& i)
{
    return value<int>(key, i);
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::readValue(const std::string& key, std::string& s)
{
    return value<std::string>(key, s);
}

} // namespace ed::io
