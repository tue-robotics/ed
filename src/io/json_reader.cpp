#include "ed/io/json_reader.h"

#include "rapidjson/reader.h"
#include "ed/io/data_writer.h"

using namespace std;
using namespace rapidjson;

namespace ed
{

namespace io
{

// ----------------------------------------------------------------------------------------------------

struct MyHandler {

    MyHandler(DataWriter& w_) : w(w_)
    {
    }

    bool Null() { return true; }

    bool Bool(bool b) { int i = b; w.setValue(key, i); ; return true; }

    bool Int(int i) { w.setValue(key, i); return true; }

    bool Uint(unsigned u) { w.setValue(key, (int)u); return true; }

    bool Int64(int64_t i) { w.setValue(key, (int)i); return true; }

    bool Uint64(uint64_t u) { w.setValue(key, (int)u); return true; }

    bool Double(double d) { w.setValue(key, d); return true; }

    bool String(const char* str, rapidjson::SizeType length, bool copy)
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

    bool Key(const char* str, rapidjson::SizeType length, bool copy) { key = str; return true; }

    bool EndObject(rapidjson::SizeType memberCount)
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

    bool EndArray(rapidjson::SizeType elementCount)
    {
        w.endArray();
        stack.pop_back();
        return true;
    }

    DataWriter& w;
    std::string key;
    std::vector<unsigned char> stack;

};

// ----------------------------------------------------------------------------------------------------

JSONReader::JSONReader(const char* s) : n_current_(Node(0, MAP))
{
    DataWriter w(data_);
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

JSONReader::~JSONReader()
{
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::readGroup(const std::string& key)
{
    std::map<std::string, Node>& map = data_.maps[n_current_.idx];
    std::map<std::string, Node>::const_iterator it = map.find(key);
    if (it == map.end())
        return false;

    n_current_ = it->second;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::endGroup()
{
    n_current_.idx = data_.map_parents[n_current_.idx];
    n_current_.type = MAP;
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::readArray(const std::string& key)
{
    std::map<std::string, Node>& map = data_.maps[n_current_.idx];
    std::map<std::string, Node>::const_iterator it = map.find(key);
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

    unsigned int& i_next_array_item_ = array_index_stack_.back();
    array_index_stack_.pop_back();

    if (n_current_.type != ARRAY && i_next_array_item_ > 0)
        n_current_.idx = data_.array_parents[data_.map_parents[n_current_.idx]];
    else
        n_current_.idx = data_.array_parents[n_current_.idx];

    n_current_.type = MAP;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool JSONReader::nextArrayItem()
{
    if (array_index_stack_.empty())
        return false;

    unsigned int& i_next_array_item_ = array_index_stack_.back();

    if (n_current_.type != ARRAY)
    {
        if (i_next_array_item_ == 0)
            return false;

        n_current_.idx = data_.map_parents[n_current_.idx];
        n_current_.type = ARRAY;
    }

    std::vector<Node>& array = data_.arrays[n_current_.idx];

    if (i_next_array_item_ >= array.size())
        return false;

    n_current_ = array[i_next_array_item_];
    ++i_next_array_item_;

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

}

}
