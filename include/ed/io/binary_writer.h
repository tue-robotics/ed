#ifndef ERA_TUE_IO_BINARY_WRITER_H_
#define ERA_TUE_IO_BINARY_WRITER_H_

#include "era/io/writer.h"

#include <map>
#include <stdint.h>

namespace era
{

namespace io
{

class BinaryWriter : Writer
{

public:

    BinaryWriter(Data& data) : Writer(data) {}

    ~BinaryWriter() {}

    void writeGroup(const std::string& name)
    {
        writeLabel("g" + name);
    }

    void endGroup()
    {
        write('e');
    }

    void writeValue(const std::string& key, float f)
    {
        writeLabel("f" + key);
        write<float>(f);
    }

    void writeValue(const std::string& key, int i)
    {
        writeLabel("i" + key);
        write<int32_t>(i);
    }

    void writeValue(const std::string& key, const std::string& s)
    {
        writeLabel("s" + key);
        temp_data_.insert(temp_data_.end(), &s[0], &s[s.size() + 1]);
    }

    void writeValue(const std::string& key, const float* fs, std::size_t size)
    {
        writeLabel("F" + key);
        temp_data_.insert(temp_data_.end(), (const char*)fs, (const char*)fs + size * sizeof(float));
    }

    void writeValue(const std::string& key, const int* is, std::size_t size)
    {
        // ...
    }

    void writeValue(const std::string& key, const std::string* ss, std::size_t size)
    {
        // ...
    }

    void writeArray(const std::string& key)
    {
        writeLabel("a" + key);
    }

    void addArrayItem() {}
    void endArrayItem() {}

    void endArray()
    {
        write('e');
    }

    void finish()
    {
        // Add all labels
        for(std::vector<std::string>::const_iterator it = labels_.begin(); it != labels_.end(); ++it)
        {
            const std::string& s = *it;
            data_.insert(data_.end(), &s[0], &s[s.size() + 1]);
        }

        data_.insert(data_.end(), temp_data_.begin(), temp_data_.end());
    }

private:

    std::vector<std::string> labels_;
    std::map<std::string, std::size_t> label_to_index_;

    void writeLabel(const std::string& label)
    {
        std::map<std::string, std::size_t>::const_iterator it = label_to_index_.find(label);

        std::size_t idx;
        if (it != label_to_index_.end())
            idx = it->second;
        else
        {
            std::size_t idx = labels_.size();
            label_to_index_[label] = idx;
            labels_.push_back(label);
        }

        if (idx < 255)
        {
            temp_data_.push_back(idx);
        }
        else
        {
            temp_data_.push_back(255);
            write<uint32_t>(idx);
        }
    }

    template<typename T>
    inline void write(const T& d)
    {
        temp_data_.insert(temp_data_.end(), (char*)&d, (char*)&d + sizeof(T));
    }

    std::vector<unsigned char> temp_data_;

};

}

} // end namespace era

#endif
