#ifndef ERA_TUE_CONFIGURATION_WRITER_H_
#define ERA_TUE_CONFIGURATION_WRITER_H_

#include "ed/io/data.h"

#include <iostream>

namespace ed::io
{

class DataWriter
{

public:
    explicit DataWriter(Data& cfg, const Node& n = Node()) : data_(cfg), n_current_(n)
    {
        if (data_.maps.empty())
        {
            data_.maps.emplace_back();
            data_.map_parents.push_back(-1);
            n_current_.idx = 0;
            n_current_.type = NodeType::MAP;
        }
    }

    void setValue(const std::string& key, const Variant& value)
    {
        data_.values.push_back(value);
        data_.maps[n_current_.idx][key] = Node(data_.values.size() - 1, NodeType::VALUE);
    }

    void writeGroup(const std::string& key)
    {
        data_.maps.emplace_back();
        data_.map_parents.push_back(n_current_.idx);

        data_.maps[n_current_.idx][key] = Node(data_.maps.size() - 1, NodeType::MAP);

        // Change current node to new group
        n_current_.idx = data_.maps.size() - 1;
        n_current_.type = NodeType::MAP;
    }

    void endGroup()
    {
        n_current_.idx = data_.map_parents[n_current_.idx];
        n_current_.type = NodeType::MAP;
    }

    void writeArray(const std::string& key)
    {
        data_.arrays.emplace_back();
        data_.array_parents.push_back(n_current_.idx);

        data_.maps[n_current_.idx][key] = Node(data_.arrays.size() - 1, NodeType::ARRAY);

        // Change current node to new array
        n_current_.idx = data_.arrays.size() - 1;
        n_current_.type = NodeType::ARRAY;
    }

    void endArray()
    {
        n_current_.idx = data_.array_parents[n_current_.idx];
        n_current_.type = NodeType::MAP;
    }

    bool addArrayItem()
    {
        if (n_current_.type != NodeType::ARRAY)
            return false;

        std::vector<Node>& array = data_.arrays[n_current_.idx];

        data_.maps.emplace_back();
        data_.map_parents.push_back(n_current_.idx);

        array.emplace_back(data_.maps.size() - 1, NodeType::MAP);

        n_current_.idx = data_.maps.size() - 1;
        n_current_.type = NodeType::MAP;
        return true;
    }

    bool endArrayItem()
    {
        n_current_.idx = data_.map_parents[n_current_.idx];
        n_current_.type = NodeType::ARRAY;

        return true;
    }

private:
    // The writer writes into a Data owned by the caller.
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    Data& data_;
    Node n_current_;
};

} // namespace ed::io

#endif
