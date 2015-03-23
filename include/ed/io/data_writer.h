#ifndef ERA_TUE_CONFIGURATION_WRITER_H_
#define ERA_TUE_CONFIGURATION_WRITER_H_

#include "ed/io/data.h"

#include <iostream>

namespace ed
{

namespace io
{

class DataWriter
{

public:

    DataWriter(Data& cfg, const Node& n = Node()) : data_(cfg), n_current_(n)
    {
        if (data_.maps.empty())
        {
            data_.maps.push_back(std::map<std::string, Node>());
            data_.map_parents.push_back(-1);
            n_current_.idx = 0;
            n_current_.type = MAP;
        }
    }

    void setValue(const std::string& key, const Variant& value)
    {
        data_.values.push_back(value);
        data_.maps[n_current_.idx][key] = Node(data_.values.size() - 1, VALUE);
    }

    void writeGroup(const std::string& key)
    {
        data_.maps.push_back(std::map<std::string, Node>());
        data_.map_parents.push_back(n_current_.idx);

        data_.maps[n_current_.idx][key] = Node(data_.maps.size() - 1, MAP);

        // Change current node to new group
        n_current_.idx = data_.maps.size() - 1;
        n_current_.type = MAP;
    }

    void endGroup()
    {
        n_current_.idx = data_.map_parents[n_current_.idx];
        n_current_.type = MAP;
    }

    void writeArray(const std::string& key)
    {
        data_.arrays.push_back(std::vector<Node>());
        data_.array_parents.push_back(n_current_.idx);

        data_.maps[n_current_.idx][key] = Node(data_.arrays.size() - 1, ARRAY);

        // Change current node to new array
        n_current_.idx = data_.arrays.size() - 1;
        n_current_.type = ARRAY;
    }

    void endArray()
    {
        n_current_.idx = data_.array_parents[n_current_.idx];
        n_current_.type = MAP;
    }

    bool addArrayItem()
    {
        if (n_current_.type != ARRAY)
            return false;

        std::vector<Node>& array = data_.arrays[n_current_.idx];

        data_.maps.push_back(std::map<std::string, Node>());
        data_.map_parents.push_back(n_current_.idx);

        array.push_back(Node(data_.maps.size() - 1, MAP));

        n_current_.idx = data_.maps.size() - 1;
        n_current_.type = MAP;
        return true;
    }

    bool endArrayItem()
    {
        n_current_.idx = data_.map_parents[n_current_.idx];
        n_current_.type = ARRAY;

        return true;
    }

private:

    Data& data_;
    Node n_current_;

};

}

}

#endif
