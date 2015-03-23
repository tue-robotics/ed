#ifndef ERA_TUE_CONFIGURATION_DATA_H_
#define ERA_TUE_CONFIGURATION_DATA_H_

#include "ed/io/variant.h"

#include <vector>
#include <map>
#include <string>

namespace ed
{

namespace io
{

// ----------------------------------------------------------------------------------------------------

enum NodeType
{
    ARRAY,
    MAP,
    VALUE
};

// ----------------------------------------------------------------------------------------------------

struct Node
{
    Node() {}
    Node(unsigned int idx_, NodeType type_) : idx(idx_), type(type_) {}

    unsigned int idx;
    NodeType type;
};

// ----------------------------------------------------------------------------------------------------

struct Data
{
    std::vector<std::vector<Node> > arrays;
    std::vector<unsigned int> array_parents;

    std::vector<std::map<std::string, Node> > maps;
    std::vector<unsigned int> map_parents;

    std::vector<Variant> values;
};

}

}

#endif
