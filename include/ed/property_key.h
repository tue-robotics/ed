#ifndef ED_PROPERTY_KEY_H_
#define ED_PROPERTY_KEY_H_

#include "ed/types.h"

namespace ed
{

template<typename T>
struct PropertyKey
{
    PropertyKey() : idx(-1) {}
    Idx idx;
    std::string name;

    bool valid() const { return idx != -1; }
};

} // end namespace

#endif
