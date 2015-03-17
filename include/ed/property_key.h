#ifndef ED_PROPERTY_KEY_H_
#define ED_PROPERTY_KEY_H_

#include "ed/types.h"

namespace ed
{

class PropertyInfo;

template<typename T>
struct PropertyKey
{
    PropertyKey() : idx(-1), info(0) {}
    Idx idx;
    const PropertyInfo* info;

    bool valid() const { return idx != -1; }
};

} // end namespace

#endif
