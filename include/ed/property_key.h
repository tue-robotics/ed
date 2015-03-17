#ifndef ED_PROPERTY_KEY_H_
#define ED_PROPERTY_KEY_H_

#include "ed/types.h"

namespace ed
{

class PropertyKeyDBEntry;

template<typename T>
struct PropertyKey
{
    PropertyKey() : idx(-1), entry(0) {}
    Idx idx;

    const PropertyKeyDBEntry* entry;

    bool valid() const { return idx != -1; }
};

} // end namespace

#endif
