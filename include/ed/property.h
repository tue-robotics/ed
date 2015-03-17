#ifndef ED_PROPERTY_H_
#define ED_PROPERTY_H_

#include "ed/types.h"
#include "ed/variant.h"

namespace ed
{

class PropertyKeyDBEntry;

struct Property
{
    Property() : entry(0), revision(-1) {}

    Variant value;
    const PropertyKeyDBEntry* entry;
    unsigned long revision;
};

} // end namespace

#endif
