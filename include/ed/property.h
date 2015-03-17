#ifndef ED_PROPERTY_H_
#define ED_PROPERTY_H_

#include "ed/types.h"
#include "ed/variant.h"

namespace ed
{

class PropertyInfo;

struct Property
{
    Property() : info(0), revision(-1) {}

    Variant value;
    const PropertyInfo* info;
    unsigned long revision;
};

} // end namespace

#endif
