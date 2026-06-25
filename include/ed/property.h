#ifndef ED_PROPERTY_H_
#define ED_PROPERTY_H_

#include "ed/types.h"
#include "ed/variant.h"

namespace ed
{

struct PropertyKeyDBEntry;

struct Property
{
    Property() = default;

    Variant value;
    const PropertyKeyDBEntry* entry = nullptr;
    unsigned long revision = -1;
};

} // namespace ed

#endif
