#ifndef ED_PROPERTY_H_
#define ED_PROPERTY_H_

#include "ed/types.h"
#include "ed/variant.h"
#include <cstdint>

namespace ed
{

struct PropertyKeyDBEntry;

struct Property
{
    Property() = default;

    Variant value;
    const PropertyKeyDBEntry* entry = nullptr;
    std::uint64_t revision = -1;
};

} // namespace ed

#endif
