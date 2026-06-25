#ifndef ED_PROPERTY_KEY_H_
#define ED_PROPERTY_KEY_H_

#include "ed/types.h"

namespace ed
{

struct PropertyKeyDBEntry;

template <typename T> struct PropertyKey
{
    PropertyKey() = default;
    Idx idx{INVALID_IDX};

    const PropertyKeyDBEntry* entry{nullptr};

    [[nodiscard]]
    bool valid() const
    {
        return idx != INVALID_IDX;
    }
};

} // namespace ed

#endif
