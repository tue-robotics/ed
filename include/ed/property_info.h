#ifndef ED_PROPERTY_INFO_H_
#define ED_PROPERTY_INFO_H_

#include "ed/types.h"
#include "ed/variant.h"

namespace ed
{

class PropertyInfo
{

public:

    PropertyInfo() : idx(-1) {}

    virtual ~PropertyInfo() {}

    virtual bool serialize(const Variant& v, std::ostream& out) const { return false; }

    Idx idx;
    std::string name;
};

} // end namespace

#endif
