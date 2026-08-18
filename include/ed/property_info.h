#ifndef ED_PROPERTY_INFO_H_
#define ED_PROPERTY_INFO_H_

#include "ed/io/reader.h"
#include "ed/io/writer.h"
#include "ed/types.h"
#include "ed/variant.h"

namespace ed
{

class PropertyInfo
{

public:
    PropertyInfo() = default;

    virtual ~PropertyInfo() = default;

    virtual void serialize(const Variant& /*v*/, io::Writer& /*out*/) const {}

    virtual bool deserialize(io::Reader& /*in*/, Variant& /*v*/) const { return false; }

    [[nodiscard]]
    virtual bool serializable() const
    {
        return false;
    }

private:
};

} // namespace ed

#endif
