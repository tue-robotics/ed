#ifndef ED_PROPERTY_INFO_H_
#define ED_PROPERTY_INFO_H_

#include "ed/types.h"
#include "ed/variant.h"
#include "ed/io/writer.h"
#include "ed/io/reader.h"

namespace ed
{

class PropertyInfo
{

public:

    PropertyInfo() {}

    virtual ~PropertyInfo() {}

    virtual void serialize(const Variant& v, io::Writer& out) const { }

    virtual bool deserialize(io::Reader& in, Variant& v) const { return false; }

    virtual bool serializable() const { return false; }

private:



};

} // end namespace

#endif
