#ifndef ED_PROPERTIES_STRING_INFO_H_
#define ED_PROPERTIES_STRING_INFO_H_

#include <ed/property_info.h>

class StringInfo : public ed::PropertyInfo
{

public:

    void serialize(const ed::Variant& v, ed::io::Writer& w) const
    {
        const std::string& s = v.getValue<std::string>();
        w.writeValue("value", s);
    }

    bool deserialize(ed::io::Reader& r, ed::Variant& v) const
    {
        std::string s;
        r.readValue("value", s);
        v = s;
        return true;
    }

    bool serializable() const { return true; }

};

#endif
