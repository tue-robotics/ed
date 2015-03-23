#ifndef ED_EXAMPLES_CUSTOM_PROPERTIES_PLUGIN_COUNTER_INFO_H_
#define ED_EXAMPLES_CUSTOM_PROPERTIES_PLUGIN_COUNTER_INFO_H_

#include <ed/property_info.h>

class CounterInfo : public ed::PropertyInfo
{

public:

    void serialize(const ed::Variant& v, ed::io::Writer& w) const
    {
        int counter = v.getValue<int>();
        w.writeValue("value", counter);
    }

    bool deserialize(ed::io::Reader& r, ed::Variant& v) const
    {
        int counter;
        r.readValue("value", counter);
        v.setValue<int>(counter);

        return true;
    }

    bool serializable() const { return true; }

};

#endif
