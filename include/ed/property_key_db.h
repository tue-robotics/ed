#ifndef ED_PROPERTY_KEY_DB_H_
#define ED_PROPERTY_KEY_DB_H_

#include "ed/types.h"
#include "ed/property_key.h"

#include <map>

namespace ed
{

class PropertyKeyDB
{

public:

    template<typename T>
    void registerProperty(const std::string& name, PropertyKey<T>& key)
    {
        std::map<std::string, Idx>::const_iterator it = name_to_idx_.find(name);

        key.name = name;

        if (it != name_to_idx_.end())
            key.idx = it->second;
        else
        {
            key.idx = name_to_idx_.size();
            name_to_idx_[name] = key.idx;
        }
    }

private:

    std::map<std::string, Idx> name_to_idx_;

};

} // end namespace

#endif
