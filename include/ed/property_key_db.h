#ifndef ED_PROPERTY_KEY_DB_H_
#define ED_PROPERTY_KEY_DB_H_

#include "ed/types.h"
#include "ed/property_key.h"
#include "ed/property_info.h"

#include <map>

namespace ed
{

class PropertyKeyDB
{

public:

    ~PropertyKeyDB()
    {
        for(std::map<std::string, PropertyInfo*>::iterator it = name_to_info_.begin(); it != name_to_info_.end(); ++it)
        {
            delete it->second;
        }
    }

    template<typename T>
    void registerProperty(const std::string& name, PropertyKey<T>& key, PropertyInfo* info = 0)
    {
        std::map<std::string, PropertyInfo*>::const_iterator it = name_to_info_.find(name);
        if (it != name_to_info_.end())
        {            
            key.info = it->second;
            key.idx = it->second->idx;

            // Make sure info is deleted to prevent mem leaks
            delete info;
        }
        else
        {
            if (!info)
                info = new PropertyInfo;

            info->name = name;
            info->idx = name_to_info_.size();
            name_to_info_[name] = info;

            key.info = info;
            key.idx = info->idx;
        }
    }

private:

    std::map<std::string, PropertyInfo*> name_to_info_;

};

} // end namespace

#endif
