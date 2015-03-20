#ifndef ED_EXAMPLES_CUSTOM_PROPERTIES_PLUGIN_POSE_INFO_H_
#define ED_EXAMPLES_CUSTOM_PROPERTIES_PLUGIN_POSE_INFO_H_

#include <ed/property_info.h>

class PoseInfo : public ed::PropertyInfo
{

public:

    void serialize(const ed::Variant& v, ed::io::Writer& w) const
    {
        const geo::Pose3D& p = v.getValue<geo::Pose3D>();

        w.writeGroup("pos");
        w.writeValue("x", p.t.x);
        w.writeValue("y", p.t.y);
        w.writeValue("z", p.t.z);
        w.endGroup();

        w.writeGroup("rot");
        w.writeValue("xx", p.R.xx);
        w.writeValue("xy", p.R.xy);
        w.writeValue("xz", p.R.xz);
        w.writeValue("yx", p.R.yx);
        w.writeValue("yy", p.R.yy);
        w.writeValue("yz", p.R.yz);
        w.writeValue("zx", p.R.zx);
        w.writeValue("zy", p.R.zy);
        w.writeValue("zz", p.R.zz);
        w.endGroup();
    }

    bool serializable() const { return true; }

};

#endif
