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

    bool deserialize(ed::io::Reader& r, ed::Variant& v) const
    {
        geo::Pose3D p = geo::Pose3D::identity();

        if (r.readGroup("pos"))
        {
            r.readValue("x", p.t.x);
            r.readValue("y", p.t.y);
            r.readValue("z", p.t.z);
            r.endGroup();
        }

        if (r.readGroup("rot"))
        {
            r.readValue("xx", p.R.xx);
            r.readValue("xy", p.R.xy);
            r.readValue("xz", p.R.xz);
            r.readValue("yx", p.R.yx);
            r.readValue("yy", p.R.yy);
            r.readValue("yz", p.R.yz);
            r.readValue("zx", p.R.zx);
            r.readValue("zy", p.R.zy);
            r.readValue("zz", p.R.zz);
            r.endGroup();
        }

        v = p;
        return true;
    }

    bool serializable() const { return true; }

};

#endif
