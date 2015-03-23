#ifndef ED_PROPERTIES_SHAPE_INFO_H_
#define ED_PROPERTIES_SHAPE_INFO_H_

#include <ed/property_info.h>

#include <geolib/Shape.h>

class ShapeInfo : public ed::PropertyInfo
{

public:

    void serialize(const ed::Variant& v, ed::io::Writer& w) const
    {
        const geo::ShapeConstPtr& s = v.getValue<geo::ShapeConstPtr>();

        w.writeArray("vertices");
        const std::vector<geo::Vector3>& vertices = s->getMesh().getPoints();
        for(std::vector<geo::Vector3>::const_iterator it = vertices.begin(); it != vertices.end(); ++it)
        {
            w.addArrayItem();
            w.writeValue("x", it->x);
            w.writeValue("y", it->y);
            w.writeValue("z", it->z);
            w.endArrayItem();
        }
        w.endArray();

        w.writeArray("triangles");
        const std::vector<geo::TriangleI>& triangles = s->getMesh().getTriangleIs();
        for(std::vector<geo::TriangleI>::const_iterator it = triangles.begin(); it != triangles.end(); ++it)
        {
            w.addArrayItem();
            w.writeValue("i1", it->i1_);
            w.writeValue("i2", it->i2_);
            w.writeValue("i3", it->i3_);
            w.endArrayItem();
        }

        w.endArray();
    }

    bool deserialize(ed::io::Reader& r, ed::Variant& v) const
    {
        geo::Mesh mesh;

        if (r.readArray("vertices"))
        {
            while(r.nextArrayItem())
            {
                float x, y, z;
                r.readValue("x", x);
                r.readValue("y", y);
                r.readValue("z", z);
                mesh.addPoint(geo::Vector3(x, y, z));
            }

            r.endArray();
        }

        if (r.readGroup("triangles"))
        {
            while(r.nextArrayItem())
            {

                int i1, i2, i3;
                r.readValue("i1", i1);
                r.readValue("i2", i2);
                r.readValue("i3", i3);
                mesh.addTriangle(i1, i2, i3);
            }
        }

        geo::ShapePtr shape(new geo::Shape);
        shape->setMesh(mesh);

        v = geo::ShapeConstPtr(shape);
        return true;
    }

    bool serializable() const { return true; }

};

#endif
