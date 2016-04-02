#include "ed/serialization/serialization.h"
#include "ed/mask.h"

#include "ed/world_model.h"
#include "ed/update_request.h"
#include "ed/entity.h"
#include "ed/convex_hull_calc.h"

#include <tue/config/reader.h>
#include <tue/config/writer.h>

#include <geolib/Shape.h>
#include <geolib/Box.h>

#include <tue/config/configuration.h>
#include <tue/config/loaders/yaml.h>

namespace ed
{

// ----------------------------------------------------------------------------------------------------

//void serialize(const WorldModel& wm, ed::io::Writer& w, unsigned long since_revision)
//{

//}

//// ----------------------------------------------------------------------------------------------------

//void serialize(const Entity& wm, ed::io::Writer& w, unsigned long since_revision)
//{

//}

// ----------------------------------------------------------------------------------------------------

bool deserialize(io::Reader &r, UpdateRequest& req)
{
    if (r.readArray("entities"))
    {
        while(r.nextArrayItem())
        {
            std::string id;
            if (!r.readValue("id", id))
            {
                std::cout << "Deserialze: Entities should have field 'id'" << std::endl;
                return false;
            }

            std::string type;
            if (r.readValue("type", type))
            {
                req.setType(id, type);
            }

            double existence_prob;
            if (r.readValue("existence_prob", existence_prob))
            {
                req.setExistenceProbability(id, existence_prob);
            }

            double timestamp = 0;
            if (r.readGroup("timestamp"))
            {
                ed::deserializeTimestamp(r, timestamp);
                req.setLastUpdateTimestamp(id, timestamp);
                r.endGroup();
            }

            geo::Pose3D pose = geo::Pose3D::identity();
            if (r.readGroup("pose"))
            {
                ed::deserialize(r, pose);
                req.setPose(id, pose);
                r.endGroup();
            }

            if (r.readGroup("convex_hull"))
            {
                ed::ConvexHull chull;
                ed::deserialize(r, chull);

                ed::convex_hull::calculateEdgesAndNormals(chull);
                req.setConvexHullNew(id, chull, pose, timestamp);
                r.endGroup();
            }

            if (r.readGroup("mesh"))
            {
                geo::ShapePtr shape(new geo::Shape);
                ed::deserialize(r, *shape);
                req.setShape(id, shape);
                r.endGroup();
            }

            std::string data_str;
            if (r.readValue("data", data_str))
            {
                std::replace(data_str.begin(), data_str.end(), '|', '"');
                std::replace(data_str.begin(), data_str.end(), '^', '\n');

                tue::Configuration cfg;
                if (tue::config::loadFromYAMLString(data_str, cfg))
                    req.addData(id, cfg.data());
            }

//            if (r.readArray("properties"))
//            {
//                while(r.nextArrayItem())
//                {
//                    std::string prop_name;
//                    if (!r.readValue("name", prop_name))
//                        continue;

//                    const ed::PropertyKeyDBEntry* entry = data.world.getPropertyInfo(prop_name);
//                    if (!entry)
//                    {
//                        error += "For entity '" + id + "': unknown property '" + prop_name +"'.\n";
//                        continue;
//                    }

//                    if (!entry->info->serializable())
//                    {
//                        error += "For entity '" + id + "': property '" + prop_name +"' is not serializable.\n";
//                        continue;
//                    }

//                    ed::Variant value;
//                    if (entry->info->deserialize(r, value))
//                    {
//                        req.setProperty(id, entry, value);
//                        ROS_INFO_STREAM("Sync plugin: setProperty " << id);
//                    } else
//                        error += "For entity '" + id + "': deserialization of property '" + prop_name +"' failed.\n";
//                }

//                r.endArray();
//            }
        }

        r.endArray();
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

void serialize(const geo::Pose3D& pose, ed::io::Writer& w)
{
    w.writeValue("x", pose.t.x);
    w.writeValue("y", pose.t.y);
    w.writeValue("z", pose.t.z);

    geo::Quaternion q = pose.getQuaternion();
    w.writeValue("qx", q.x);
    w.writeValue("qy", q.y);
    w.writeValue("qz", q.z);
    w.writeValue("qw", q.w);
}

// ----------------------------------------------------------------------------------------------------

bool deserialize(ed::io::Reader& r, geo::Pose3D& pose)
{
    // Read position
    if (r.readValue("x", pose.t.x))
    {
        r.readValue("y", pose.t.y);
        r.readValue("z", pose.t.z);
    }
    else if (r.readGroup("pos") || r.readGroup("t"))
    {
        r.readValue("x", pose.t.x);
        r.readValue("y", pose.t.y);
        r.readValue("z", pose.t.z);
        r.endGroup();
    }
    else
        return false;

    // Read orientation
    geo::Quaternion q;
    if (r.readValue("qx", q.x))
    {
        r.readValue("qy", q.y);
        r.readValue("qz", q.z);
        r.readValue("qw", q.w);

        pose.R.setRotation(q);
    }
    else if (r.readGroup("rot") || r.readGroup("R"))
    {
        r.readValue("xx", pose.R.xx);
        r.readValue("xy", pose.R.xy);
        r.readValue("xz", pose.R.xz);
        r.readValue("yx", pose.R.yx);
        r.readValue("yy", pose.R.yy);
        r.readValue("yz", pose.R.yz);
        r.readValue("zx", pose.R.zx);
        r.readValue("zy", pose.R.zy);
        r.readValue("zz", pose.R.zz);
        r.endGroup();
    }
    else
        return false;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool deserialize(tue::config::Reader& r, const std::string& group, geo::Pose3D& pose)
{
    if (!r.readGroup(group))
        return false;

    pose.t = geo::Vec3(0, 0, 0);
    r.value("x", pose.t.x, tue::config::OPTIONAL);
    r.value("y", pose.t.y, tue::config::OPTIONAL);
    r.value("z", pose.t.z, tue::config::OPTIONAL);

    double roll = 0, pitch = 0, yaw = 0;
    r.value("X", roll,  tue::config::OPTIONAL);
    r.value("Y", pitch, tue::config::OPTIONAL);
    r.value("Z", yaw,   tue::config::OPTIONAL);
    r.value("roll",  roll,  tue::config::OPTIONAL);
    r.value("pitch", pitch, tue::config::OPTIONAL);
    r.value("yaw",   yaw,   tue::config::OPTIONAL);

    // Set rotation
    pose.R.setRPY(roll, pitch, yaw);

    geo::Quaternion q;
    if (r.value("qx", q.x, tue::config::OPTIONAL)
            && r.value("qy", q.y, tue::config::OPTIONAL)
            && r.value("qz", q.z, tue::config::OPTIONAL)
            && r.value("qw", q.w, tue::config::OPTIONAL))
    {
        pose.R.setRotation(q);
    }

    r.endGroup();
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool deserialize(tue::config::Reader& r, const std::string& group, geo::Vec3& p)
{
    if (!r.readGroup(group))
        return false;

    r.value("x", p.x);
    r.value("y", p.y);
    r.value("z", p.z);

    r.endGroup();
    return true;
}

// ----------------------------------------------------------------------------------------------------

void serialize(const ConvexHull& ch, ed::io::Writer& w)
{
    w.writeArray("points");
    for (std::vector<geo::Vec2f>::const_iterator it = ch.points.begin(); it != ch.points.end(); ++it)
    {
        w.addArrayItem();
        w.writeValue("x", it->x); w.writeValue("y", it->y);
        w.endArrayItem();
    }
    w.endArray();
    w.writeValue("z_min", ch.z_min);
    w.writeValue("z_max", ch.z_max);
}

// ----------------------------------------------------------------------------------------------------

bool deserialize(ed::io::Reader& r, ConvexHull& ch)
{
    if (r.readArray("points"))
    {
        while(r.nextArrayItem())
        {
            geo::Vec2f p;
            r.readValue("x", p.x);
            r.readValue("y", p.y);
            ch.points.push_back(p);
        }
        r.endArray();
    }

    r.readValue("z_min", ch.z_min);
    r.readValue("z_max", ch.z_max);

    convex_hull::calculateEdgesAndNormals(ch);
    convex_hull::calculateArea(ch);

    return true;
}

// ----------------------------------------------------------------------------------------------------

void serialize(const geo::Shape& s, ed::io::Writer& w)
{
    w.writeArray("vertices");
    const std::vector<geo::Vector3>& vertices = s.getMesh().getPoints();
    for(std::vector<geo::Vector3>::const_iterator it = vertices.begin(); it != vertices.end(); ++it)
    {
        w.addArrayItem();
        w.writeValue("x", it->x); w.writeValue("y", it->y); w.writeValue("z", it->z);
        w.endArrayItem();
    }
    w.endArray();

    w.writeArray("triangles");
    const std::vector<geo::TriangleI>& triangles = s.getMesh().getTriangleIs();
    for(unsigned int i = 0; i < triangles.size(); ++i)
    {
        w.addArrayItem();
        w.writeValue("i1", triangles[i].i1_);
        w.writeValue("i2", triangles[i].i2_);
        w.writeValue("i3", triangles[i].i3_);
        w.endArrayItem();

    }
    w.endArray();
}

// ----------------------------------------------------------------------------------------------------

bool deserialize(ed::io::Reader& r, geo::Shape& s)
{
    geo::Mesh mesh;

    // Vertices
    if (r.readArray("vertices"))
    {
        while(r.nextArrayItem())
        {
            geo::Vector3 p;
            r.readValue("x", p.x);
            r.readValue("y", p.y);
            r.readValue("z", p.z);
            mesh.addPoint(p);
        }

        r.endArray();
    }
    else
        return false;

    // Triangles
    if (r.readArray("triangles"))
    {
        while(r.nextArrayItem())
        {
            int i1, i2, i3;
            r.readValue("i1", i1);
            r.readValue("i2", i2);
            r.readValue("i3", i3);
            mesh.addTriangle(i1, i2, i3);
        }

        r.endArray();
    }
    else
        return false;

    s.setMesh(mesh);
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool deserialize(tue::config::Reader& r_orig, const std::string& group, geo::Shape& s)
{
    tue::config::Reader r = r_orig;

    if (!r.readArray(group))
        return false;

    geo::Mesh mesh;

    while(r.nextArrayItem())
    {
        if (r.readGroup("box"))
        {
            geo::Vec3 min, max, size;
            if (deserialize(r, "min", min))
            {
                if (!deserialize(r, "max", max))
                    return false;
            }
            else if (deserialize(r, "size", size))
            {
                min = -0.5 * size;
                max =  0.5 * size;
            }
            else
            {
                return false;
            }

            geo::Pose3D pose;
            if (!deserialize(r, "pose", pose))
                pose = geo::Pose3D::identity();

            mesh.add(geo::Box(min, max).getMesh().getTransformed(pose));
        }
    }

    if (mesh.getTriangleIs().empty())
        return false;

    s.setMesh(mesh);

    return true;
}

// ----------------------------------------------------------------------------------------------------

void serializeTimestamp(double time, ed::io::Writer& w)
{
    w.writeValue("sec", (int)time);
    w.writeValue("nsec", (int)((time - (int)time) * 1e9));
}

// ----------------------------------------------------------------------------------------------------

bool deserializeTimestamp(ed::io::Reader& r, double& time)
{
    int sec, nsec;
    r.readValue("sec", sec);
    r.readValue("nsec", nsec);
    time = sec + (double)nsec / 1e9;
    return true;
}

// ----------------------------------------------------------------------------------------------------

void serialize(const ImageMask& mask, tue::serialization::OutputArchive& m)
{
    const static int MASK_SERIALIZATION_VERSION = 0;

    m << MASK_SERIALIZATION_VERSION;

    m << mask.width();
    m << mask.height();

    // Determine size
    int size = 0;
    for(ImageMask::const_iterator it = mask.begin(); it != mask.end(); ++it)
        ++size;

    m << size;

    for(ImageMask::const_iterator it = mask.begin(); it != mask.end(); ++it)
        m << it->y * mask.width() + it->x;
}

// ----------------------------------------------------------------------------------------------------

bool deserialize(tue::serialization::InputArchive& m, ImageMask& mask)
{
    int version;
    m >> version;

    int width, height;
    m >> width;
    m >> height;
    mask = ImageMask(width, height);

    int size;
    m >> size;

    for(int i = 0; i < size; ++i)
    {
        int idx;
        m >> idx;

        mask.addPoint(idx % width, idx / width);
    }

    return true;
}



// ----------------------------------------------------------------------------------------------------
//
//                                          SERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------------

//void serialize(const WorldModel& wm, tue::config::Writer& w)
//{
//    w.writeArray("entities");

//    for(WorldModel::const_iterator it = wm.begin(); it != wm.end(); ++it)
//    {
//        const EntityConstPtr& e = *it;

//        std::cout << e->id() << std::endl;

//        w.addArrayItem();

//        w.setValue("id", e->id().str());
//        w.setValue("type", e->type());

//        if (e->has_pose())
//        {
//            w.writeGroup("pose");
//            w.setValue("x", e->pose().t.x);
//            w.setValue("y", e->pose().t.y);
//            w.setValue("z", e->pose().t.z);

//            double X, Y, Z;
//            getEulerYPR(e->pose().R, Z, Y, X);
//            w.setValue("X", X);
//            w.setValue("Y", Y);
//            w.setValue("Z", Z);
//            w.endGroup();
//        }

//        w.endArrayItem();
//    }

//    w.endArray();
//}

// ----------------------------------------------------------------------------------------------------
//
//                                         DESERIALIZATION
//
// ----------------------------------------------------------------------------------------------------



// ----------------------------------------------------------------------------------------------------

//void deserialize(tue::config::Reader& r, UpdateRequest& req)
//{
//    if (r.readArray("entities"))
//    {
//        while(r.nextArrayItem())
//        {
//            std::string id;
//            if (!r.value("id", id))
//                continue;

//            if (r.readGroup("pose"))
//            {
//                geo::Pose3D pose;

//                if (!r.value("x", pose.t.x) || !r.value("y", pose.t.y) || !r.value("z", pose.t.z))
//                    continue;

//                double rx = 0, ry = 0, rz = 0;
//                r.value("rx", rx, tue::config::OPTIONAL);
//                r.value("ry", ry, tue::config::OPTIONAL);
//                r.value("rz", rz, tue::config::OPTIONAL);

//                pose.R.setRPY(rx, ry, rz);

//                req.setPose(id, pose);

//                r.endGroup();
//            }
//        }

//        r.endArray();
//    }
//}

}
