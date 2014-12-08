#include "ed/serialization/serialization.h"
#include "ed/mask.h"

#include <tue/config/reader.h>
#include <tue/config/writer.h>

#include "ed/world_model.h"
#include "ed/update_request.h"
#include "ed/entity.h"

namespace ed
{

const static int MASK_SERIALIZATION_VERSION = 0;

// ----------------------------------------------------------------------------------------------------
//
//                                          HELPER METHODS
//
// ----------------------------------------------------------------------------------------------------

// Taken directly from the TF ROS package (https://github.com/ros/geometry.git)
void getEulerYPR(const geo::Matrix3& m, double& yaw, double& pitch, double& roll)
{
    // Check that pitch is not at a singularity
    if (fabs(m.zx >= 1))
    {
        yaw = 0;

        // From difference of angles formula
        if (m.zx < 0)  //gimbal locked down
        {
            pitch = M_PI / 2.0;
            roll = atan2(m.xy, m.xz);
        }
        else // gimbal locked up
        {
            pitch = -M_PI / 2.0;
            roll = atan2(-m.yx, -m.xz);
        }
    }
    else
    {
        pitch = -asin(m.zx);
        roll = atan2(m.zy / cos(pitch), m.zz / cos(pitch));
        yaw  = atan2(m.yx / cos(pitch), m.xx / cos(pitch));
    }
}

// ----------------------------------------------------------------------------------------------------
//
//                                          SERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

void serialize(const ImageMask& mask, tue::serialization::OutputArchive& m)
{
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

void serialize(const WorldModel& wm, tue::config::Writer& w)
{
    w.writeArray("entities");

    for(WorldModel::const_iterator it = wm.begin(); it != wm.end(); ++it)
    {
        const EntityConstPtr& e = *it;

        std::cout << e->id() << std::endl;

        w.addArrayItem();

        w.setValue("id", e->id().str());
        w.setValue("type", e->type());

        w.writeGroup("pose");
        w.setValue("x", e->pose().t.x);
        w.setValue("y", e->pose().t.y);
        w.setValue("z", e->pose().t.z);

        double X, Y, Z;
        getEulerYPR(e->pose().R, Z, Y, X);
        w.setValue("X", X);
        w.setValue("Y", Y);
        w.setValue("Z", Z);
        w.endGroup();

        w.endArrayItem();
    }

    w.endArray();
}

// ----------------------------------------------------------------------------------------------------
//
//                                         DESERIALIZATION
//
// ----------------------------------------------------------------------------------------------------

void deserialize(tue::serialization::InputArchive& m, ImageMask& mask)
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
}

// ----------------------------------------------------------------------------------------------------

void deserialize(tue::config::Reader& r, UpdateRequest& req)
{
    if (r.readArray("entities"))
    {
        while(r.nextArrayItem())
        {
            std::string id;
            if (!r.value("id", id))
                continue;

            if (r.readGroup("pose"))
            {
                geo::Pose3D pose;

                if (!r.value("x", pose.t.x) || !r.value("y", pose.t.y) || !r.value("z", pose.t.z))
                    continue;

                double rx = 0, ry = 0, rz = 0;
                r.value("rx", rx, tue::config::OPTIONAL);
                r.value("ry", ry, tue::config::OPTIONAL);
                r.value("rz", rz, tue::config::OPTIONAL);

                pose.R.setRPY(rx, ry, rz);

                req.setPose(id, pose);

                r.endGroup();
            }
        }

        r.endArray();
    }
}

}
