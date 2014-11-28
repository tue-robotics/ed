#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/relations/transform_cache.h>

#include <ros/time.h>    // Why do we need this?

int main(int argc, char **argv)
{
    ros::Time::init();                      // Why do we need this?

    ed::WorldModel wm;

    ed::UpdateRequest req;

    req.setType("map", "waypoint");
    req.setType("table", "object");
    req.setType("bottle", "object");

    boost::shared_ptr<ed::TransformCache> t1(new ed::TransformCache());
    t1->insert(0, geo::Pose3D(1, 2, 3, 0, 0, M_PI / 2));
    req.setRelation("map", "table", t1);

    boost::shared_ptr<ed::TransformCache> t2(new ed::TransformCache());
    t2->insert(0, geo::Pose3D(1, 0, 0.75).inverse());
    req.setRelation("bottle", "table", t2);

    wm.update(req);

    ed::UUID id1 = "map";
    ed::UUID id2 = "bottle";

    geo::Pose3D tr;
    if (wm.calculateTransform(id1, id2, 0, tr))
    {
        std::cout << tr << std::endl;
    }

    return 0;
}
