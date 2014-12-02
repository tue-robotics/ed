#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/relations/transform_cache.h>

#include <ros/time.h>    // Why do we need this?

// Profiling
#include <tue/profiling/timer.h>

// ----------------------------------------------------------------------------------------------------

void buildWorldModel(ed::WorldModel& wm)
{
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

    std::string parent_id = "bottle";
    for(unsigned int i = 0; i < 100000; ++i)
    {
        std::stringstream id;
        id << "e" << i;
        req.setType(id.str(), "object");

        boost::shared_ptr<ed::TransformCache> t1(new ed::TransformCache());
        t1->insert(0, geo::Pose3D(1, 2, 3, 0, 0, M_PI / 2));
        req.setRelation(parent_id, id.str(), t1);

        parent_id = id.str();
    }

    wm.update(req);
}

// ----------------------------------------------------------------------------------------------------

void profile(const ed::WorldModel& wm)
{
    unsigned int N = 1000;

    ed::UUID id1 = "map";
    ed::UUID id2 = "e100";

    tue::Timer timer;
    timer.start();

    geo::Pose3D tr;
    for(unsigned int i = 0; i < N; ++i)
    {
        wm.calculateTransform(id1, id2, 0, tr);
    }

    std::cout << timer.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
}

// ----------------------------------------------------------------------------------------------------

void testCorrectness(const ed::WorldModel& wm)
{
    ed::UUID id1 = "map";
    ed::UUID id2 = "bottle";

    geo::Pose3D tr;
    if (wm.calculateTransform(id1, id2, 0, tr))
    {
        std::cout << tr << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::Time::init();                      // Why do we need this?

    ed::WorldModel wm;
    buildWorldModel(wm);

    std::string cmd;
    if (argc > 1)
        cmd = argv[1];

    if (cmd == "profile")
        profile(wm);
    else
        testCorrectness(wm);

    return 0;
}
