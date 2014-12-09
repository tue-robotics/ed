#include <ed/world_model.h>
#include <ed/update_request.h>
#include <ed/entity.h>
#include <ed/relations/transform_cache.h>
#include <ed/world_model/transform_crawler.h>

#include <ros/time.h>    // Why do we need this?

// Profiling
#include <tue/profiling/timer.h>

// ----------------------------------------------------------------------------------------------------

void setTransform(const ed::UUID& id1, const ed::UUID& id2, const geo::Pose3D& transform, ed::UpdateRequest& req)
{
    boost::shared_ptr<ed::TransformCache> t1(new ed::TransformCache());
    t1->insert(0, transform);
    req.setRelation(id1, id2, t1);
}

// ----------------------------------------------------------------------------------------------------

void buildWorldModel(ed::WorldModel& wm, unsigned int num_extra_entities = 0)
{
    ed::UpdateRequest req;

    req.setType("map", "waypoint");
    req.setType("table", "object");
    req.setType("bottle", "object");
    req.setType("cup", "object");

    setTransform("map", "table",  geo::Pose3D(1, 2, 3, 0, 0, M_PI / 2), req);
    setTransform("bottle", "table", geo::Pose3D(1, 0, 0.75).inverse(), req);
    setTransform("table", "cup", geo::Pose3D(0.5, 0.2, 0.75), req);

    std::string parent_id = "bottle";
    for(unsigned int i = 0; i < num_extra_entities; ++i)
    {
        std::stringstream id;
        id << "e" << i;
        req.setType(id.str(), "object");

        setTransform(parent_id, id.str(), geo::Pose3D(1, 2, 3, 0, 0, M_PI / 2), req);

        parent_id = id.str();
    }

    wm.update(req);
}

// ----------------------------------------------------------------------------------------------------

void profile(const ed::WorldModel& wm)
{
    unsigned int N = 1000;

    {
        ed::UUID id1 = "map";
        ed::UUID id2 = "e100";

        tue::Timer timer;
        timer.start();

        geo::Pose3D tr;
        for(unsigned int i = 0; i < N; ++i)
        {
            wm.calculateTransform(id1, id2, 0, tr);
        }

        std::cout << "Single transform: " << timer.getElapsedTimeInMilliSec() / N << " ms" << std::endl;
    }

    {
        tue::Timer timer;
        timer.start();

        for(unsigned int i = 0; i < N; ++i)
        {
            for(ed::world_model::TransformCrawler tc(wm, "bottle", 0); tc.hasNext(); tc.next())
            {
            }
        }

        std::cout << "Transform crawling (per entity): " << timer.getElapsedTimeInMilliSec() / N / wm.numEntities() << " ms" << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

void testCorrectness(const ed::WorldModel& wm)
{
    ed::UUID id1 = "map";
    ed::UUID id2 = "bottle";

    geo::Pose3D tr;
    if (wm.calculateTransform(id1, id2, 0, tr))
    {
        std::cout << std::fixed << std::setprecision(3) << tr << std::endl;
    }

    for(ed::world_model::TransformCrawler tc(wm, "map", 0); tc.hasNext(); tc.next())
    {
        std::cout << tc.entity()->id() << ": " << tc.transform() << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::Time::init();                      // Why do we need this?

    ed::WorldModel wm;

    std::string cmd;
    if (argc > 1)
        cmd = argv[1];

    if (cmd == "profile")
    {
        buildWorldModel(wm, 1000);
        profile(wm);
    }
    else
    {
        buildWorldModel(wm);
        testCorrectness(wm);
    }

    return 0;
}
