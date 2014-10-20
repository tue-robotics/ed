#include "ed/server.h"

#include <ros/package.h>

// Query
#include <ed/entity.h>
#include <ed/SimpleQuery.h>
#include <geolib/ros/msg_conversions.h>

// Publish TF
#include <tf/transform_broadcaster.h>
#include <geolib/ros/tf_conversions.h>

// Visualization
#include <ed/measurement.h>
#include <rgbd/Image.h>
#include <opencv2/highgui/highgui.hpp>

#include <ed/event_clock.h>
#include <std_srvs/Empty.h>

// Plugin loading
#include <ed/plugin.h>
#include <ed/plugin_container.h>
#include <ed/LoadPlugin.h>
#include <tue/config/loaders/yaml.h>

ed::Server* ed_wm;

// ----------------------------------------------------------------------------------------------------

void entityToMsg(const ed::Entity& e, ed::EntityInfo& msg)
{
    msg.id = e.id();
    msg.type = e.type();
    msg.creation_time = ros::Time(e.creationTime());

    // Convex hull
    const ed::ConvexHull2D& convex_hull = e.convexHull();
    if (!convex_hull.chull.empty())
    {
        geo::convert(convex_hull.center_point, msg.center_point);
        msg.z_min = convex_hull.min_z;
        msg.z_max = convex_hull.max_z;

        msg.convex_hull.resize(convex_hull.chull.size());
        for(unsigned int i = 0; i < msg.convex_hull.size(); ++i)
        {
            msg.convex_hull[i].x = convex_hull.chull[i].x;
            msg.convex_hull[i].y = convex_hull.chull[i].y;
            msg.convex_hull[i].z = convex_hull.chull[i].z;
        }
    }

    msg.has_shape = e.shape() ? true : false;
    if (msg.has_shape)
    {
        // If the entity has a shape, use the pose of this shape
        geo::convert(e.pose(), msg.pose);
    }
    else
    {
        // If the entity has no shape, use the center point of the convex hull as point
        geo::convert(geo::Pose3D(geo::Matrix3::identity(), e.convexHull().center_point), msg.pose);
    }

    ed::MeasurementConstPtr m = e.lastMeasurement();
    if (m)
        msg.last_update_time =  ros::Time(m->timestamp());
}

// ----------------------------------------------------------------------------------------------------

bool srvReset(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ed_wm->reset();
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvSimpleQuery(ed::SimpleQuery::Request& req, ed::SimpleQuery::Response& res)
{
    double radius = req.radius;
    geo::Vector3 center_point;
    geo::convert(req.center_point, center_point);

    const std::map<ed::UUID, ed::EntityConstPtr>& entities = ed_wm->entities();

    for(std::map<ed::UUID, ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
//        std::cout << it->first << std::endl;

        const ed::EntityConstPtr& e = it->second;
        if (!req.id.empty() && e->id() != req.id)
            continue;

        if (!req.type.empty())
        {
            if (req.type == "unknown")
            {
                if (e->type() != "")
                    continue;
            }
            else
            {
                if (e->type() != req.type)
                    continue;
            }
        }

        bool geom_ok = true;
        if (radius > 0)
        {
            geo::Vector3 p_entity;
            if (e->shape())
                p_entity = e->pose().getOrigin();
            else
            {
                p_entity.x = e->convexHull().center_point.x;
                p_entity.y = e->convexHull().center_point.y;
                p_entity.z = e->convexHull().center_point.z;
            }

            geom_ok = (radius * radius > (p_entity - center_point).length2());
        }

        if (geom_ok)
        {
            res.entities.push_back(ed::EntityInfo());
            entityToMsg(*e, res.entities.back());
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool srvLoadPlugin(const ed::LoadPlugin::Request& req, ed::LoadPlugin::Response& res)
{
    std::string error;
    ed::PluginContainerPtr container = ed_wm->loadPlugin(req.plugin_name, req.library_path, error);
    if (!container)
    {
        res.error_msg = error;
    }
    else
    {
        if (!req.configuration.empty())
        {
            tue::Configuration cfg;
            if (tue::config::loadFromYAMLString(req.configuration, cfg))
            {
                container->plugin()->configure(cfg);

                double loop_frequency;
                if (cfg.value("loop_frequency", loop_frequency, tue::OPTIONAL))
                {
                    container->setLoopFrequency(loop_frequency);
                }
            }

            if (cfg.hasError())
                res.error_msg = cfg.error();
        }

        container->plugin()->initialize();
        container->runThreaded();
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

void publishTFs(tf::TransformBroadcaster& tf_broadcaster)
{
    const std::map<ed::UUID, ed::EntityConstPtr>& entities = ed_wm->entities();

    for(std::map<ed::UUID, ed::EntityConstPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
    {
        const ed::EntityConstPtr& e = it->second;

        geo::Pose3D pose_MAP;

        if (!e->shape())
        {
            // determine pose based on convex hull
            pose_MAP.t = e->convexHull().center_point;
            pose_MAP.R = geo::Matrix3::identity();
        }
        else
        {
            pose_MAP = e->pose();
        }

        tf::StampedTransform t;
        geo::convert(pose_MAP, t);
        t.frame_id_ = "/map";
        t.child_frame_id_ = e->id();
        t.stamp_ = ros::Time::now();

        tf_broadcaster.sendTransform(t);
    }
}

// ----------------------------------------------------------------------------------------------------

cv::Mat visualize()
{
    cv::Mat rgb_image;

//    double t_latest_image = 0;
//    rgbd::ImageConstPtr latest_image;

//    // determine latest image
//    const std::map<ed::UUID, ed::EntityPtr>& entities = ed_wm->entities();
//    for(std::map<ed::UUID, ed::EntityPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
//    {
//        const ed::EntityPtr& e = it->second;
//        ed::MeasurementConstPtr msr = e->getLastMeasurement();

//        if (msr && msr->image()->getTimestamp() > t_latest_image)
//        {
//            t_latest_image = msr->image()->getTimestamp();
//            latest_image = msr->image();
//        }
//    }

//    if (!latest_image)
//        return rgb_image;

//    // Get RGB image
//    rgb_image = latest_image->getRGBImage().clone();

//    // Draw masks
//    for(std::map<ed::UUID, ed::EntityPtr>::const_iterator it = entities.begin(); it != entities.end(); ++it)
//    {
//        const ed::EntityPtr& e = it->second;
//        ed::MeasurementConstPtr msr = e->getLastMeasurement();
//        if (msr)
//        {
//            cv::Mat contour_img = cv::Mat::zeros(rgb_image.rows, rgb_image.cols, CV_8UC1);
//            for(ed::Mask::const_iterator it = msr->mask().begin(rgb_image.rows); it != msr->mask().end(); ++it)
//            {
////                std::cout << *it << std::endl;
//                contour_img.at<unsigned char>(*it) = 255;
//            }

//            std::vector<std::vector<cv::Point> > contours;
//            cv::findContours(contour_img, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

//            drawContours(rgb_image, contours, -1, cv::Scalar(255, 255, 255), 5);
//        }
//    }



    return rgb_image;
}

// ----------------------------------------------------------------------------------------------------

bool getEnvironmentVariable(const std::string& var, std::string& value)
{
     const char * val = ::getenv(var.c_str());
     if ( val == 0 )
         return false;

     value = val;
     return true;
}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char** argv)
{

    ros::init(argc, argv, "ed");

    ed_wm = new ed::Server();

    // - - - - - - - - - - - - - - - configure - - - - - - - - - - - - - - -

    // Get plugin paths
    std::string ed_plugin_path;
    if (getEnvironmentVariable("ED_PLUGIN_PATH", ed_plugin_path))
    {
        std::stringstream ss(ed_plugin_path);
        std::string item;
        while (std::getline(ss, item, ':'))
            ed_wm->addPluginPath(item);
    }
    else
    {
        std::cout << "Error: Environment variable ED_PLUGIN_PATH not set." << std::endl;
        return 1;
    }

    tue::Configuration config;

    // Check if a config file was provided. If so, load it. If not, load a default config.
    if (argc >= 2)
    {
        std::string yaml_filename = argv[1];
        config.loadFromYAMLFile(yaml_filename);
    }
    else
    {
        // Get the ED directory
        std::string ed_dir = ros::package::getPath("ed");

        // Load the YAML config file
        config.loadFromYAMLFile(ed_dir + "/config/config.yml");
    }

    // Configure ED
    ed_wm->configure(config);

    if (config.hasError())
    {
        std::cout << std::endl << "Error during configuration:" << std::endl << std::endl << config.error() << std::endl;
        return 1;
    }

    // - - - - - - - - - - - - service initialization - - - - - - - - - - - -

    ros::NodeHandle nh;

    ros::CallbackQueue cb_queue;

    ros::AdvertiseServiceOptions opt_simple_query =
            ros::AdvertiseServiceOptions::create<ed::SimpleQuery>(
                "/ed/simple_query", srvSimpleQuery, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_simple_query = nh.advertiseService(opt_simple_query);

    ros::AdvertiseServiceOptions opt_reset =
            ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
                "/ed/reset", srvReset, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_reset = nh.advertiseService(opt_reset);

    ros::AdvertiseServiceOptions opt_load_plugin =
            ros::AdvertiseServiceOptions::create<ed::LoadPlugin>(
                "/ed/load_plugin", srvLoadPlugin, ros::VoidPtr(), &cb_queue);
    ros::ServiceServer srv_load_plugin = nh.advertiseService(opt_load_plugin);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Init tf broadcaster
    tf::TransformBroadcaster tf_broadcaster;

    // Init ED
    ed_wm->initialize();

    ed::EventClock trigger_config(10);
    ed::EventClock trigger_ed(10);
    ed::EventClock trigger_gui(100);
    ed::EventClock trigger_tf(10);
    ed::EventClock trigger_plugins(1000);

    ros::Rate r(1000);
    while(ros::ok()) {
        cb_queue.callAvailable();

        // Check if configuration has changed. If so, call reconfigure
        if (trigger_config.triggers() && config.sync())
            ed_wm->configure(config, true);

////        std::cout << "CycleTime: " << r.cycleTime() << " seconds. - " << ed_wm->size() << " entities in world model." << std::endl;

        if (trigger_ed.triggers())
            ed_wm->update();
//        ed.storeEntityMeasurements("/tmp/ed_measurements");

//        if (trigger_tf.triggers())
//            publishTFs(tf_broadcaster);

//        if (trigger_gui.triggers())
//            ed_wm->updateGUI();

        if (trigger_plugins.triggers())
            ed_wm->stepPlugins();

//        cv::Mat visualization = visualize();
//        if (visualization.data)
//        {
//            cv::imshow("visualization", visualization);
//            cv::waitKey(3);
//        }

        r.sleep();

    }

    delete ed_wm;

    return 0;
}
