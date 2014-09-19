#include "occupancy_grid_publisher_plugin.h"

//#include <ros/node_handle.h>
#include <nav_msgs/OccupancyGrid.h>

#include <geolib/ros/msg_conversions.h>
#include <geolib/Shape.h>

#include <ed/world_model.h>
#include <ed/entity.h>

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisherPlugin::configure(tue::Configuration config)
{
    config.value("frequency", frequency_);

    config.value("resolution", res_);

    std::string old_topic = topic_;
    config.value("topic", topic_);
    config.value("frame_id", frame_id_);

    //! Re-initialize if topic has changed
    if (old_topic != "" && topic_ != old_topic)
    {
        initialize();
    }
}

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisherPlugin::initialize()
{
    //! Initialize occupancy map publisher
    ros::NodeHandle nh;
    map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>(topic_, 0, false);

//    std::cout << "Map publisher: \n" <<
//                 "- frequency: " << frequency_ << "\n" <<
//                 "- size_x: " << size_x_ << "\n" <<
//                 "- size_y: " << size_y_ << "\n" <<
//                 "- resolution: " << res_ << "\n" <<
//                 "- topic: " << topic_ << "\n" <<
//                 "- frame_id: " << frame_id_ << std::endl;
}

bool getOriginWidthAndHeight(const ed::WorldModel& world, const double& res, geo::Vector3& origin, int& width, int& height)
{
    geo::Vector3 min(1e6,1e6,0);
    geo::Vector3 max(-1e6,-1e6,0);
    for(std::map<ed::UUID, ed::EntityConstPtr>::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        ed::EntityConstPtr e = it->second;
        geo::ShapeConstPtr shape = e->shape();
        if (shape)  // Do shape
        {
            const std::vector<geo::Triangle>& triangles = shape->getMesh().getTriangles();

            for(std::vector<geo::Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {

                geo::Vector3 p1w = e->pose() * it->p1_;
                geo::Vector3 p2w = e->pose() * it->p2_;
                geo::Vector3 p3w = e->pose() * it->p3_;

                // Filter the ground
                if (p1w.getZ() > 0.05001 && p2w.getZ() > 0.050001 && p3w.getZ() > 0.05001)
                {
                    // Update min max
                    min.x = std::min(p1w.x, std::min(p2w.x, std::min(p3w.x, min.x)));
                    min.y = std::min(p1w.y, std::min(p2w.y, std::min(p3w.y, min.y)));

                    max.x = std::max(p1w.x, std::max(p2w.x, std::max(p3w.x, max.x)));
                    max.y = std::max(p1w.y, std::max(p2w.y, std::max(p3w.y, max.y)));
                }
            }
        }
        else // Do convex hull
        {
            const pcl::PointCloud<pcl::PointXYZ>& chull_points = e->convexHull().chull;

            for (unsigned int i = 0; i < chull_points.size(); ++i)
            {

                geo::Vector3 p1w(chull_points.points[i].x, chull_points.points[i].y, 0);
                geo::Vector3 p2w;
                if (i == chull_points.size() - 1)
                    p2w = geo::Vector3(chull_points.points[0].x, chull_points.points[0].y, 0);
                else
                    p2w = geo::Vector3(chull_points.points[i+1].x, chull_points.points[i+1].y, 0);

                // Update min max
                min.x = std::min(p1w.x, std::min(p2w.x, min.x));
                min.y = std::min(p1w.y, std::min(p2w.y, min.y));

                max.x = std::max(p1w.x, std::max(p2w.x, max.x));
                max.y = std::max(p1w.y, std::max(p2w.y, max.y));
            }
        }
    }

    // Bounds fix
    min.x-=1.0;
    min.y-=1.0;

    max.x+=1.0;
    max.y+=1.0;

    // Set the origin
    origin = min;

    double size_x = max.x - min.x;
    double size_y = max.y - min.y;

    if (size_x > 0 && size_y > 0)
    {
        width = size_x / res;
        height = size_y / res;
    }
    else
    {
        width = 0;
        height = 0;
        return false;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisherPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{    
    if (getOriginWidthAndHeight(world, res_, origin_, width_, height_))
    {
        cv::Mat map = cv::Mat::zeros(height_, width_, CV_8U);

        for(std::map<ed::UUID, ed::EntityConstPtr>::const_iterator it = world.begin(); it != world.end(); ++it)
            updateMap(it->second, map);

        publishMapMsg(map);
    }
}

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisherPlugin::publishMapMsg(const cv::Mat& map)
{
    nav_msgs::OccupancyGrid map_msg;
    geo::convert(origin_, map_msg.info.origin.position);

    map_msg.info.resolution = res_;
    map_msg.info.width = width_;
    map_msg.info.height = height_;

    map_msg.data.resize(width_ * height_);
    unsigned int i = 0;
    for(int my = 0; my < height_; ++my)
    {
        for(int mx = 0; mx < width_; ++mx)
        {
            unsigned char c = map.at<unsigned char>(my, mx);
            if (c > 0)
                map_msg.data[i] = c;
            else
                map_msg.data[i] = -1;
            ++i;
        }
    }

    map_msg.header.stamp = ros::Time::now();
    map_msg.header.frame_id = frame_id_;
    map_pub_.publish(map_msg);
}

// ----------------------------------------------------------------------------------------------------

bool OccupancyGridPublisherPlugin::worldToMap(double wx, double wy, int& mx, int& my) const
{
    if (wx < origin_.x || wy < origin_.y)
        return false;

    mx = (wx - origin_.x) / res_ ;
    my = (wy - origin_.y) / res_ ;

    if (mx < width_ && my < height_)
        return true;

    return false;
}

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisherPlugin::updateMap(const ed::EntityConstPtr& e, cv::Mat& map)
{
    geo::ShapeConstPtr shape = e->shape();
    if (shape)  // Do shape
    {
        const std::vector<geo::Triangle>& triangles = shape->getMesh().getTriangles();

        for(std::vector<geo::Triangle>::const_iterator it = triangles.begin(); it != triangles.end(); ++it) {

            geo::Vector3 p1w = e->pose() * it->p1_;
            geo::Vector3 p2w = e->pose() * it->p2_;
            geo::Vector3 p3w = e->pose() * it->p3_;

            // Filter the ground
            if (p1w.getZ() > 0.05001 && p2w.getZ() > 0.050001 && p3w.getZ() > 0.05001) {

                cv::Point2i p1, p2, p3;

                // Check if all points are on the map
                if ( worldToMap(p1w.x, p1w.y, p1.x, p1.y) && worldToMap(p2w.x, p2w.y, p2.x, p2.y) && worldToMap(p3w.x, p3w.y, p3.x, p3.y) ) {
                    cv::line(map, p1, p2, 100);
                    cv::line(map, p1, p3, 100);
                    cv::line(map, p2, p3, 100);
                }
            }
        }
    }
    else // Do convex hull
    {
        const pcl::PointCloud<pcl::PointXYZ>& chull_points = e->convexHull().chull;

        for (unsigned int i = 0; i < chull_points.size(); ++i)
        {

            geo::Vector3 p1w(chull_points.points[i].x, chull_points.points[i].y, 0);
            geo::Vector3 p2w;
            if (i == chull_points.size() - 1)
                p2w = geo::Vector3(chull_points.points[0].x, chull_points.points[0].y, 0);
            else
                p2w = geo::Vector3(chull_points.points[i+1].x, chull_points.points[i+1].y, 0);

            // Check if all points are on the map
            cv::Point2i p1, p2;
            if ( worldToMap(p1w.x, p1w.y, p1.x, p1.y) && worldToMap(p2w.x, p2w.y, p2.x, p2.y) )
                cv::line(map, p1, p2, 100);

        }
    }
}

// ----------------------------------------------------------------------------------------------------

void OccupancyGridPublisherPlugin::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
    wx = origin_.x + (mx + 0.5) * res_;
    wy = origin_.y + (my + 0.5) * res_;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(OccupancyGridPublisherPlugin)



