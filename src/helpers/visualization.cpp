#include "ed/helpers/visualization.h"
#include "ed/mask.h"
#include "ed/entity.h"
#include "ed/measurement.h"
#include "ed/world_model.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>
#include <opencv/highgui.h>

#include <geolib/Shape.h>
#include <geolib/ros/msg_conversions.h>

#include <stdlib.h>

#include <tue/config/reader.h>

namespace ed
{

namespace helpers
{

namespace visualization
{

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

int getHash(const std::string& id)
{
    const char* str = id.c_str();

//    unsigned long hash = 5381;
    int hash = 5381;
    int c;

    while ((c = *str++))
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

    return hash;// % max_val;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

std_msgs::ColorRGBA getColor(unsigned int id)
{
    float colors[8][3] = { {1,0,0} , {0,1,0} , {0,0,1} , {1,1,0} , {1,0,1} , {0,1,1} , {1,1,1} , {0,0,0} };
    unsigned int color_id = id % 8;

    //float red_id = (id % 100)/100.0;
    //float green_id = ((id/2) % 100)/100.0;
    //float blue_id = ((id/3) % 100)/100.0;

    std_msgs::ColorRGBA c;

    c.a = 1.0;

    //c.r = red_id;
    //c.g = green_id;
    //c.b = blue_id;

    c.r = colors[color_id][0];
    c.g = colors[color_id][1];
    c.b = colors[color_id][2];

    return c;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

std_msgs::ColorRGBA getColor(const std::string& id)
{
    return getColor(getHash(id));
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void getNameAndTypeVisualizationMarker(const geo::Vector3& center_point, const UUID& name, const TYPE& type, visualization_msgs::Marker& m, int id, const std::string& ns)
{
    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.id = id;
    m.ns = ns;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();

    m.scale.z = 0.08;

    m.color.r = m.color.g = m.color.b = 1.0;

    geo::convert(center_point,m.pose.position);
    m.pose.position.z += 0.1;

    if (type=="")
        m.text = type + "(" + name.str().substr(0,4) +  ")";
    else
        m.text = name.str() + "(" + type.substr(0,4) +  ")";
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


void getCenterPointVisualizationMarker(bool in_frustrum, bool object_in_front, const geo::Vector3& center_point, visualization_msgs::Marker& m, int id, const std::string& ns)
{
    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::SPHERE;
    m.id = id;
    m.ns = ns;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();

    if (in_frustrum) {
        if (object_in_front) {
            m.color.r = m.color.g = 1.0;
            m.color.b = 0.0;
        }
        else
        {
            m.color.r = 1.0;
            m.color.g = m.color.b = 0.0;
        }
    }
    else
    {
        m.color.r = m.color.g = m.color.b = 1.0;
    }

    geo::convert(center_point,m.pose.position);

    m.scale.x = m.scale.y = m.scale.z = 0.05;
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void getHumanVisualizationMarkers(std::vector<visualization_msgs::Marker>& markers, const geo::Vector3& center_point, int id)
{
    visualization_msgs::Marker m;

    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::CYLINDER;
    m.id = id;
    m.ns = "human_body";

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();

    m.color.a = 1.0;
    m.color.r = 0.2; m.color.g = 0.7; m.color.b = 1.0;

    geo::convert(center_point,m.pose.position);
    m.pose.position.z = .5;

    m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 1.0;

    markers.push_back(m);

    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::SPHERE;
    m.id = id;
    m.ns = "human_head";

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();

    m.color.a = 1.0;
    m.color.r = 0.8; m.color.g = 0.3; m.color.b = 0.5;

    geo::convert(center_point,m.pose.position);
    m.pose.position.z = 1.15;

    m.scale.x = 0.3; m.scale.y = 0.3; m.scale.z = 0.3;

    markers.push_back(m);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void getConvexHullVisualizationMarker(visualization_msgs::Marker& m, const ConvexHull2D& polygon, int id, const std::string& ns)
{
    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::LINE_LIST;
    m.id = id;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.color = getColor(id);
    m.scale.x = 0.01;

    for (unsigned int j = 0; j < polygon.chull.points.size(); ++j) {

        geo::Vector3 p1a(polygon.chull.points[j].x,polygon.chull.points[j].y,polygon.min_z);
        geo::Vector3 p1b(polygon.chull.points[j].x,polygon.chull.points[j].y,polygon.max_z);

        geo::Vector3 p2a, p2b;
        if (j == polygon.chull.points.size() - 1) {
            p2a = geo::Vector3(polygon.chull.points[0].x,polygon.chull.points[0].y,polygon.min_z);
            p2b = geo::Vector3(polygon.chull.points[0].x,polygon.chull.points[0].y,polygon.max_z);
        } else {
            p2a = geo::Vector3(polygon.chull.points[j+1].x,polygon.chull.points[j+1].y,polygon.min_z);
            p2b = geo::Vector3(polygon.chull.points[j+1].x,polygon.chull.points[j+1].y,polygon.max_z);
        }

        geometry_msgs::Point p;

        // min line

        p.x = p1a.x;
        p.y = p1a.y;
        p.z = p1a.z;
        m.points.push_back(p);

        p.x = p2a.x;
        p.y = p2a.y;
        p.z = p2a.z;
        m.points.push_back(p);

        // max line

        p.x = p1b.x;
        p.y = p1b.y;
        p.z = p1b.z;
        m.points.push_back(p);

        p.x = p2b.x;
        p.y = p2b.y;
        p.z = p2b.z;
        m.points.push_back(p);

        // Vertical line

        p.x = p1a.x;
        p.y = p1a.y;
        p.z = p1a.z;
        m.points.push_back(p);

        p.x = p1b.x;
        p.y = p1b.y;
        p.z = p1b.z;
        m.points.push_back(p);

    }
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void publishWorldModelVisualizationMarkerArray(const WorldModel& world_model, const ros::Publisher& pub)
{
//    std::cout << "Entities in world model: " << entities.size() << std::endl;

    visualization_msgs::MarkerArray m_array;

    unsigned int i = 0;
    for (WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it )
    {
        const EntityConstPtr& e = *it;

        if (e->lastMeasurement()) {
//            helpers::visualization::showMeasurement(e->getBestMeasurement(), it->second->getType() + "-" + it->first);
        }

        // No floor, that sucks
        const std::string& id = e->id().str();
        if (id.size() >= 5 && id.substr(id.size() - 5) == "floor")
            continue;

        visualization_msgs::Marker m;// = m_array.markers[i];
        m.action = visualization_msgs::Marker::ADD;
        m.header.frame_id = "/map";
        m.header.stamp = ros::Time::now();
        m.lifetime = ros::Duration(0.5);
        m.color = getColor(e->id().str());

        geo::ShapeConstPtr shape = e->shape();

        if (shape) { // Do the shape

            m.type = visualization_msgs::Marker::TRIANGLE_LIST;
            m.scale.x = m.scale.y = m.scale.z = 1.0;
            m.color.a = 0.4;
            m.id = getHash(e->id().str());

            const geo::Mesh& mesh = shape->getMesh();
            const std::vector<geo::Vector3>& points = mesh.getPoints();
            const std::vector<geo::TriangleI>& triangles = mesh.getTriangleIs();

            m.points.resize(triangles.size()*3);
            for( unsigned int j = 0; j < triangles.size(); ++j )
            {
                geo::Vector3 p = e->pose() * points[triangles[j].i1_];
                m.points[3*j].x = p.x;
                m.points[3*j].y = p.y;
                m.points[3*j].z = p.z;

                p = e->pose() * points[triangles[j].i2_];
                m.points[3*j+1].x = p.x;
                m.points[3*j+1].y = p.y;
                m.points[3*j+1].z = p.z;

                p = e->pose() * points[triangles[j].i3_];
                m.points[3*j+2].x = p.x;
                m.points[3*j+2].y = p.y;
                m.points[3*j+2].z = p.z;
            }
            m_array.markers.push_back(m);

        } else { // Do the convex hull

            getConvexHullVisualizationMarker(m, e->convexHull(), getHash(e->id().str()), "ns");
            m_array.markers.push_back(m);

            // ARROW
            geometry_msgs::Point p1, p2;
            geo::convert(e->convexHull().center_point, p1);
            geo::convert(e->convexHull().center_point + e->velocity().getOrigin(), p2);

            m.scale.x = 0.02;
            m.scale.y = 0.05;
            m.scale.z = 0.05;
            m.color.a = m.color.r =  m.color.g = m.color.b = 1.0;
            m.type = visualization_msgs::Marker::ARROW;
            m.points.clear();
            m.points.push_back(p1);
            m.points.push_back(p2);

            m_array.markers.push_back(m);
        }

//        getCenterPointVisualizationMarker(e->in_frustrum, e->object_in_front, e->convexHull().center_point,m,m.id,"pose");
//        m_array.markers.push_back(m);

        if (e->lastMeasurement())
        {
            getNameAndTypeVisualizationMarker(e->convexHull().center_point,e->id(),e->type(),m,m.id,"name_and_type");
            m_array.markers.push_back(m);
            getCenterPointVisualizationMarker(false, false, e->convexHull().center_point, m, m.id, "center_point" );
            m_array.markers.push_back(m);
        }
        else
        {
            getNameAndTypeVisualizationMarker(e->pose().getOrigin(),e->id(),e->type(),m,m.id,"name_and_type");
            m_array.markers.push_back(m);
            getCenterPointVisualizationMarker(false, false, e->pose().getOrigin(), m, m.id, "center_point" );
            m_array.markers.push_back(m);
        }

        // If human

        if (e->type() == "human")
        {
            std::vector<visualization_msgs::Marker> human;
            getHumanVisualizationMarkers(human, e->convexHull().center_point, m.id);
            for (std::vector<visualization_msgs::Marker>::const_iterator it = human.begin(); it != human.end(); ++it)
            {
                m_array.markers.push_back(*it);
            }
        }

        ++i;

    }

    unsigned int j = 0;
    for (std::vector<visualization_msgs::Marker>::iterator it = m_array.markers.begin(); it != m_array.markers.end(); ++it)
    {
        it->id = j;
        ++j;
        it->lifetime = ros::Duration(0.5);
    }

    pub.publish(m_array);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void publishPclVisualizationMarker(const geo::Pose3D& pose, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& pc, const ros::Publisher& pub, int id, const std::string& ns)
{
    visualization_msgs::Marker m;

    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::POINTS;
    m.id = id;
    m.ns = ns;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();

    m.scale.x = 0.01;

    m.color = getColor(id);

    m.points.resize(pc->size());
    for (unsigned int i = 0; i < pc->size(); ++i)
    {
        const pcl::PointXYZ& p = pc->points[i];

        geo::Vector3 v(p.x, p.y, p.z);

        v = pose * v; // to the map frame

        m.points[i].x = v.x;
        m.points[i].y = v.y;
        m.points[i].z = v.z;
    }

    pub.publish(m);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void publishPclVisualizationMarker(const geo::Pose3D& pose, const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pc, const ros::Publisher& pub, int id, const std::string& ns)
{
    visualization_msgs::Marker m;

    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::POINTS;
    m.id = id;
    m.ns = ns;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();

    m.scale.x = 0.01;

    m.color = getColor(id);

    m.points.resize(pc->size());
    for (unsigned int i = 0; i < pc->size(); ++i)
    {
        const pcl::PointNormal& p = pc->points[i];

        geo::Vector3 v(p.x, p.y, p.z);

        v = pose * v; // to the map frame

        m.points[i].x = v.x;
        m.points[i].y = v.y;
        m.points[i].z = v.z;
    }

    pub.publish(m);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void publishNpclVisualizationMarker(const geo::Pose3D& pose, const pcl::PointCloud<pcl::PointNormal>::ConstPtr& pc, const ros::Publisher& pub, int id, const std::string& ns)
{
    visualization_msgs::Marker m;

    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::LINE_LIST;
    m.id = id;
    m.ns = ns;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();

    m.scale.x = 0.01;

    m.color.a = 1;
    double length = 0.1;

    std_msgs::ColorRGBA color = getColor(id);
    m.colors.resize(2 * pc->size());
    m.points.resize(2 * pc->size());
    for (unsigned int i = 0; i < pc->size(); ++i)
    {
        const pcl::PointNormal& p = pc->points[i];

        geo::Vector3 v(p.x, p.y, p.z);
        geo::Vector3 n(p.normal_x, p.normal_y, p.normal_z);
        v = pose * v; // to the map frame
        n = pose.getBasis() * n;

        m.points[2*i].x = v.x;
        m.points[2*i].y = v.y;
        m.points[2*i].z = v.z;

        m.points[2*i+1].x = v.x + length*n.x;
        m.points[2*i+1].y = v.y + length*n.y;
        m.points[2*i+1].z = v.z + length*n.z;

        m.colors[2*i].a = 1.0;
        m.colors[2*i+1].a = 1.0;

        m.colors[2*i].r = color.r * 0.2;
        m.colors[2*i+1].r = color.r * 1.0;
        m.colors[2*i].g = color.g * 0.2;
        m.colors[2*i+1].g = color.g * 1.0;
        m.colors[2*i].b = color.b * 0.2;
        m.colors[2*i+1].b = color.b * 1.0;
    }

    pub.publish(m);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void publishConvexHull2DVisualizationMarker(const ConvexHull2D& polygon, const ros::Publisher& pub, int id, const std::string& ns)
{
    visualization_msgs::Marker m;

    getConvexHullVisualizationMarker(m, polygon, id, ns);

    pub.publish(m);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void getRGBDViewFrustrumVisualizationMarker(visualization_msgs::Marker& m, const rgbd::View& view, const geo::Pose3D& pose, int id, const std::string& ns)
{
    float max_range = 2.5;

    m.action =  visualization_msgs::Marker::ADD;

    m.type = visualization_msgs::Marker::LINE_LIST;
    m.id = id;

    m.header.frame_id = "/map";
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.color = getColor(id);
    m.scale.x = 0.02;

    m.points.resize(16);
    geo::Vector3 p1 = pose * (max_range * view.getRasterizer().project2Dto3D(0,0));
    geo::Vector3 p2 = pose * (max_range * view.getRasterizer().project2Dto3D(view.getWidth()-1, 0));
    geo::Vector3 p3 = pose * (max_range * view.getRasterizer().project2Dto3D(view.getWidth()-1 ,view.getHeight()-1));
    geo::Vector3 p4 = pose * (max_range * view.getRasterizer().project2Dto3D(0, view.getHeight()-1));

    geo::convert(pose.getOrigin(), m.points[0]);
    geo::convert(p1, m.points[1]);

    geo::convert(pose.getOrigin(), m.points[2]);
    geo::convert(p2, m.points[3]);

    geo::convert(pose.getOrigin(), m.points[4]);
    geo::convert(p3, m.points[5]);

    geo::convert(pose.getOrigin(), m.points[6]);
    geo::convert(p4, m.points[7]);

    geo::convert(p1, m.points[8]);
    geo::convert(p2, m.points[9]);

    geo::convert(p2, m.points[10]);
    geo::convert(p3, m.points[11]);

    geo::convert(p3, m.points[12]);
    geo::convert(p4, m.points[13]);

    geo::convert(p4, m.points[14]);
    geo::convert(p1, m.points[15]);
}
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void publishRGBDViewFrustrumVisualizationMarker(const rgbd::View& view, const geo::Pose3D& pose, const ros::Publisher& pub, int id, const std::string& ns)
{
    visualization_msgs::Marker m;

    getRGBDViewFrustrumVisualizationMarker(m, view, pose, id, ns);

    pub.publish(m);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -


void showMeasurement(MeasurementConstPtr measurement, const std::string& id)
{
    cv::Mat black(measurement->image()->getRGBImage().rows, measurement->image()->getRGBImage().cols, CV_8UC3, cv::Scalar(0, 0, 0));

    rgbd::View view(*measurement->image(),measurement->image()->getRGBImage().cols);

    std::vector<cv::Point2i> scaled_indices;
    for(ImageMask::const_iterator it = measurement->imageMask().begin(); it != measurement->imageMask().end(); ++it)
    {
        black.at<cv::Vec3b>(*it) = view.getColor(it->x,it->y);
        scaled_indices.push_back(*it);
    }
    cv::Rect rect = cv::boundingRect(scaled_indices);

    //show the img
    cv::imshow(id,black(rect));
    cv::waitKey(3);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void showMeasurements(const WorldModel& world_model, rgbd::ImageConstPtr rgbd_image)
{
    cv::Mat color_img = rgbd_image->getRGBImage().clone() * 0.2;
    for (WorldModel::const_iterator it = world_model.begin(); it != world_model.end(); ++it)
    {
        const EntityConstPtr& e = *it;

        if (e && !e->shape()) //! if it has no shape
        {
            if (e->lastMeasurement())
            {
                MeasurementConstPtr m = e->lastMeasurement();

                if (m->timestamp() == rgbd_image->getTimestamp() && e->measurementSeq() > 5)
                {                   
                    std::vector<cv::Point2i> pnts;
                    for (ImageMask::const_iterator mit = m->imageMask().begin(color_img.cols); mit != m->imageMask().end(); ++mit)
                    {
                        color_img.at<cv::Vec3b>(*mit) = rgbd_image->getRGBImage().at<cv::Vec3b>(*mit);
                        pnts.push_back(*mit);
                    }

                    // get the bounding rectangle of the mask
                    cv::Rect bounding_rect = cv::boundingRect(pnts);

                    // calculate color components
                    std_msgs::ColorRGBA c_rgba = getColor(e->id().str());
                    int red = c_rgba.r * 255;
                    int green = c_rgba.g * 255;
                    int blue = c_rgba.b * 255;

                    // create Scalar color with a minimum
                    cv::Scalar color(std::max(red, 80), std::max(green, 80), std::max(blue, 80));

                    // draw bounding box rectangle
                    cv::rectangle(color_img, bounding_rect, color, 2);

//                    std::vector<cv::Point2i> chull;
//                    cv::convexHull(pnts,chull);
//                    std::vector<std::vector<cv::Point2i> > contours; contours.push_back(chull);
//                    // draw convex hull contours
//                    cv::drawContours(color_img,contours,0,color, 1);

                    tue::config::Reader config(e->data());
                    std::string type;
                    std::string info ;//= e->id().substr(0,4);
                    float score = 0;

                    // update type given by perception modules type and certainty
                    if (config.readGroup("perception_result", tue::config::OPTIONAL))
                    {
                        if (config.readGroup("type_aggregator", tue::config::OPTIONAL))
                        {
                            if (config.value("type", type, tue::config::OPTIONAL) &&
                                config.value("score", score, tue::config::OPTIONAL)){
                                info = boost::str(boost::format("%.2f") % score);
                            }
                        }
                        config.endGroup(); // close type_aggregator group
                    }
                    config.endGroup();  // close perception_result group

                    // if no type was read, use the default and the UID
                    if (type.empty()){
                        type = e->type();
                        info = e->id().str().substr(0,4);
                    }else if (type.compare("unknown") == 0){
                        type = "";
                        info = e->id().str().substr(0,4);
                    }else if (type.compare("human") == 0){
                        // in case type is human, replace by name
                        if (config.readGroup("perception_result", tue::config::OPTIONAL)){
                            if (config.readGroup("face_recognizer", tue::config::OPTIONAL))
                            {
                                std::string person_name;
                                if (config.value("label", person_name, tue::config::OPTIONAL) &&
                                    config.value("score", score, tue::config::OPTIONAL)){
                                    if (!person_name.empty() && score > 0){
                                        type = person_name;
                                        info = boost::str(boost::format("%.2f") % score);
                                    }
                                }
                            }
                            config.endGroup(); // close type_aggregator group
                        }
                        config.endGroup();  // close perception_result group
                    }

                    // draw name background rectangle
                    cv::rectangle(color_img, cv::Point(bounding_rect.x, bounding_rect.y) + cv::Point(0, -22),
                                  cv::Point(bounding_rect.x, bounding_rect.y) + cv::Point(((type.size() + 6) * 10), -2),
                                  color - cv::Scalar(140, 140, 140), CV_FILLED);

                    // draw name and ID
                    cv::putText(color_img, type + "(" + info + ")",
                                cv::Point(bounding_rect.x, bounding_rect.y) + cv::Point(5, -8),
                                1, 1.0, color, 1, CV_AA);
                }
            }
        }
    }

//    static cv::VideoWriter v("output.avi", CV_FOURCC('M','J','P','G'), 10, cv::Size(color_img.cols, color_img.rows));
//    v.write(color_img);

    cv::imshow("measurements", color_img);
    cv::waitKey(3);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void showSegmentedImage(const rgbd::ImageConstPtr image, const std::vector<ImageMask>& segments, const std::string& name)
{
    rgbd::View view(*image,640);

    cv::Rect rect(0,0,view.getWidth(),view.getHeight());
    cv::Mat original_img = image->getRGBImage().clone();
    cv::Mat img = original_img(rect);

    for (unsigned int i = 0; i < segments.size(); ++i) {

        std_msgs::ColorRGBA c = getColor(i);

        const ImageMask& seg = segments[i];
        for(ImageMask::const_iterator it = seg.begin(img.cols); it != seg.end(); ++it)
        {
            img.at<cv::Vec3b>(*it) = cv::Vec3b(c.b*255,c.g*255,c.r*255);
        }

    }

    cv::imshow(name,img);

    cv::waitKey(3);
}

// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

void showMask(const ImageMask& mask, const std::string& name)
{
    cv::Mat img(mask.height(), mask.width(), CV_32FC1, 0.0);
    for(ImageMask::const_iterator it = mask.begin(img.cols); it != mask.end(); ++it)
    {
        img.at<float>(*it) = 1;
    }

    cv::imshow(name, img);
    cv::waitKey(3);
}

}

}

}
