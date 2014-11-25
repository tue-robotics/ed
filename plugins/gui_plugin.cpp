#include "gui_plugin.h"

#include <ed/entity.h>
#include <ed/measurement.h>
#include <ed/world_model.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ros/node_handle.h>
#include <ros/advertise_service_options.h>

#include <geolib/Shape.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

class ColorRenderResult : public geo::RenderResult
{

public:

    ColorRenderResult(cv::Mat& image, cv::Mat& z_buffer, const cv::Vec3b& color, float min_depth, float max_depth)
        : geo::RenderResult(image.rows, image.rows), image_(image), z_buffer_(z_buffer), color_(color),
          min_depth_(min_depth), max_depth_(max_depth)
    {
    }

    void renderPixel(int x, int y, float depth, int i_triangle)
    {
        float old_depth = z_buffer_.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            float f = (depth - min_depth_) / (max_depth_ - min_depth_);

            f = 1.0 - std::min(1.0f, std::max(0.0f, f));

            z_buffer_.at<float>(y, x) = depth;
            image_.at<cv::Vec3b>(y, x) = f * color_;
        }
    }

protected:

    cv::Mat image_;
    cv::Mat z_buffer_;
    cv::Vec3b color_;
    float min_depth_, max_depth_;


};

// ----------------------------------------------------------------------------------------------------

bool inPolygon(const std::vector<cv::Point2i>& points_list, const cv::Point2i& point)
{
    int nvert = points_list.size();
    float vertx[nvert];
    float verty[nvert];
    float testx = point.x;
    float testy = point.y;

    unsigned int ii = 0;
    for(; ii < points_list.size(); ++ii){
        vertx[ii] = points_list[ii].x;
        verty[ii] = points_list[ii].y;
    }

    int i, j, c = 0;
    for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
        (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
        c = !c;
    }
    return c > 0;
}

// ----------------------------------------------------------------------------------------------------

int REDS[] =   { 255, 0  , 255, 0,   255, 0  , 255};
int GREENS[] = { 255, 255, 0  , 0,   255, 255, 0  };
int BLUES[] =  { 255, 255, 255, 255, 0,   0,   0  };

// ----------------------------------------------------------------------------------------------------

int hash(const char *str, int max_val)
{
    unsigned long hash = 5381;
    int c;

    while ((c = *str++))
        hash = ((hash << 5) + hash) + c; /* hash * 33 + c */

    return hash % max_val;
}

// ----------------------------------------------------------------------------------------------------

cv::Scalar idToColor(const ed::UUID& id)
{
    int i = hash(id.c_str(), 7);
    return cv::Scalar(BLUES[i], GREENS[i], REDS[i]);
}

// ----------------------------------------------------------------------------------------------------

enum ImageCompressionType
{
    IMAGE_COMPRESSION_JPG,
    IMAGE_COMPRESSION_PNG
};

bool imageToBinary(const cv::Mat& image, std::vector<unsigned char>& data, ImageCompressionType compression_type)
{
    if (compression_type == IMAGE_COMPRESSION_JPG)
    {
        // OpenCV compression settings
        std::vector<int> rgb_params;
        rgb_params.resize(3, 0);

        rgb_params[0] = CV_IMWRITE_JPEG_QUALITY;
        rgb_params[1] = 95; // default is 95

        // Compress image
        if (!cv::imencode(".jpg", image, data, rgb_params)) {
            std::cout << "RGB image compression failed" << std::endl;
            return false;
        }
    }
    else if (compression_type == IMAGE_COMPRESSION_PNG)
    {
        std::vector<int> params;
        params.resize(3, 0);

        params[0] = CV_IMWRITE_PNG_COMPRESSION;
        params[1] = 1;

        if (!cv::imencode(".png", image, data, params)) {
            std::cout << "PNG image compression failed" << std::endl;
            return false;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

GUIPlugin::GUIPlugin()
{
    command_ = "explore";
    command_id_ = ed::Entity::generateID();
    t_command_ = ros::Time::now();
}

// ----------------------------------------------------------------------------------------------------

GUIPlugin::~GUIPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void GUIPlugin::configure(tue::Configuration config)
{
    std::string srv_get_measurements, srv_set_label, srv_raise_event, srv_get_command, map_image_topic;

    config.value("srv_get_measurements", srv_get_measurements);
    config.value("srv_set_label", srv_set_label);
    config.value("srv_raise_event", srv_raise_event);
    config.value("srv_get_command", srv_get_command);
    config.value("map_image_topic", map_image_topic);

    // TODO: make configurable
    trigger_map_publish_ = ed::EventClock(10);

    int image_width, image_height;
    config.value("map_image_width", image_width);
    config.value("map_image_height", image_height);

    double world_width, world_height;
    config.value("world_width", world_width);
    config.value("world_height", world_height);

    double cam_x, cam_y, cam_z;
    config.value("cam_x", cam_x);
    config.value("cam_y", cam_y);
    config.value("cam_z", cam_z);

    if (config.hasError())
        return;

    ros::NodeHandle nh;

    if (srv_get_measurements_.getService() != srv_get_measurements)
    {
        ros::AdvertiseServiceOptions opt_get_measurements =
                ros::AdvertiseServiceOptions::create<ed::GetMeasurements>(
                    srv_get_measurements, boost::bind(&GUIPlugin::srvGetMeasurements, this, _1, _2), ros::VoidPtr(), &cb_queue_);
        srv_get_measurements_ = nh.advertiseService(opt_get_measurements);
    }

    if (srv_set_label_.getService() != srv_set_label)
    {
        ros::AdvertiseServiceOptions opt_set_label =
                ros::AdvertiseServiceOptions::create<ed::SetLabel>(
                    srv_set_label, boost::bind(&GUIPlugin::srvSetLabel, this, _1, _2), ros::VoidPtr(), &cb_queue_);
        srv_set_label_ = nh.advertiseService(opt_set_label);
    }

    if (srv_raise_event_.getService() != srv_raise_event)
    {
        ros::AdvertiseServiceOptions opt_raise_event =
                ros::AdvertiseServiceOptions::create<ed::RaiseEvent>(
                    srv_raise_event, boost::bind(&GUIPlugin::srvRaiseEvent, this, _1, _2), ros::VoidPtr(), &cb_queue_);
        srv_raise_event_ = nh.advertiseService(opt_raise_event);
    }

    if (srv_get_command_.getService() != srv_get_command)
    {
        ros::AdvertiseServiceOptions opt_get_command =
                ros::AdvertiseServiceOptions::create<ed::GetGUICommand>(
                    srv_get_command, boost::bind(&GUIPlugin::srvGetCommand, this, _1, _2), ros::VoidPtr(), &cb_queue_);
        srv_get_command_ = nh.advertiseService(opt_get_command);
    }

    if (pub_image_map_.getTopic() != map_image_topic)
        pub_image_map_ = nh.advertise<tue_serialization::Binary>("/ed/gui/map_image", 1);

    map_image_ = cv::Mat(image_width, image_height, CV_8UC3, cv::Scalar(50, 50, 50));

    projector_pose_.setOrigin(geo::Vector3(cam_x, cam_y, cam_z));
    projector_pose_.setBasis(geo::Matrix3::identity());

    projector_.setFocalLengths(image_width / (world_width / cam_z),
                               image_height / (world_height / cam_z));
    projector_.setOpticalCenter(image_width / 2, image_height / 2);
    projector_.setOpticalTranslation(0, 0);
}

// ----------------------------------------------------------------------------------------------------

void GUIPlugin::initialize()
{
}

// ----------------------------------------------------------------------------------------------------

void GUIPlugin::process(const ed::WorldModel& world, ed::UpdateRequest& req)
{
    world_model_ = &world;

    handleRequests();

    publishMapImage();
}

// ----------------------------------------------------------------------------------------------------

cv::Point2i GUIPlugin::coordinateToPixel(const geo::Vector3& p) const
{
    return projector_.project3Dto2D(projector_pose_.inverse() * p);
}

// ----------------------------------------------------------------------------------------------------

ed::UUID GUIPlugin::getEntityFromClick(const cv::Point2i& p) const
{
    for(ed::WorldModel::const_iterator it_entity = world_model_->begin(); it_entity != world_model_->end(); ++it_entity)
    {
        const ed::EntityConstPtr& e = *it_entity;
        if (!e->shape())
        {
            const pcl::PointCloud<pcl::PointXYZ>& chull_points = e->convexHull().chull;

            if (!chull_points.empty())
            {
                std::vector<cv::Point2i> image_chull(chull_points.size());
                for(unsigned int i = 0; i < chull_points.size(); ++i)
                    image_chull[i] = coordinateToPixel(chull_points[i]);

                if (inPolygon(image_chull, p))
                    return e->id();
            }
        }
    }

    return "";
}

// ----------------------------------------------------------------------------------------------------

void GUIPlugin::publishMapImage()
{
    // clear image
    map_image_ = cv::Mat(map_image_.rows, map_image_.cols, CV_8UC3, cv::Scalar(10, 10, 10));

    // Draw world model shapes

    cv::Mat z_buffer(map_image_.rows, map_image_.cols, CV_32FC1, 0.0);

    for(ed::WorldModel::const_iterator it_entity = world_model_->begin(); it_entity != world_model_->end(); ++it_entity)
    {
        const ed::EntityConstPtr& e = *it_entity;

        if (e->shape() && e->id() != "floor")
        {
            cv::Scalar color = idToColor(e->id());
            cv::Vec3b color_vec(0.5 * color[0], 0.5 * color[1], 0.5 * color[2]);

            geo::Pose3D pose = projector_pose_.inverse() * e->pose();
            geo::RenderOptions opt;
            opt.setMesh(e->shape()->getMesh(), pose);

            ColorRenderResult res(map_image_, z_buffer, color_vec, projector_pose_.t.z - 3, projector_pose_.t.z + 1);

            // Render
            projector_.render(opt, res);
        }
    }

    for(ed::WorldModel::const_iterator it_entity = world_model_->begin(); it_entity != world_model_->end(); ++it_entity)
    {
        const ed::EntityConstPtr& e = *it_entity;

        const pcl::PointCloud<pcl::PointXYZ>& chull_points = e->convexHull().chull;

        cv::Scalar color = idToColor(e->id());
        int thickness = 1;
        if (selected_id_ == e->id())
            thickness = 3;

        // draw the polygon
        if (!chull_points.empty())
        {
            // Lower polygon
            for(unsigned int i = 0; i < chull_points.size(); ++i)
            {
                int j = (i + 1) % chull_points.size();

                const pcl::PointXYZ& p1 = chull_points[i];
                const pcl::PointXYZ& p2 = chull_points[j];

                cv::line(map_image_,
                         coordinateToPixel(p1.x, p1.y, e->convexHull().min_z),
                         coordinateToPixel(p2.x, p2.y, e->convexHull().min_z),
                         0.3 * color, thickness);
            }

            // Edges in between
            for(unsigned int i = 0; i < chull_points.size(); ++i)
            {
                const pcl::PointXYZ p = chull_points[i];
                cv::line(map_image_,
                         coordinateToPixel(p.x, p.y, e->convexHull().min_z),
                         coordinateToPixel(p.x, p.y, e->convexHull().max_z),
                         0.5 * color, thickness);
            }


            // Upper polygon
            for(unsigned int i = 0; i < chull_points.size(); ++i)
            {
                int j = (i + 1) % chull_points.size();

                const pcl::PointXYZ& p1 = chull_points[i];
                const pcl::PointXYZ& p2 = chull_points[j];

                cv::line(map_image_,
                         coordinateToPixel(p1.x, p1.y, e->convexHull().max_z),
                         coordinateToPixel(p2.x, p2.y, e->convexHull().max_z),
                         color, thickness);
            }

            if (e->type() == "person")
            {

                cv::circle(map_image_, coordinateToPixel(e->convexHull().center_point), 15, cv::Scalar(255, 0, 0), 10);
            }
        }
    }

    // Find the global most recent measurement
    ed::MeasurementConstPtr last_measurement;
    for(ed::WorldModel::const_iterator it_entity = world_model_->begin(); it_entity != world_model_->end(); ++it_entity)
    {
        const ed::EntityConstPtr& e = *it_entity;
        if (e->lastMeasurement() && (!last_measurement || e->lastMeasurement()->timestamp() > e->lastMeasurement()->timestamp()))
            last_measurement = e->lastMeasurement();
    }

    if (last_measurement)
    {
        const geo::Pose3D& sensor_pose = last_measurement->sensorPose();

        // Draw sensor origin
        const geo::Vector3& p = sensor_pose.getOrigin();
        cv::Point2i p_2d = coordinateToPixel(p);

        cv::circle(map_image_, p_2d, 10, cv::Scalar(255, 255, 255));

        rgbd::View view(*last_measurement->image(), 100); // width doesnt matter; we'll go back to world coordinates anyway
        geo::Vector3 p1 = view.getRasterizer().project2Dto3D(0, 0) * 3;
        geo::Vector3 p2 = view.getRasterizer().project2Dto3D(view.getWidth() - 1, 0) * 3;
        geo::Vector3 p3 = view.getRasterizer().project2Dto3D(0, view.getHeight() - 1) * 3;
        geo::Vector3 p4 = view.getRasterizer().project2Dto3D(view.getWidth() - 1, view.getHeight() - 1) * 3;

        cv::Point2i p1_2d = coordinateToPixel(sensor_pose * p1);
        cv::Point2i p2_2d = coordinateToPixel(sensor_pose * p2);
        cv::Point2i p3_2d = coordinateToPixel(sensor_pose * p3);
        cv::Point2i p4_2d = coordinateToPixel(sensor_pose * p4);

        cv::Scalar fustrum_color(100, 100, 100);

        cv::line(map_image_, p_2d, p1_2d, fustrum_color);
        cv::line(map_image_, p_2d, p2_2d, fustrum_color);
        cv::line(map_image_, p_2d, p3_2d, fustrum_color);
        cv::line(map_image_, p_2d, p4_2d, fustrum_color);

        cv::line(map_image_, p1_2d, p2_2d, fustrum_color);
        cv::line(map_image_, p1_2d, p3_2d, fustrum_color);
        cv::line(map_image_, p2_2d, p4_2d, fustrum_color);
        cv::line(map_image_, p3_2d, p4_2d, fustrum_color);
    }

    if (t_last_click_ > ros::Time(0) && t_last_click_ > ros::Time::now() - ros::Duration(1))
    {
        cv::circle(map_image_, p_click_, 20, cv::Scalar(0, 0, 255), 5);
    }

    // Convert to binary
    tue_serialization::Binary msg;
    if (imageToBinary(map_image_, msg.data, IMAGE_COMPRESSION_PNG))
    {
        pub_image_map_.publish(msg);
    }

//    cv::imshow("gui", map_image_);
//    cv::waitKey(3);
}

// ----------------------------------------------------------------------------------------------------

void GUIPlugin::handleRequests()
{
    cb_queue_.callAvailable();
}

// ----------------------------------------------------------------------------------------------------

// Returns measurement of currently selected id
bool GUIPlugin::srvGetMeasurements(ed::GetMeasurements::Request& req, ed::GetMeasurements::Response& res)
{
    if (selected_id_.empty())
        return true;

    ed::EntityConstPtr e = world_model_->getEntity(selected_id_);
    if (!e)
        return true;

    ed::MeasurementConstPtr m = e->bestMeasurement();
    if (m)
    {
        const cv::Mat& rgb_image = m->image()->getRGBImage();
        const ed::ImageMask& image_mask = m->imageMask();

        cv::Mat rgb_image_masked(rgb_image.rows, rgb_image.cols, CV_8UC3, cv::Scalar(0, 0, 0));

        for(ed::ImageMask::const_iterator it = image_mask.begin(rgb_image.cols); it != image_mask.end(); ++it)
            rgb_image_masked.at<cv::Vec3b>(*it) = rgb_image.at<cv::Vec3b>(*it);

        res.images.push_back(tue_serialization::Binary());
        if (imageToBinary(rgb_image_masked, res.images.back().data, IMAGE_COMPRESSION_JPG))
        {
            res.ids.push_back(e->id());
            res.images.back().info = e->type();
        }
        else
            res.images.pop_back();
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool GUIPlugin::srvSetLabel(ed::SetLabel::Request& req, ed::SetLabel::Response& res)
{
    ed::UUID id = req.id;
    if (id.empty())
        id = selected_id_;

    if (id.empty())
    {
        std::cout << "GUIServer: srvSetLabel: no id given or selected" << std::endl;
        return true;
    }

    ed::EntityConstPtr e = world_model_->getEntity(id);
    if (!e)
    {
        res.msg = "GUIServer: srvSetLabel: Id does not exist: " + id;
        std::cout << res.msg << std::endl;
    }
    else
    {
        // Make a copy of the entity
        ed::EntityPtr e_updated(new ed::Entity(*e));

        // Set the label
        e_updated->setType(req.label);

        // TODO: fill update request to set the label
        // ...

        res.msg = "[ED] GUI Plugin: srv SetLabel not yet implemented.";
        std::cout << res.msg << std::endl;


//        std::cout << "Setting entity '" << id << "' to type '" << req.label << "'" << std::endl;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool parseParam(const std::map<std::string, std::string>& params, const std::string& key, float& value)
{
    std::map<std::string, std::string>::const_iterator it = params.find(key);
    if (it == params.end())
        return false;
    value = atof(it->second.c_str());
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool parseParam(const std::map<std::string, std::string>& params, const std::string& key, std::string& value)
{
    std::map<std::string, std::string>::const_iterator it = params.find(key);
    if (it == params.end())
        return false;
    value = it->second.c_str();
    return true;
}

// ----------------------------------------------------------------------------------------------------

bool GUIPlugin::srvRaiseEvent(ed::RaiseEvent::Request& req, ed::RaiseEvent::Response& res)
{
    std::map<std::string, std::string> params;
    for(unsigned int i = 0; i < req.param_names.size(); ++i)
        params[req.param_names[i]] = req.param_values[i];

    if (req.name == "click")
    {
        float x, y;
        std::string type;
        if (parseParam(params, "x", x) && parseParam(params, "y", y) && parseParam(params, "type", type))
        {
            p_click_ = cv::Point2i(x, y);
            t_last_click_ = ros::Time::now();
            std::cout << "Detected click at " << params["x"] << ", " << params["y"] << std::endl;

            command_ = "";
            selected_id_ = getEntityFromClick(p_click_);

            if (type == "delete")
            {
                if (!selected_id_.empty())
                {
                    // TODO: implement deletion
                    res.msg = "Deletion not yet implemented";
                    std::cout << "[ED] GUI Plugin: " << res.msg << std::endl;


//                    world_model_->erase(selected_id_);
//                    res.msg = "Deleted object with id '" + selected_id_ + "'";
                }
            }
            else if (type == "navigate")
            {
                command_ = "navigate";
                command_params_["id"] = selected_id_;
                res.msg = "Navigating to '" + selected_id_ + "'";
            }
            else if (type == "select")
            {
                res.msg = "Selected '" + selected_id_ + "'";
            }
            else
            {
                res.msg = "Unknown click type: " + type;
            }

            if (!command_.empty())
            {
                t_command_ = ros::Time::now();
                command_id_ = ed::Entity::generateID();
            }
        }
        else
        {
            res.msg = "Could not parse click event";
        }
    }
    else if (req.name == "zoom")
    {
        float f;
        if (parseParam(params, "factor", f))
        {
            float z_current = projector_pose_.t.z;
            projector_pose_.t.z = z_current / f;
        }
    }
    else if (req.name == "pan")
    {
        float dx, dy;
        if (parseParam(params, "dx", dx) && parseParam(params, "dy", dy))
        {
            projector_pose_.t.x += dx;
            projector_pose_.t.y += dy;
        }
    }
    else if (req.name == "explore")
    {
        command_ = "explore";
        t_command_ = ros::Time::now();
        command_id_ = ed::Entity::generateID();
    }
    else if (req.name == "wait")
    {
        command_ = "wait";
        t_command_ = ros::Time::now();
        command_id_ = ed::Entity::generateID();
    }
    else
    {
        res.msg = "Unknown event received: " + req.name;
    }

    std::cout << "ServerGUI: RaiseEvent: " << res.msg << std::endl;

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool GUIPlugin::srvGetCommand(ed::GetGUICommand::Request& req, ed::GetGUICommand::Response& res)
{
    if (!command_.empty())
    {
        res.age = ros::Time::now() - t_command_;
        res.command = command_;
        res.command_id = command_id_;

        for(std::map<std::string, std::string>::const_iterator it = command_params_.begin(); it != command_params_.end(); ++it)
        {
            res.param_names.push_back(it->first);
            res.param_values.push_back(it->second);
        }
    }

    return true;
}


ED_REGISTER_PLUGIN(GUIPlugin)
