cmake_minimum_required(VERSION 2.8.3)
project(ed)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  class_loader
  cv_bridge
  ed_object_models
  geolib2
  geometry_msgs
  rgbd
  pcl_conversions
  roscpp
  roslib
  std_srvs
  tue_config
  tue_filesystem
  tue_serialization
  message_generation
  visualization_msgs
  nav_msgs
  tue_config
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)

# Generate messages in the 'msg' folder
add_message_files(
   FILES
   EntityInfo.msg
)

# Generate services in the 'srv' folder
add_service_files(
   FILES
   GetGUICommand.srv
   GetMeasurements.srv
   RaiseEvent.srv
   SetClick.srv
   SetLabel.srv
   SimpleQuery.srv
   LoadPlugin.srv

   # For builder plugin
   SetEntity.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   geometry_msgs
   tue_serialization
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ed_core ed_io
    CATKIN_DEPENDS tue_config tue_serialization rgbd ed_object_models
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include
    ${PCL_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
)

# ------------------------------------------------------------------------------------------------
#                                          ED CORE
# ------------------------------------------------------------------------------------------------

# Get all the headers
file(GLOB_RECURSE HEADER_FILES include/*.h)

# Declare a cpp library
add_library(ed_core
    src/entity.cpp
    src/measurement.cpp
    src/update_request.cpp

    # DDP is used by measurement.cpp, so put here
    src/helpers/depth_data_processing.cpp
    src/helpers/clipper/clipper.cpp
    ${HEADER_FILES}
)
target_link_libraries(ed_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ed_core ${PROJECT_NAME}_gencpp)

add_library(ed_perception
    src/perception_modules/perception_module.cpp
    src/perception.cpp
    src/perception_worker.cpp
    # src/perception/aggregator.cpp
    src/perception/model_fitter.cpp
)
target_link_libraries(ed_perception ed_core)
add_dependencies(ed_perception ${PROJECT_NAME}_gencpp)

add_library(ed_io
    src/serialization/serialization.cpp
    src/io/filesystem/read.cpp
    src/io/filesystem/write.cpp
    src/io/transport/probe.cpp
    src/io/transport/probe_ros.cpp
    src/io/transport/probe_client.cpp
)
target_link_libraries(ed_io ed_core)
add_dependencies(ed_io ${PROJECT_NAME}_gencpp)

add_library(ed_association
    src/association_localization_modules/point_normal_alm.cpp
    src/association_localization_modules/polygon_height_alm.cpp
    src/association_localization_modules/world_model_renderer.cpp
)
add_dependencies(ed_association ${PROJECT_NAME}_gencpp)

add_library(ed_segmentation
    src/segmentation_modules/euclidean_clustering_sm.cpp
    src/segmentation_modules/floodfill_sm.cpp
)
add_dependencies(ed_segmentation ${PROJECT_NAME}_gencpp)

add_library(ed_kinect
    src/sensor_modules/kinect.cpp
)
target_link_libraries(ed_kinect ed_segmentation ed_association)
add_dependencies(ed_kinect ${PROJECT_NAME}_gencpp)

add_library(ed_visualization
    src/helpers/visualization.cpp
)
target_link_libraries(ed_visualization tue_config)
add_dependencies(ed_visualization ${PROJECT_NAME}_gencpp)

# ------------------------------------------------------------------------------------------------
#                                               SERVER
# ------------------------------------------------------------------------------------------------

# Create executable
add_executable(ed_server
    src/ed.cpp
    src/server.cpp
    src/plugin_container.cpp
)
target_link_libraries(ed_server ed_core ed_perception ed_association ed_segmentation ed_io ed_kinect ed_visualization)
add_dependencies(ed_server ${PROJECT_NAME}_gencpp)

# ------------------------------------------------------------------------------------------------
#                                               PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(ed_occupancy_grid_publisher_plugin plugins/occupancy_grid_publisher_plugin.cpp)
add_dependencies(ed_occupancy_grid_publisher_plugin ${PROJECT_NAME}_gencpp)

add_library(ed_hello_world_plugin plugins/hello_world_plugin.cpp)

add_library(ed_tf_publisher_plugin plugins/tf_publisher_plugin.cpp)

add_library(ed_robot_plugin plugins/robot_plugin.cpp)

add_library(ed_gui_plugin plugins/gui_plugin.cpp)
add_dependencies(ed_gui_plugin ${PROJECT_NAME}_gencpp)

#add_library(ed_builder_plugin plugins/builder_plugin.cpp)
#add_dependencies(ed_builder_plugin ${PROJECT_NAME}_gencpp)

# ------------------------------------------------------------------------------------------------
#                                                TESTS
# ------------------------------------------------------------------------------------------------

add_executable(test_mask test/test_mask.cpp)
target_link_libraries(test_mask ed_core ${OpenCV_LIBRARIES})
add_dependencies(test_mask ${PROJECT_NAME}_gencpp)

add_executable(test_sampling test/test_sampling.cpp)
target_link_libraries(test_sampling ed_core ed_visualization ${OpenCV_LIBRARIES})
add_dependencies(test_sampling ${PROJECT_NAME}_gencpp)

#add_executable(test_human_classifier test/test_human_classifier.cpp)
#target_link_libraries(test_human_classifier human_contour_matcher ${OpenCV_LIBRARIES})
#target_link_libraries(test_human_classifier face_detector ${OpenCV_LIBRARIES})
#add_dependencies(test_human_classifier ${PROJECT_NAME}_gencpp)

add_executable(test_service_speed test/test_service_speed.cpp)
target_link_libraries(test_service_speed ed_core)
add_dependencies(test_service_speed ${PROJECT_NAME}_gencpp)

add_executable(show_gui test/show_gui.cpp)
target_link_libraries(show_gui ed_core ${OpenCV_LIBRARIES})
add_dependencies(show_gui ${PROJECT_NAME}_gencpp)

