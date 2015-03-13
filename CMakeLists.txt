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
   UpdateSrv.srv

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
    LIBRARIES ed_core ed_io ed_perception ed_visualization
    CATKIN_DEPENDS tue_config tue_serialization rgbd ed_object_models
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
    include
    3rdparty
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
    src/world_model.cpp
    src/transform_cache.cpp

    # Model loading
    src/models/model_loader.cpp
    src/models/shape_loader.cpp
    src/models/xml_shape_parser.cpp
    3rdparty/polypartition/polypartition.cpp

    # DDP is used by measurement.cpp, so put here
    src/helpers/depth_data_processing.cpp
    src/helpers/clipper/clipper.cpp

    src/error_context.cpp
    include/ed/error_context.h

    ${HEADER_FILES}
)
target_link_libraries(ed_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ed_core ${PROJECT_NAME}_gencpp)

add_library(ed_perception
    src/perception_modules/perception_module.cpp
    src/perception.cpp
    src/perception_worker.cpp
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

add_library(ed_visualization
    include/ed/helpers/image_publisher.h
    src/helpers/image_publisher.cpp
    src/helpers/visualization.cpp
)
target_link_libraries(ed_visualization ${catkin_LIBRARIES})
add_dependencies(ed_visualization ${PROJECT_NAME}_gencpp)

# ------------------------------------------------------------------------------------------------
#                                               SERVER
# ------------------------------------------------------------------------------------------------

# Create executable
add_executable(ed
    src/ed.cpp
    src/server.cpp
    src/plugin_container.cpp
)
target_link_libraries(ed ed_core ed_perception ed_io ed_visualization)
add_dependencies(ed ${PROJECT_NAME}_gencpp)

# ------------------------------------------------------------------------------------------------
#                                               PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(ed_tf_publisher_plugin plugins/tf_publisher_plugin.cpp)

find_package(kdl_parser REQUIRED)
add_library(ed_robot_plugin plugins/robot_plugin.cpp)
target_link_libraries(ed_robot_plugin ${kdl_parser_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(ed-list-ids tools/list_ids.cpp)

add_executable(ed_view_model tools/view_model.cpp)
target_link_libraries(ed_view_model ed_core)

#add_executable(ed_repl tools/repl.cpp)
#target_link_libraries(ed_repl readline)

# ------------------------------------------------------------------------------------------------
#                                               EXAMPLES
# ------------------------------------------------------------------------------------------------

add_library(ed_hello_world_plugin examples/hello_world/hello_world_plugin.cpp)
add_library(ed_custom_properties_plugin examples/custom_properties/custom_properties_plugin.cpp)

# ------------------------------------------------------------------------------------------------
#                                                TESTS
# ------------------------------------------------------------------------------------------------

add_executable(ed_test_wm test/test_wm.cpp)
target_link_libraries(ed_test_wm ed_core ${OpenCV_LIBRARIES})

add_executable(test_mask test/test_mask.cpp)
target_link_libraries(test_mask ed_core ${OpenCV_LIBRARIES})
add_dependencies(test_mask ${PROJECT_NAME}_gencpp)

add_executable(test_sampling test/test_sampling.cpp)
target_link_libraries(test_sampling ed_core ed_visualization ${OpenCV_LIBRARIES})
add_dependencies(test_sampling ${PROJECT_NAME}_gencpp)

add_executable(test_service_speed test/test_service_speed.cpp)
target_link_libraries(test_service_speed ed_core)
add_dependencies(test_service_speed ${PROJECT_NAME}_gencpp)

add_executable(show_gui test/show_gui.cpp)
target_link_libraries(show_gui ed_core ${OpenCV_LIBRARIES})
add_dependencies(show_gui ${PROJECT_NAME}_gencpp)

add_executable(ed_test_heightmap_triangulation
	test/test_heightmap_triangulation.cpp
	3rdparty/polypartition/polypartition.cpp
)
target_link_libraries(ed_test_heightmap_triangulation ed_core ${OpenCV_LIBRARIES} ${geolib2_LIBRARIES} ${tue_config_LIBRARIES})


