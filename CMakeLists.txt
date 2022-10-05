cmake_minimum_required(VERSION 3.0.2)
project(ed)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  code_profiler
  cv_bridge
  diagnostic_updater
  ed_msgs
  geolib2
  kdl_parser
  pluginlib
  rgbd
  rgbd_msgs
  rosconsole_bridge
  roscpp
  tue_config
  tue_filesystem
  tue_serialization
)

find_package(PCL REQUIRED COMPONENTS common)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(SDFormat REQUIRED)
find_package(TinyXML2 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ed_core ed_io ed_server
  CATKIN_DEPENDS code_profiler cv_bridge ed_msgs geolib2 kdl_parser pluginlib rgbd rgbd_msgs rosconsole_bridge roscpp tue_config tue_filesystem tue_serialization
  DEPENDS OpenCV PCL
)

###########
## Build ##
###########

add_compile_options(-Wreturn-type)
add_compile_options(-Wsign-compare)
add_compile_options(-Wreorder)

include_directories(
  include
  3rdparty/polypartition/include
  3rdparty/rapidjson/include
  ${TinyXML2_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(polypartition 3rdparty/polypartition/src/polypartition.cpp)

# ------------------------------------------------------------------------------------------------
#                                          ED CORE
# ------------------------------------------------------------------------------------------------

# Get all the headers
file(GLOB_RECURSE HEADER_FILES include/*.h)

# Declare a cpp library
add_library(ed_core
  src/entity.cpp
  src/measurement.cpp
  src/world_model.cpp
  src/transform_cache.cpp
  src/convex_hull_2d.cpp
  src/convex_hull_calc.cpp
  src/plugin_container.cpp
  src/rendering.cpp

  # World model querying
  src/world_model/transform_crawler.cpp

  # Model loading
  src/models/load_model.cpp
  src/models/model_loader.cpp
  src/models/shape_loader.cpp
  src/models/xml_shape_parser.cpp

  # Logging
  src/logging.cpp
  src/error_context.cpp

  # Forward geolib2 logging to rosconsole
  src/rosconsole_bridge.cpp

  ${HEADER_FILES}
)
target_link_libraries(ed_core polypartition ${TinyXML2_LIBRARIES} ${PCL_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(ed_core ${catkin_EXPORTED_TARGETS})

add_library(ed_io
  src/serialization/serialization.cpp
  src/io/filesystem/read.cpp
  src/io/filesystem/write.cpp
  src/io/transport/probe.cpp
  src/io/transport/probe_ros.cpp
  src/io/transport/probe_client.cpp

  src/io/json_reader.cpp
)
target_link_libraries(ed_io ed_core)

add_library(ed_server src/server.cpp include/ed/server.h)
target_link_libraries(ed_server ed_core ed_io)

# ------------------------------------------------------------------------------------------------
#                                               SERVER
# ------------------------------------------------------------------------------------------------

# Create executable
add_executable(ed src/ed.cpp)
target_link_libraries(ed ed_core ed_io ed_server)

# ------------------------------------------------------------------------------------------------
#                                               PLUGINS
# ------------------------------------------------------------------------------------------------

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
add_library(ed_tf_publisher_plugin plugins/tf_publisher_plugin.cpp)
target_include_directories(ed_tf_publisher_plugin PRIVATE ${tf2_geometry_msgs_INCLUDE_DIRS})
target_link_libraries(ed_tf_publisher_plugin ${tf2_LIBRARIES} ${tf2_ros_LIBRARIES})

find_package(kdl_parser REQUIRED)
add_library(ed_robot_plugin plugins/robot_plugin.cpp)
target_include_directories(ed_tf_publisher_plugin PRIVATE ${kdl_parser_INCLUDE_DIRS})
target_link_libraries(ed_robot_plugin ${catkin_LIBRARIES} ${kdl_parser_LIBRARIES})

add_library(ed_sync_plugin plugins/sync_plugin.cpp)
target_link_libraries(ed_sync_plugin ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(ed_view_model tools/view_model.cpp)
target_link_libraries(ed_view_model ed_core)

add_executable(ed_heightmap_to_mesh tools/heightmap_to_mesh.cpp)
target_link_libraries(ed_heightmap_to_mesh ed_core)

add_executable(configure tools/configure.cpp)
target_link_libraries(configure ed_core)

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

add_executable(test_service_speed test/test_service_speed.cpp)
target_link_libraries(test_service_speed ed_core)

add_executable(show_gui test/show_gui.cpp)
target_link_libraries(show_gui ed_core ${OpenCV_LIBRARIES})

add_executable(ed_test_heightmap_triangulation
  test/test_heightmap_triangulation.cpp
)
target_link_libraries(ed_test_heightmap_triangulation ed_core ${OpenCV_LIBRARIES} ${geolib2_LIBRARIES} ${tue_config_LIBRARIES})

