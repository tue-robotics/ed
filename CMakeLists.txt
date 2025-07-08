cmake_minimum_required(VERSION 3.5)
project(ed)

add_compile_options(-Wall -Werror=all)
add_compile_options(-Wextra -Werror=extra)

# Find before adding cmake_modules, to not use their deprecated FindEigen
find_package(PCL REQUIRED COMPONENTS common)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  code_profiler
  diagnostic_updater
  ${PROJECT_NAME}_msgs
  geolib2
  pluginlib
  rgbd
  rgbd_msgs
  rosconsole_bridge
  roscpp
  tf2_ros
  tue_config
  tue_filesystem
  tue_serialization
)

find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(SDFormat REQUIRED)
find_package(TinyXML2 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_core ${PROJECT_NAME}_io ${PROJECT_NAME}_server ${PROJECT_NAME}_hello_world_plugin ${PROJECT_NAME}_custom_properties_plugin
  CATKIN_DEPENDS code_profiler ${PROJECT_NAME}_msgs geolib2 pluginlib rgbd rgbd_msgs roscpp tf2_ros tue_config tue_filesystem tue_serialization
  DEPENDS OpenCV PCL
)

###########
## Build ##
###########

include_directories(
  include
  SYSTEM
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
add_library(${PROJECT_NAME}_core
  ${HEADER_FILES}

  src/convex_hull_2d.cpp
  src/convex_hull_calc.cpp
  src/entity.cpp
  src/measurement.cpp
  src/plugin_container.cpp
  src/rendering.cpp
  src/transform_cache.cpp
  src/world_model.cpp

  # World model querying
  src/world_model/transform_crawler.cpp

  # Model loading
  src/models/load_model.cpp
  src/models/model_loader.cpp
  src/models/shape_loader.cpp
  src/models/xml_shape_parser.cpp

  # Logging
  src/error_context.cpp
  src/logging.cpp

  # Forward geolib2/tue_filesystem logging to rosconsole
  src/rosconsole_bridge.cpp
)
target_link_libraries(${PROJECT_NAME}_core polypartition ${TinyXML2_LIBRARIES} ${PCL_LIBRARIES} ${SDFormat_LIBRARIES} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_core ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME}_io
  src/io/filesystem/read.cpp
  src/io/filesystem/write.cpp

  src/io/json_reader.cpp

  src/io/transport/probe.cpp
  src/io/transport/probe_client.cpp
  src/io/transport/probe_ros.cpp

  src/serialization/serialization.cpp
)
target_link_libraries(${PROJECT_NAME}_io ${PROJECT_NAME}_core)

add_library(${PROJECT_NAME}_server
  include/${PROJECT_NAME}/server.h
  src/server.cpp
)
target_link_libraries(${PROJECT_NAME}_server ${PROJECT_NAME}_core ${PROJECT_NAME}_io)

# ------------------------------------------------------------------------------------------------
#                                               SERVER
# ------------------------------------------------------------------------------------------------

# Create executable
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_core ${PROJECT_NAME}_io ${PROJECT_NAME}_server)

# ------------------------------------------------------------------------------------------------
#                                               PLUGINS
# ------------------------------------------------------------------------------------------------

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_library(${PROJECT_NAME}_tf_publisher_plugin plugins/tf_publisher_plugin.cpp)
target_include_directories(${PROJECT_NAME}_tf_publisher_plugin BEFORE PRIVATE SYSTEM ${tf2_geometry_msgs_INCLUDE_DIRS} ${tf2_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_tf_publisher_plugin ${tf2_geometry_msgs_LIBRARIES} ${tf2_LIBRARIES} ${catkin_LIBRARIES})

find_package(kdl_parser REQUIRED)
add_library(${PROJECT_NAME}_robot_plugin plugins/robot_plugin.cpp)
target_include_directories(${PROJECT_NAME}_robot_plugin BEFORE PRIVATE SYSTEM ${kdl_parser_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_robot_plugin ${kdl_parser_LIBRARIES} ${catkin_LIBRARIES})

add_library(${PROJECT_NAME}_sync_plugin plugins/sync_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_sync_plugin ${catkin_LIBRARIES})

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}_view_model tools/view_model.cpp)
target_link_libraries(${PROJECT_NAME}_view_model ${PROJECT_NAME}_core)

add_executable(${PROJECT_NAME}_heightmap_to_mesh tools/heightmap_to_mesh.cpp)
target_link_libraries(${PROJECT_NAME}_heightmap_to_mesh ${PROJECT_NAME}_core)

add_executable(configure tools/configure.cpp)
target_link_libraries(configure ${PROJECT_NAME}_core)

#add_executable(${PROJECT_NAME}_repl tools/repl.cpp)
#target_link_libraries(${PROJECT_NAME}_repl readline)

# ------------------------------------------------------------------------------------------------
#                                               EXAMPLES
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_hello_world_plugin examples/hello_world/hello_world_plugin.cpp)
add_library(${PROJECT_NAME}_custom_properties_plugin examples/custom_properties/custom_properties_plugin.cpp)

# ------------------------------------------------------------------------------------------------
#                                                TESTS
# ------------------------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}_test_wm test/test_wm.cpp)
target_link_libraries(${PROJECT_NAME}_test_wm ${PROJECT_NAME}_core ${OpenCV_LIBRARIES})

add_executable(test_mask test/test_mask.cpp)
target_link_libraries(test_mask ${PROJECT_NAME}_core ${OpenCV_LIBRARIES})

add_executable(test_service_speed test/test_service_speed.cpp)
target_link_libraries(test_service_speed ${PROJECT_NAME}_core)

add_executable(show_gui test/show_gui.cpp)
target_link_libraries(show_gui ${PROJECT_NAME}_core ${OpenCV_LIBRARIES})

add_executable(${PROJECT_NAME}_test_heightmap_triangulation
  test/test_heightmap_triangulation.cpp
)
target_link_libraries(${PROJECT_NAME}_test_heightmap_triangulation ${PROJECT_NAME}_core ${OpenCV_LIBRARIES} ${geolib2_LIBRARIES} ${tue_config_LIBRARIES})


# ------------------------------------------------------------------------------------------------
#                                               INSTALL
# ------------------------------------------------------------------------------------------------

catkin_install_python(
  PROGRAMS
    tools/entity-teleop
    tools/list_plugins
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  FILES plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_io
    ${PROJECT_NAME}_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(
  TARGETS
    configure
    ${PROJECT_NAME}
    ${PROJECT_NAME}_custom_properties_plugin
    ${PROJECT_NAME}_heightmap_to_mesh
    ${PROJECT_NAME}_hello_world_plugin
    ${PROJECT_NAME}_test_heightmap_triangulation
    ${PROJECT_NAME}_test_wm
    ${PROJECT_NAME}_view_model
    polypartition
    show_gui
    test_mask
    test_service_speed
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# ------------------------------------------------------------------------------------------------
#                                                 CI
# ------------------------------------------------------------------------------------------------

if (CATKIN_ENABLE_TESTING)
  find_package(catkin_lint_cmake REQUIRED)
  catkin_add_catkin_lint_test("-W2 --ignore LITERAL_PROJECT_NAME --ignore UNSORTED_LIST")
endif()
