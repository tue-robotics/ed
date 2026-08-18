cmake_minimum_required(VERSION 3.8)
project(ed)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror=all)
  add_compile_options(-Wextra -Werror=extra)
endif()

find_package(ament_cmake REQUIRED)

find_package(ament_index_cpp REQUIRED)
find_package(code_profiler REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(ed_interfaces REQUIRED)
find_package(geolib2 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rgbd REQUIRED)
find_package(rgbd_interfaces REQUIRED)
find_package(rosconsole_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tue_config REQUIRED)
find_package(tue_serialization REQUIRED)
find_package(tue_serialization_interfaces REQUIRED)

# Non-ROS dependencies
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl REQUIRED)
# Find before adding cmake_modules, to not use their deprecated FindEigen
find_package(PCL REQUIRED COMPONENTS common)
# SDFormat's own config package is major-versioned and the major differs per ROS
# distro (sdformat14 on Jazzy, sdformat16 on Rolling), so it cannot be found by
# name. Requesting a version does not help either: its config-version file is
# SameMajorVersion, so both `find_package(sdformat 14)` and a version range are
# rejected by every distro shipping another major. What is portable is the
# unversioned `sdformat` config; enforce our own minimum against sdformat_VERSION.
# find_package(sdformat_vendor) must come first - on Jazzy it is what puts the
# unversioned config (an alias shim over sdformat14) on CMAKE_PREFIX_PATH.
find_package(sdformat_vendor REQUIRED)
find_package(sdformat REQUIRED)
if(sdformat_VERSION VERSION_LESS 14)
  message(FATAL_ERROR "ed requires SDFormat 14 or newer, found ${sdformat_VERSION}")
endif()
find_package(TinyXML2 REQUIRED)

# ------------------------------------------------------------------------------------------------
#                                          3RDPARTY
# ------------------------------------------------------------------------------------------------

# Exclude vendored 3rdparty source from compile_commands.json so clang-tidy does not lint it
set(CMAKE_EXPORT_COMPILE_COMMANDS OFF)
add_library(polypartition 3rdparty/polypartition/src/polypartition.cpp)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
target_include_directories(polypartition PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/polypartition/include>
  $<INSTALL_INTERFACE:include>
)

# ------------------------------------------------------------------------------------------------
#                                          ED CORE
# ------------------------------------------------------------------------------------------------

# Get all the headers
file(GLOB_RECURSE HEADER_FILES include/*.h)

add_library(${PROJECT_NAME}_core SHARED
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

  # Forward geolib2/tue_config (console_bridge) logging to ROS
  src/rosconsole_bridge.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/rapidjson/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(${PROJECT_NAME}_core SYSTEM PRIVATE
  ${PCL_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  polypartition
  Boost::thread
  code_profiler::code_profiler
  geolib2::geolib
  rgbd::rgbd
  rosconsole_bridge::rosconsole_bridge
  tinyxml2::tinyxml2
  tue_config::tue_config
  tue_serialization::tue_serialization
  ${PCL_LIBRARIES}
)
# Only src/models/model_loader.cpp includes <sdf/*.hh>; keep it out of the export.
target_link_libraries(${PROJECT_NAME}_core PRIVATE sdformat::sdformat)
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC
  diagnostic_updater
  pluginlib
  rclcpp
  tf2_ros
)

add_library(${PROJECT_NAME}_io SHARED
  src/io/filesystem/read.cpp
  src/io/filesystem/write.cpp

  src/io/json_reader.cpp

  src/io/transport/probe.cpp
  src/io/transport/probe_client.cpp
  src/io/transport/probe_ros.cpp

  src/serialization/serialization.cpp
)
target_link_libraries(${PROJECT_NAME}_io
  ${PROJECT_NAME}_core
  tue_serialization::tue_serialization
  ${tue_serialization_interfaces_TARGETS}
  ${ed_interfaces_TARGETS}
)

add_library(${PROJECT_NAME}_server SHARED
  include/${PROJECT_NAME}/server.h
  src/server.cpp
)
target_link_libraries(${PROJECT_NAME}_server PUBLIC
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_io
  ${ed_interfaces_TARGETS}
)
ament_target_dependencies(${PROJECT_NAME}_server PUBLIC std_msgs)

# ------------------------------------------------------------------------------------------------
#                                               SERVER
# ------------------------------------------------------------------------------------------------

add_executable(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_core
  ${PROJECT_NAME}_io
  ${PROJECT_NAME}_server
  ${ed_interfaces_TARGETS}
)

# ------------------------------------------------------------------------------------------------
#                                               PLUGINS
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_tf_publisher_plugin SHARED plugins/tf_publisher_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_tf_publisher_plugin PUBLIC ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_tf_publisher_plugin PUBLIC tf2 tf2_geometry_msgs tf2_ros)

add_library(${PROJECT_NAME}_robot_plugin SHARED plugins/robot_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_robot_plugin PUBLIC ${PROJECT_NAME}_core)
ament_target_dependencies(${PROJECT_NAME}_robot_plugin PUBLIC kdl_parser sensor_msgs urdf)

add_library(${PROJECT_NAME}_sync_plugin SHARED plugins/sync_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_sync_plugin ${PROJECT_NAME}_core ${ed_interfaces_TARGETS})

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

# ------------------------------------------------------------------------------------------------
#                                                TOOLS
# ------------------------------------------------------------------------------------------------

add_executable(${PROJECT_NAME}_view_model tools/view_model.cpp)
target_link_libraries(${PROJECT_NAME}_view_model ${PROJECT_NAME}_core)

add_executable(${PROJECT_NAME}_heightmap_to_mesh tools/heightmap_to_mesh.cpp)
target_link_libraries(${PROJECT_NAME}_heightmap_to_mesh ${PROJECT_NAME}_core)

add_executable(configure tools/configure.cpp)
target_link_libraries(configure ${PROJECT_NAME}_core ${ed_interfaces_TARGETS})

# ------------------------------------------------------------------------------------------------
#                                               EXAMPLES
# ------------------------------------------------------------------------------------------------

add_library(${PROJECT_NAME}_hello_world_plugin SHARED examples/hello_world/hello_world_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_hello_world_plugin ${PROJECT_NAME}_core)

add_library(${PROJECT_NAME}_custom_properties_plugin SHARED examples/custom_properties/custom_properties_plugin.cpp)
target_link_libraries(${PROJECT_NAME}_custom_properties_plugin ${PROJECT_NAME}_core)

# ------------------------------------------------------------------------------------------------
#                                               INSTALL
# ------------------------------------------------------------------------------------------------

install(
  PROGRAMS
    tools/entity-teleop
    tools/list_plugins
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

install(
  TARGETS
    polypartition
    ${PROJECT_NAME}_core
    ${PROJECT_NAME}_io
    ${PROJECT_NAME}_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_tf_publisher_plugin
    ${PROJECT_NAME}_robot_plugin
    ${PROJECT_NAME}_sync_plugin
    ${PROJECT_NAME}_hello_world_plugin
    ${PROJECT_NAME}_custom_properties_plugin
    ${PROJECT_NAME}_view_model
    ${PROJECT_NAME}_heightmap_to_mesh
    configure
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  ament_index_cpp
  code_profiler
  diagnostic_updater
  ed_interfaces
  geolib2
  pluginlib
  rclcpp
  rgbd
  rosconsole_bridge
  std_msgs
  tf2_ros
  tue_config
  tue_serialization
  Boost
  OpenCV
  PCL
)

# ------------------------------------------------------------------------------------------------
#                                                TESTS
# ------------------------------------------------------------------------------------------------

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(tue_lint_config REQUIRED)

  add_executable(${PROJECT_NAME}_test_wm test/test_wm.cpp)
  target_link_libraries(${PROJECT_NAME}_test_wm ${PROJECT_NAME}_core ${OpenCV_LIBRARIES})

  add_executable(test_mask test/test_mask.cpp)
  target_link_libraries(test_mask ${PROJECT_NAME}_core ${OpenCV_LIBRARIES})

  add_executable(test_service_speed test/test_service_speed.cpp)
  target_link_libraries(test_service_speed ${PROJECT_NAME}_core ${ed_interfaces_TARGETS})

  add_executable(show_gui test/show_gui.cpp)
  target_link_libraries(show_gui
    ${PROJECT_NAME}_core ${OpenCV_LIBRARIES} ${ed_interfaces_TARGETS} ${tue_serialization_interfaces_TARGETS})

  add_executable(${PROJECT_NAME}_test_heightmap_triangulation test/test_heightmap_triangulation.cpp)
  target_link_libraries(${PROJECT_NAME}_test_heightmap_triangulation
    ${PROJECT_NAME}_core ${OpenCV_LIBRARIES} geolib2::geolib tue_config::tue_config)

  ament_clang_format(CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-format --clang-format-version=21)
  # ed has many translation units; run clang-tidy in parallel and allow extra time so the test
  # does not hit the default 300s timeout (which manifests as a "did not generate a result file" error).
  cmake_host_system_information(RESULT CLANG_TIDY_JOBS QUERY NUMBER_OF_LOGICAL_CORES)
  if(NOT CLANG_TIDY_JOBS OR CLANG_TIDY_JOBS EQUAL 0)
    set(CLANG_TIDY_JOBS 1)
  endif()
  ament_clang_tidy(CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-tidy --clang-tidy-version=21
    JOBS ${CLANG_TIDY_JOBS} TIMEOUT 600
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR}/compile_commands.json)
  ament_lint_cmake(MAX_LINE_LENGTH 120)
  ament_xmllint(MAX_LINE_LENGTH 120)
endif()

ament_package()
