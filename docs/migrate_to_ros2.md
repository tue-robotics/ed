# Skill: Migrate a tue-robotics package from ROS 1 (catkin) to ROS 2 (ament / Jazzy)

> **Purpose:** Migrate `ed` — and any remaining ROS 1 package in the
> `tue-robotics` stack — from catkin/roscpp to ament_cmake/rclcpp on ROS 2
> (Humble / Jazzy / Rolling). Preserve behaviour. Make surgical, reviewable
> changes.
>
> **Reference templates.** Several packages are already migrated. Their old
> ROS 1 code lives on the `ros1` git branch and the ROS 2 code on `master`.
> When in doubt, diff them: `git -C <pkg> diff origin/ros1 master`.
>
> | Package | Use it as the model for… |
> | --- | --- |
> | `tue_config`, `geolib2` | C++ library, CMake/ament target export, console_bridge logging |
> | `ed_msgs` → `ed_interfaces` | message/service (`rosidl`) package, renaming, type changes |
> | `rgbd` | ROS **node** (roscpp→rclcpp), nodelet→component, tf2, launch files |
> | `tue_filesystem` | **deprecated** — replaced by `std::filesystem`, see §3 |

---

## 0. Migration order (dependencies first)

A package can only build once its dependencies are ROS 2. The dependency
order for `ed`:

1. Interface packages: `tue_serialization_interfaces`, `ed_interfaces`,
   `rgbd_interfaces` — **already migrated**.
2. Libraries: `tue_config`, `geolib2`, `tue_serialization`, `rgbd`,
   `code_profiler` — **already migrated**.
3. `tue_filesystem` — **do not migrate**; remove it from every package (§3).
4. `ed` itself — this package (§4–§9).

Confirm a dependency is migrated before relying on it:
`git -C <dep> rev-parse --abbrev-ref HEAD` is `master` **and**
`grep buildtool_depend <dep>/package.xml` shows `ament_cmake`.

---

## 1. `package.xml`

Apply these edits (see `git diff origin/ros1 master -- package.xml` in any
reference package for a worked example):

| ROS 1 | ROS 2 |
| --- | --- |
| `<buildtool_depend>catkin</buildtool_depend>` | `<buildtool_depend>ament_cmake</buildtool_depend>` |
| `<build_depend>cmake_modules</build_depend>` | **remove** (FindEigen etc. are not needed) |
| `<build_depend>message_generation</build_depend>` | `<build_depend>rosidl_default_generators</build_depend>` *(interface pkgs only)* |
| `<exec_depend>message_runtime</exec_depend>` | `<exec_depend>rosidl_default_runtime</exec_depend>` *(interface pkgs only)* |
| `<depend>roscpp</depend>` | `<depend>rclcpp</depend>` (+ `<depend>rclcpp_components</depend>` if it has nodes) |
| `<depend>roslib</depend>`, `<exec_depend>python3-rospkg</exec_depend>` | **remove** (ROS 1 only; use `ament_index_cpp` if package lookup is needed) |
| `<depend>rosconsole_bridge</depend>` | **remove** (see logging, §6) |
| `<depend>tf</depend>` | `<depend>tf2</depend>` / `<depend>tf2_ros</depend>` |
| `<depend>nodelet</depend>` / `<depend>pluginlib</depend>` (for nodelets) | `<depend>rclcpp_components</depend>` (pluginlib stays only for *your own* plugin systems, see §7) |
| `<depend>ed_msgs</depend>` | `<depend>ed_interfaces</depend>` |
| `<depend>rgbd_msgs</depend>` | `<depend>rgbd_interfaces</depend>` |
| `<depend>tue_serialization</depend>` (when used as **messages**) | `<depend>tue_serialization_interfaces</depend>` |
| `<depend>tue_filesystem</depend>` | **remove** (§3) |
| `<test_depend>catkin_lint_cmake</test_depend>` / `rosunit` / `rostest` | the ament linter set, below |

Add the ament linter/test deps and the build-type export (copy verbatim from
`tue_config`/`geolib2` `master`):

```xml
<test_depend>ament_cmake_clang_format</test_depend>
<test_depend>ament_cmake_clang_tidy</test_depend>
<test_depend>ament_cmake_gtest</test_depend>
<test_depend>ament_cmake_lint_cmake</test_depend>
<test_depend>ament_cmake_xmllint</test_depend>
<test_depend>clang-format-21</test_depend>
<test_depend>clang-tidy-21</test_depend>
<test_depend>tue_lint_config</test_depend>

<export>
  <build_type>ament_cmake</build_type>
  <!-- keep the rosdoc line; replace the ROS1 plugin export, see §7 -->
  <rosdoc config="rosdoc.yaml" />
</export>
```

Interface packages additionally need
`<member_of_group>rosidl_interface_packages</member_of_group>`.

---

## 2. `CMakeLists.txt`

### 2.1 Header (compiler flags + C++17)

```cmake
cmake_minimum_required(VERSION 3.8)   # was 3.5
project(ed)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON) # needed by ament_clang_tidy

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)          # required: enables std::filesystem (§3)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Werror=all)
  add_compile_options(-Wextra -Werror=extra)
endif()
```

### 2.2 Dependency discovery

Replace the single `find_package(catkin REQUIRED COMPONENTS ...)` with
`find_package(ament_cmake REQUIRED)` plus **one `find_package(<dep> REQUIRED)`
per dependency**. Delete the `catkin_package(...)` block entirely.

```cmake
# ROS 1
find_package(catkin REQUIRED COMPONENTS code_profiler geolib2 pluginlib rgbd
  rgbd_msgs roscpp tf2_ros tue_config tue_serialization ...)
catkin_package(INCLUDE_DIRS include LIBRARIES ... CATKIN_DEPENDS ... DEPENDS ...)

# ROS 2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(code_profiler REQUIRED)
find_package(geolib2 REQUIRED)
find_package(rgbd REQUIRED)
find_package(rgbd_interfaces REQUIRED)
find_package(ed_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tue_config REQUIRED)
find_package(tue_serialization REQUIRED)
find_package(diagnostic_updater REQUIRED)
# non-ROS deps stay as they were:
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(orocos_kdl REQUIRED)
find_package(SDFormat REQUIRED)
find_package(TinyXML2 REQUIRED)
# NOTE: no find_package(tue_filesystem) — removed (§3)
```

### 2.3 Includes and linking — go target-based

Drop the global `include_directories(... ${catkin_INCLUDE_DIRS})`. For each
library/executable, set includes on the target and link explicit, namespaced
targets (no more `${catkin_LIBRARIES}` / `add_dependencies(... ${catkin_EXPORTED_TARGETS})`).

```cmake
add_library(${PROJECT_NAME}_core SHARED ${HEADER_FILES} src/...)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(${PROJECT_NAME}_core SYSTEM PRIVATE
  3rdparty/polypartition/include
  3rdparty/rapidjson/include
  ${PCL_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_core
  polypartition
  geolib2::geolib2
  tue_config::tue_config
  tue_serialization::tue_serialization
  rgbd::rgbd
  console_bridge::console_bridge
  tinyxml2::tinyxml2
  ${PCL_LIBRARIES} ${SDFormat_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_core PUBLIC diagnostic_updater tf2_ros)
```

For **generated interfaces**, link the typesupport target, not a bare name:

```cmake
target_link_libraries(${PROJECT_NAME}_server
  ${PROJECT_NAME}_core ${PROJECT_NAME}_io
  rclcpp::rclcpp
  ${ed_interfaces_TARGETS})        # or ed_interfaces::ed_interfaces__rosidl_typesupport_cpp
```

> Check exactly which target name a dependency exports with
> `cmake --find-package` or by reading `rgbd/master` and `geolib2/master`
> `CMakeLists.txt`. `geolib2` links e.g.
> `geometry_msgs::geometry_msgs__rosidl_generator_cpp` and `tf2::tf2`.

### 2.4 Install + export

```cmake
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME})
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_io ${PROJECT_NAME}_server
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)

# executables and plugin .so's go under lib/${PROJECT_NAME}
install(TARGETS ${PROJECT_NAME} configure ${PROJECT_NAME}_view_model ...
  DESTINATION lib/${PROJECT_NAME})

# Python tools: was catkin_install_python(...)
install(PROGRAMS tools/entity-teleop tools/list_plugins
  DESTINATION lib/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(geolib2 tue_config tue_serialization rgbd rclcpp
  pluginlib tf2_ros diagnostic_updater ed_interfaces OpenCV PCL)
```

End the file with `ament_package()` (must be the **last** call).

### 2.5 Tests / linters

```cmake
# ROS 1: if (CATKIN_ENABLE_TESTING) ... catkin_add_gtest(...) / catkin_lint
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_clang_format REQUIRED)
  find_package(ament_cmake_clang_tidy REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(tue_lint_config REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_wm test/test_wm.cpp)
  target_link_libraries(${PROJECT_NAME}_test_wm ${PROJECT_NAME}_core ${OpenCV_LIBRARIES})

  ament_clang_format(CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-format --clang-format-version=21)
  ament_clang_tidy(CONFIG_FILE ${tue_lint_config_DIR}/../config/.clang-tidy --clang-tidy-version=21
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_BINARY_DIR}/compile_commands.json)
  ament_lint_cmake(MAX_LINE_LENGTH 120 "--filter=...")  # copy filter from geolib2/master
  ament_xmllint(MAX_LINE_LENGTH 120)
endif()
```

---

## 3. Remove `tue_filesystem` → `std::filesystem` (deprecated dependency)

`tue_filesystem` is obsolete. **Do not depend on it.** Replace every use with
C++17 `<filesystem>` following its own skill:
`tue_filesystem/doc/migrate_to_std_filesystem.md`.

In `ed`, only `tue::filesystem::Path` is used (no `Crawler`). Affected files:
`src/io/filesystem/read.cpp`, `src/io/filesystem/write.cpp`,
`src/models/load_model.cpp`, `src/models/model_loader.cpp`,
`src/models/shape_loader.cpp`, `src/server.cpp`, `tools/configure.cpp`,
`tools/view_model.cpp`.

Key substitutions (full table in that skill):

| `tue::filesystem::Path` | `std::filesystem` |
| --- | --- |
| `#include <tue/filesystem/path.h>` | `#include <filesystem>` |
| `tue::filesystem::Path` | `std::filesystem::path` |
| `p.exists()` | `std::filesystem::exists(p)` |
| `p.extension()` *(returns `std::string`)* | `p.extension().string()` |
| `p.filename()` *(returns `std::string`)* | `p.filename().string()` |
| `p.parentPath()` | `p.parent_path()` — **quirk:** returns empty, not `"."` |
| `Path(a) + "/" + b` | `std::filesystem::path(a) / b` |

Then remove `find_package(tue_filesystem ...)`, the `tue_filesystem` link/export
entries, and the `<depend>tue_filesystem</depend>` line. Verify clean:
`grep -rn "tue/filesystem\|tue::filesystem\|tue_filesystem" .`

---

## 4. C++ node API: roscpp → rclcpp

(Model: `rgbd/master`.) Header include style for **all** ROS message packages
changes from `<pkg/Type.h>` to `<pkg/msg/type.hpp>` / `<pkg/srv/type.hpp>`
(snake_case), and types gain `::msg::` / `::srv::`:

```cpp
// ROS 1                                  // ROS 2
#include <ed_msgs/Query.h>                #include <ed_interfaces/srv/query.hpp>
ed_msgs::Query::Request req;             ed_interfaces::srv::Query::Request req;
#include <std_msgs/String.h>             #include <std_msgs/msg/string.hpp>
#include <geometry_msgs/TransformStamped.h>  #include <geometry_msgs/msg/transform_stamped.hpp>
```

Core API mapping:

| ROS 1 (roscpp) | ROS 2 (rclcpp) |
| --- | --- |
| `#include <ros/ros.h>` | `#include <rclcpp/rclcpp.hpp>` |
| `ros::init(argc,argv,"ed")` | `rclcpp::init(argc,argv)` |
| `ros::NodeHandle nh; nh_private("~")` | `auto node = rclcpp::Node::make_shared("ed");` |
| `nh.advertise<T>(topic,q)` | `node->create_publisher<T>(topic,q)` |
| `pub.publish(m)` | `pub->publish(m)` |
| `pub.getNumSubscribers()` | `pub->get_subscription_count()` |
| `nh.subscribe(topic,q,cb)` | `node->create_subscription<T>(topic,q,cb)` |
| `nh.advertiseService(name,cb)` | `node->create_service<T>(name,cb)` (cb takes `Request::SharedPtr`, `Response::SharedPtr`, returns `void`) |
| `nh.serviceClient<T>(name)` | `node->create_client<T>(name)` |
| `nh.getParam("rate",r)` / `nh.param(...)` | `node->declare_parameter<double>("rate",30.0)` |
| `ros::ok()` | `rclcpp::ok()` |
| `ros::spinOnce()` / `ros::spin()` | `rclcpp::spin_some(node)` / `rclcpp::spin(node)` |
| `ros::CallbackQueue` | `rclcpp::CallbackGroup` + an executor (`SingleThreadedExecutor`) |
| `ros::package::getPath("x")` | `ament_index_cpp::get_package_share_directory("x")` |

Time / rates (used in `src/ed.cpp`, `src/plugin_container.cpp`,
`plugins/*.cpp`):

| ROS 1 | ROS 2 |
| --- | --- |
| `ros::Time::now()` | `node->now()` or `clock->now()` |
| `ros::Time` (in headers, no node) | `rclcpp::Time` |
| `ros::Duration(1)` | `rclcpp::Duration::from_seconds(1)` |
| `ros::Rate r(f); r.sleep()` | `rclcpp::Rate r(f); r.sleep()` |
| `stamp.fromSec(t)` / `stamp.toSec()` | `rclcpp::Time(int64_t(t*1e9))` / `rclcpp::Time(stamp).seconds()` |
| `ros::Time::init()` (tests) | not needed; create a `rclcpp::Clock` |

`tf2_ros::Buffer` now needs a clock: `tf2_ros::Buffer buffer(node->get_clock())`.
For header-location differences across distros, guard with `__has_include`
(see `geolib2/master` for tf2/image_geometry/cv_bridge examples):

```cpp
#if __has_include(<tf2/LinearMath/Transform.hpp>)
#include <tf2/LinearMath/Transform.hpp>
#else
#include <tf2/LinearMath/Transform.h>
#endif
```

---

## 5. Interface (message/service) dependencies

`ed` only **consumes** interfaces — it does not define them (those moved to
`ed_interfaces`). So no `rosidl` work here; just:

- depend on `ed_interfaces` / `rgbd_interfaces` /
  `tue_serialization_interfaces` (§1, §2.2),
- update includes and type names to `::msg::` / `::srv::` (§4),
- link the typesupport target (§2.3).

Note `ed_interfaces` already changed field types: `time` →
`builtin_interfaces/Time`, `duration` → `builtin_interfaces/Duration`,
`tue_serialization/Binary` → `tue_serialization_interfaces/Binary`. Any code
in `include/ed/helpers/msg_conversions.h` that builds these fields must use the
new types.

---

## 6. Logging

`ed` has its own logger (`include/ed/logging.h`, `src/logging.cpp`) — **keep
it**. The ROS-coupled pieces to change:

- Delete `src/rosconsole_bridge.cpp` and its source entry in `CMakeLists.txt`;
  remove the `rosconsole_bridge` dependency. geolib2/tue_filesystem now log via
  `console_bridge` directly, which surfaces without the bridge.
- Replace the few `ROS_*` macros (`ROS_ERROR_STREAM`, `ROS_WARN_NAMED` in
  `plugins/robot_plugin.cpp`, `plugins/sync_plugin.cpp`,
  `src/models/load_model.cpp`) with either the existing `ed::log::*` API or
  `RCLCPP_*(get_logger(), ...)` where a node/logger is in scope.
- Library code with no node uses `console_bridge`:
  `CONSOLE_BRIDGE_logError("...: %s", e.c_str())`.

---

## 7. Plugins (pluginlib) and components

`ed`'s **own** plugin system (`ed::Plugin` base, `ED_REGISTER_PLUGIN`,
`pluginlib::ClassLoader<ed::Plugin>` in `src/plugin_container.cpp`,
`plugins.xml`) stays on **pluginlib** — pluginlib is fully supported in ROS 2.
Changes:

- Include path: `#include <pluginlib/class_list_macros.h>` →
  `#include <pluginlib/class_list_macros.hpp>`. `PLUGINLIB_EXPORT_CLASS`
  (wrapped by `ED_REGISTER_PLUGIN`) is unchanged.
- `plugins.xml` content is unchanged (same `<library path="lib/libed_*">`
  scheme). Export it the ROS 2 way instead of via the `<export><ed .../>` tag:
  add to `CMakeLists.txt`
  `pluginlib_export_plugin_description_file(ed plugins.xml)` and keep the
  `install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})`.
- Build plugin libraries as `SHARED`.

Distinguish this from **nodelets**: if any tue package used `nodelet`
(`rgbd` did), that becomes an `rclcpp_components` component
(`RCLCPP_COMPONENTS_REGISTER_NODE` + `rclcpp_components_register_nodes(...)`).
`ed`'s plugins are *not* nodelets, so they stay pluginlib.

---

## 8. Launch and Python tools

- `*.launch` (XML) → `*.launch.py` (Python `LaunchDescription` / `Node`).
  `type=` becomes `executable=`; `<remap>` becomes `remappings=[(from,to)]`.
- Python tools (`tools/entity-teleop`, `tools/list_plugins`): port any
  `rospy`/`rospkg`/`roslib` use to `rclpy` / `ament_index_python`; install via
  `install(PROGRAMS ... DESTINATION lib/${PROJECT_NAME})`.

---

## 9. Boost → std (do alongside, low risk)

ROS 2 / C++17 lets you drop Boost for the standard library. `include/ed/types.h`
already typedefs the smart pointers, so most call sites are insulated. Replace:
`boost::shared_ptr`→`std::shared_ptr`, `boost::make_shared`→`std::make_shared`,
`boost::mutex`/`scoped_lock`/`unique_lock`/`lock_guard`→`std::` equivalents,
`boost::thread`→`std::thread`, `boost::this_thread`→`std::this_thread`. This is
optional for a first build but removes a dependency; keep it a separate commit.

---

## 10. CI

Update `.github/workflows/main.yml` to the ROS 2 matrix used by the reference
packages (copy from `geolib2/master`):

```yaml
on: [push, pull_request, workflow_dispatch]
permissions:
  contents: read
jobs:
  tue-ci:
    strategy:
      fail-fast: false
      matrix:
        ros_distro: [humble, jazzy, rolling-u24]
    steps:
      - uses: tue-robotics/tue-env/ci/main@master
        with:
          image: ghcr.io/tue-robotics/tue-env-ros-${{ matrix.ros_distro }}
          package: ${{ github.event.repository.name }}
```

---

## 11. Validation

```bash
colcon build --packages-up-to ed
colcon test --packages-select ed && colcon test-result --verbose
# no leftover ROS 1 / tue_filesystem references:
grep -rn "ros/ros.h\|ros::\|ROS_INFO\|ROS_WARN\|ROS_ERROR\|catkin\|tue_filesystem\|ed_msgs\|rgbd_msgs" \
  src include plugins tools test CMakeLists.txt package.xml
```

Acceptable remaining hits: comments/changelog you intentionally wrote.

---

## 12. Pitfalls checklist (review before submitting)

- [ ] `package.xml`: `ament_cmake`, no `catkin`/`message_generation`/`roslib`/
      `rosconsole_bridge`/`tue_filesystem`; `*_msgs`→`*_interfaces`;
      `<build_type>ament_cmake</build_type>` present.
- [ ] `CMakeLists.txt`: one `find_package` per dep; no `catkin_package`;
      target-based includes/links; `ament_export_targets` +
      `ament_export_dependencies`; `ament_package()` last.
- [ ] All ROS message includes converted to `<pkg/msg/type.hpp>` and types to
      `::msg::`/`::srv::`; interfaces linked via the typesupport target.
- [ ] roscpp→rclcpp complete; service callbacks return `void` and take
      `SharedPtr` args; `ros::Time`/`Duration`/`Rate`→`rclcpp::`.
- [ ] `tf2_ros::Buffer` constructed with a clock.
- [ ] `tue_filesystem` fully removed in favour of `std::filesystem`; C++17 set.
- [ ] `rosconsole_bridge.cpp` deleted; `ROS_*` macros replaced.
- [ ] pluginlib: `class_list_macros.hpp` include; plugin libs `SHARED`;
      `pluginlib_export_plugin_description_file(...)` added.
- [ ] Launch files ported to `.launch.py`; Python tools on rclpy.
- [ ] CI matrix updated; `colcon build`/`test` green on targeted distros.

---

## 13. Commit / PR

Stage the work as logical commits, mirroring the reference PRs (e.g.
ed_msgs#10 "Migrate to ROS 2"):

1. `Migrate from tue_filesystem to std::filesystem` (§3, isolated).
2. `Migrate ed to ROS 2 (ament_cmake + rclcpp)` (§1–§8).
3. `Replace Boost with std` (§9, optional).
4. `CI: ROS 2 distro matrix` (§10).
</content>
</invoke>
