# Extra find_package() calls needed by anyone consuming ed's exported targets.
#
# ed::ed_core carries Boost::thread on its PUBLIC link interface, because
# ed/server.h, ed/plugin_container.h and ed/loop_usage_status.h use boost::thread
# and boost::mutex. ament_export_dependencies(Boost) can only emit a componentless
# find_package(Boost), which does not define Boost::thread, so CMake fails at
# generate time with "the link interface of target ed::ed_core contains
# Boost::thread but the target was not found". Request the component here.
find_package(Boost REQUIRED COMPONENTS thread)
