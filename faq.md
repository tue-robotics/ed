# Frequently asked questions

## How is the data in ED represented?

The figure below illustrates "how ED works". ED on itself is a container for a database and several plugins that can interact with the database and/or the outside world.

![](https://cdn.rawgit.com/tue-robotics/ed/master/docs/images/wm.svg)
*ED Overview*

The database of ED contains of entities with various properties such as pose, velocity, shape and color. Users can add properties to entities with use of the query interface (ROS Service calls) or by writing their own plugins that modify the data. ED can be configured with use of a YAML file that describes the initial world state, this YAML file can also be altered by the user to add inititial properties to the entities in the scene. The first tutorial (https://github.com/tue-robotics/ed_tutorials/tree/master/tutorial01) of the ed_tutorials (https://github.com/tue-robotics/ed_tutorials) package illustrates how to initially configure ED.

## How can I configure ED

ED can be configured with use of a YAML file. This YAML file describes that plugins to load and with what initial world state, the database should be initialized. The first tutorial (https://github.com/tue-robotics/ed_tutorials/tree/master/tutorial01) of the ed_tutorials (https://github.com/tue-robotics/ed_tutorials) package illustrates how to configure ED.

## What are the data structures used in the interfaces, e.g. maps, semantic maps, etc.

The plugins living withing ED can expose interfaces to the outside world. The default ED packages use ROS (http://ros.org) to expose various interfaces to other nodes in the ROS network with use of services and topics, e.g.

### ed (https://github.com/tue-robotics/ed)
- Topic: Transform from entities to map (geometry_msgs/TransformStamped)
- Service: Reset service to reset the world state to the initial state (ed/Reset)
- Service: Query service to get data from ED and/or modify ED (ed/Query)

### ed_navigation (https://github.com/tue-robotics/ed_navigation)
- Topic: Occupancy grid of ed objects (nav_msgs/OccupancyGrid)
- Topic: Pointcloud2 of depth sensor (sensor_msgs/PointCloud2)

### ed_localization (https://github.com/tue-robotics/ed_localization)
- Topic: Particles of estimated robot pose (geometry_msgs/PoseArray)
- Topic: Transform from map to base_link (geometry_msgs/TransformStamped)

### ed_gui_server (https://github.com/tue-robotics/ed_gui_server)
- Topic: Entity states (ed_gui_server/EntityInfos)
- Service: Mesh query service (ed_gui_server/QueryMeshes)

### ed_sensor_integration (https://github.com/tue-robotics/ed_sensor_integration)
- Service: Segment objects with use of the kinect (ed_sensor_integration/Update)

### ed_perception (https://github.com/tue-robotics/ed_perception)
- Service: Classify segmented objects (ed_perception/Classify)

## What is a plugin and what can I do with it?

A plugin is a class derived from ed::Plugin (https://github.com/tue-robotics/ed/blob/master/include/ed/plugin.h). Plugins are used to add functionality to ED. A plugin contains an update hook that gets a const pointer of the current world state and an update request reference to add requests for updating the database. With use of plugins, people can interact with the world model in order to expose and/or update the information of the database.

