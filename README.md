ED : Environment Descriptor [![Build Status](https://travis-ci.org/tue-robotics/ed.svg?branch=master)](https://travis-ci.org/tue-robotics/ed)
======

## Introduction

ED - Environment Description - is a 3D geometric, object-based world representation system for robots. At the moment different ED modules exist which enable robots to localize themselves, update positions of known objects based on recent sensor data, segment and store newly encountered objects and visualize all this through a web-based GUI.

### Tutorials

All ED tutorials can be found in the ed_tutorials package: https://github.com/tue-robotics/ed_tutorials

### Why use ED?

* ED is **one re-usable environment description** that can be used for a multitude of needed functionalities. Instead of having different environment representations for localization, navigation, manipulation, interaction, etc, you now only need *one*. An improvement in this single, central world model will reflect in the performances of the separate robot capabilities.
* ED is an **object-based, geometrical world representation**. As opposed to occupancy grids, octomaps, high-density monolithic meshes and the like, ED allows the user to use object-based, semantic statements such as 'inspect the objects on top of the table' or to 'navigate to the couch', while the geometrical representation allows the robot to still avoid obstacles - known and unknown - that it encounters on its path.
* The world geometry and objects can easily be specified with a set of easily accessible **human-readable files, re-usable models, heightmaps, primitives**, etc.
* ED has a **plugin system**. In itself, ED is 'just' a data structure containing a world representation. Functionality is added through plugins, which run concurrently and query and update the world model in a thread-safe manner. This makes the system **easily extendable**.
* ED's default **localization** module consists of a fast particle filter implementation and sensor models which always take into account the most recent state of the world. This means that if the world representation improves while the robot is running, localization becomes better. The localization module is more efficient and accurate than the well-known [AMCL-module](http://wiki.ros.org/amcl) and *no* separate occupancy grid is needed.
* The *sensor_integration* module enables **object updates**: using data from a depth sensor such as the Kinect, positions of known objects, such as pieces of furniture, can be updated. This allows the robot to successfully interact with its environment, even if the world model specified by the user contains errors.
* The *sensor_integration* module also enables **object segmentation**: the geometrical knowledge about the world enables fast, reliable and efficient object segmentation. This enables the robot to "find all objects on top of the table" or "inside the cabinet".
* Though not yet released, ED has modules for **real-time, on-line generation of costmaps** that can be used directly by existing **navigation modules** such as [MoveBase](http://wiki.ros.org/move_base). Whenever the world model representation changes, the costmaps will instantly reflect the change.
* ED can be visualized using a **web-based GUI**. This enables users to monitor the state of the world using a large variety of systems, including PC's, smart phones and tablets.

![](https://cdn.rawgit.com/tue-robotics/ed/master/docs/images/wm.svg)
*ED Overview*

## Installation

Requirements:
* Ubuntu (12.04 or newer)
* ROS (Hydro or newer)

We assume you have successfully installed ROS and set-up a Catkin workspace. Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src

    git clone https://github.com/tue-robotics/ed.git
    git clone https://github.com/tue-robotics/tue_filesystem
    git clone https://github.com/tue-robotics/geolib2
    git clone https://github.com/tue-robotics/code_profiler
    git clone https://github.com/tue-robotics/tue_config.git
    git clone https://github.com/tue-robotics/ed_object_models.git
    git clone https://github.com/tue-robotics/tue_serialization
    git clone https://github.com/tue-robotics/rgbd

You will also need the following system dependencies (ROS Indigo, Ubuntu 14.04):

    sudo apt-get install ros-indigo-geometry-msgs ros-indigo-pcl-ros ros-indigo-message-filters ros-indigo-image-geometry ros-indigo-kdl-parser ros-indigo-roslib ros-indigo-std-srvs libyaml-cpp-dev ros-indigo-cv-bridge ros-indigo-tf libassimp-dev ros-indigo-message-generation ros-indigo-roscpp ros-indigo-message-runtime ros-indigo-class-loader ros-indigo-pcl-conversions

This should be sufficient to successfully compile ED:

    cd <your_catkin_workspace>
    catkin_make

## ED Extensions

### Localization
A fast particle filter implementation and sensor models for robot localization which always take into account the most recent state of the world.
- https://github.com/tue-robotics/ed_localization.git

### Visualization
Extension for visualizing the current world model state. The GUI server exposes ROS API's that can be used by various clients. Example clients are the `ed_rviz_publisher` of the `ed_gui_server` package and the `ed_rviz_plugins/WorldModel`. display.
- https://github.com/tue-robotics/ed_gui_server.git
- https://github.com/tue-robotics/ed_rviz_plugins.git
    
### Sensor integration
Plugins for integrating sensor data into ED, e.g. Lidar, RGBD
- https://github.com/tue-robotics/ed_sensor_integration.git

### Navigation
Extension for publishing occupancy grids and calculating map navigation constraints.
- https://github.com/tue-robotics/ed_navigation.git

### Perception
Extension for classifying entities based on their attached RGBD measurements.
- https://github.com/tue-robotics/ed_perception.git
