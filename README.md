ED : Environment Descriptor [![Build Status](https://travis-ci.org/tue-robotics/ed.svg?branch=master)](https://travis-ci.org/tue-robotics/ed)
======

## Introduction

ED - Environment Description - is a 3D geometric, object-based world representation system for robots. At the moment different ED modules exist which enable robots to localize themselves, update positions of known objects based on recent sensor data, segment and store newly encountered objects and visualize all this through a web-based GUI.

### Why use ED?

* ED is **one re-usable environment description** that can be used for a multitude of needed functionalities. Instead of having different environment representations for localization, navigation, manipulation, interaction, etc, you now only need *one*. An improvement in this single, central world model will reflect in the performances of the separate robot capabilities.
* ED is an **object-based, geometrical world representation**. As opposed to occupancy grids, octomaps, high-density monolithic meshes and the like, ED allows the user to use object-based, semantic statements such as 'inspect the objects on top of the table' or to 'navigate to the couch', while the geometrical representation allows the robot to still avoid obstacles - known and unknown - that it encounters on its path.
* The world geometry and objects can easily be specified with a set of easily accessible **human-readable files, re-usable models, heightmaps, primitives**, etc.
* ED has a **plugin system**. In itself, ED is 'just' a data structure containing a world representation. Functionality is added through plugins, which run concurrently and query and update the world model in a thread-safe manner. This makes the system **easily extendable**.
* ED's default **localization** module consists of a fast particle filter implementation and sensor models which always take into account the most recent state of the world. This means that if the world representation improves while the robot is running, localization becomes better. The localization module is more efficient and reliable than the well-known AMCL-module while at the same time *no* separate occupancy grid is needed.
* The *sensor_integration* module enables **object updates**: using data from a depth sensor such as the Kinect, positions of known objects, such as pieces of furniture, can be updated. This allows the robot to successfully interact with its environment, even if the world model specified by the user contains errors.
* The *sensor_integration* module also enables **object segmentation**: the geometrical knowledge about the world enables fast, reliable and efficient object segmentation. This enables the robot to "find all objects on top of the table" or "inside the cabinet".
* Though not yet released, ED has modules for **real-time, on-line generation of costmaps** that can be used directly by existing **navigation modules** such as MoveBase. Whenever the world model representation changes, the costmaps will instantly reflect the change.
* ED can be visualized using a **web-based GUI**. This enables users to monitor the state of the world using a large variety of systems, including PC's, smart phones and tables.

![ED Overview](https://cdn.rawgit.com/tue-robotics/ed/master/docs/images/wm.svg)

## Installation

### Requirements

* Ubuntu (12.04 or newer)
* ROS (Hydro or newer)
* TF-graph: transforms must be available from the robots' odometry frame to the sensor frames

### Installation

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
    git clone https://github.com/tue-robotics/ed_gui_server.git
    
You will also need the following system dependencies:

    sudo apt-get install ros-hydro-visualization-msgs ros-hydro-geometry-msgs ros-hydro-pcl-ros ros-hydro-message-filters ros-hydro-image-geometry ros-hydro-kdl-parser libyaml-dev ros-hydro-roslib ros-hydro-navigation ros-hydro-std-srvs ros-hydro-cv-bridge ros-hydro-tf libassimp-dev ros-hydro-message-generation ros-hydro-roscpp ros-hydro-message-runtime ros-hydro-class-loader ros-hydro-pcl-conversions
    
This should be sufficient to successfully compile ED:

    cd <your_catkin_workspace>
    catkin_make
    
## Quick Start

ED strongly relies on plugins to integrate sensor data, estimate object positions, recognize objects, etc. Before you can start ED, you have to specify the location of these plugins using the environment variable ED_PLUGIN_PATH. For example, for a ROS Catkin workspace, the variable should be set to something like:

    export ED_PLUGIN_PATH=<your_catkin_workspace>/devel/lib
    
You can provide multiple paths by seperating them using ':'.

You can then start ED by running:

    rosrun ed ed_server
    
You can specify a configuration file for ED as a command line argument. Some example configuration files are in ed/config. 

If you are not running ED on the Amigo or Sergio robots but you have a standard kinect config, then:
    
    roslaunch ed standard_kinect.launch
