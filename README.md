ED : Environment Descriptor [![Build Status](https://travis-ci.org/tue-robotics/ed.svg?branch=master)](https://travis-ci.org/tue-robotics/ed)
======

## Introduction

ED - Environment Description - is a 3D geometric, object-based world representation system for robots. At the moment different ED modules exist which enable robots to localize themselves, update positions of known objects based on recent sensor data, segment and store newly encountered objects and visualize all this through a web-based GUI.

### Why use ED?

* ED is **one re-usable environment description** that can be used for a multitude of needed functionalities. Instead of having different environment representations for localization, navigation, manipulation, interaction, etc, you now only need *one*. An improvement in this single, central world model will reflect in the performances of the separate robot capabilities.
* ED is an **object-based, geometrical world representation**. As opposed to occupancy grids, octomaps, high-density monolithic meshes and the like, ED allows the user to use object-based, semantic statements such as 'inspect the objects on top of the table' or to 'navigate to the couch', while the geometrical representation allows the robot to still avoid obstacles - known and unknown - that it encounters on its path.
* The world geometry and objects can easily be specified with a set of easily accessible **human-readable files, re-usable models, heightmaps, primitives**, etc.
* ED has a **plugin system**. In itself, ED is 'just' a data structure containing a world representation. Functionality is added through plugins, which run concurrently and query and update the world model in a thread-safe manner. This makes the system **easily extendable**.
* ED's default **localization** module consists of a fast particle filter implementation and sensor models which always take into account the most recent state of the world. This means that if the world representation improves while the robot is running, localization becomes better. The localization module is more efficient and accurate than the well-known [AMCL-module](http://wiki.ros.org/amcl) and *no* separate occupancy grid is needed.
* The *sensor_integration* module enables **object updates**: using data from a depth sensor such as the Kinect, positions of known objects, such as pieces of furniture, can be updated. This allows the robot to successfully interact with its environment, even if the world model specified by the user contains errors.
* The *sensor_integration* module also enables **object segmentation**: the geometrical knowledge about the world enables fast, reliable and efficient object segmentation. This enables the robot to "find all objects on top of the table" or "inside the cabinet".
* Though not yet released, ED has modules for **real-time, on-line generation of costmaps** that can be used directly by existing **navigation modules** such as [MoveBase](http://wiki.ros.org/move_base). Whenever the world model representation changes, the costmaps will instantly reflect the change.
* ED can be visualized using a **web-based GUI**. This enables users to monitor the state of the world using a large variety of systems, including PC's, smart phones and tables.

![](https://cdn.rawgit.com/tue-robotics/ed/master/docs/images/wm.svg)
*ED Overview*

## Installation

### ED

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

### ED localization

Requirements:
* ED (see above)
* A 2D Range Finder (http://wiki.ros.org/Sensors) which scans in a plane parallel to the floor
* A [TF](wiki.ros.org/tf) containing transforms from the robots' odometry frame to the laser range finder frame

Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src
    git clone https://github.com/tue-robotics/ed_localization.git

And compile

    cd <your_catkin_workspace>:
    catkin_make

### ED visualization:

Requirements:
* ED (see above)

You will also need the following system dependencies (ROS Indigo, Ubuntu 14.04):

    sudo apt-get install ros-indigo-urdf

Check out the following packages in your workspace:

    cd <your_catkin_workspace>/src
    git clone https://github.com/tue-robotics/ed_gui_server.git

And compile

    cd <your_catkin_workspace>:
    catkin_make

## Quick Start

Requirements:
* Installed ED, ED localization and ED visualization (see above)

ED strongly relies on plugins to integrate sensor data, estimate object positions, recognize objects, etc. Before you can start ED, you have to specify the location of these plugins using the environment variable ED_PLUGIN_PATH. For example, for a ROS Catkin workspace, the variable should be set to something like:

    export ED_PLUGIN_PATH=<your_catkin_workspace>/devel/lib

You can provide multiple paths by separating them using ':'.

You can then start ED by running:

    rosrun ed ed

This will start the ED server which holds the world data structure. However, no world data is loaded, so the ED instance is quite useless at the moment. The world data that should be loaded and the plugins that should be started can be specified in a *configuration file*. Then, it's simply a matter of running ED like this:

    rosrun ed ed CONFIG-FILE

The configuration file should be written in a [YAML-format](www.yaml.org) and mainly consists of two parts: a world specification, and a list of plugins. The first determines what the world looks like, the second what we want to do with the world representation. Let's leave out the plugin part for now. Create a file called 'my-ed-config.yaml' and put this in:

    world:
    - id: block1
      pose: { x: 2, y: 0, z: 0.5 }
      shape:
        box:
          size: { x: 0.5, y: 0.5, z: 1 }
    - id: block2
      pose: { x: 3, y: 2, z: 0.75 }
      shape:
        box:
          size: { x: 0.5, y: 0.5, z: 1.5 }

    plugins:
      - name: gui_server
        lib: libed_gui_server_plugin.so


Take some time to try to understand what the config file states. A world is specified consisting of two objects - we'll call these entities. Both entities have an ID, a position in the world, and a shape. The shape is described as being a box with a certain size. Furthermore, we specify that we want to run the gui_server plugin. This will allow us to visualize the world in RViz.

Now, run ED using the configuration file we created above:

    rosrun ed ed my-ed-config.yaml

You probably won't see a lot happening. But ED *is* running. To visualize the world we have described, we need to run another node. This is some sort of proxy which communicates over a low-bandwidth channel with the ED GUI server plugin and generates visualization markers, which are typically high-bandwidth. This is usefull: when running ED on the robot, make sure you run the proxy on a local machine. The local machine will communicate with the robot over a low-bandwidth topic, while RViz will 'listen' to the high-bandwidth visualization markers which are produced on the same local machine. To run the proxy:

    rosrun ed_gui_server ed_rviz_publisher

Now start RViz, and listen to the Marker topic '/ed/rviz'. You should see two blocks appearing: the blocks you specified in the configuration file.
