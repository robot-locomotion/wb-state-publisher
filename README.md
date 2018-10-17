The wb-state-publisher
==============================================

## <img align="center" height="20" src="https://i.imgur.com/vAYeCzC.png"/> Introduction

The wb_state_publisher is a ros controller that publishes the actual whole-body state of the robot. The whole-body state is described as ROS message (i.e. dwl_msgs/WholeBodyState) inside dwl_msgs. This message structure can be transformed to the dwl::WholeBodyState class which contains functions that allows us to get the robot state in the desired frame (e.g. world, base and horizontal). For using this ros control plugin, it is required to define a ros_control::JointStateInterface hardware interface.

You can also visualize the robot's whole-body state by using the dwl_rviz_plugin::WholeBodyStateDisplay, for more information see dwl-rviz-plugin:

have tools that allows us to create our custom plugin such as: points (dwl_rviz_plugin::PointVisual), lines (dwl_rviz_plugin::LineVisual), polygons (dwl_rviz_plugin::PolygonVisual), arrows (dwl_rviz_plugin::ArrowVisual), etc.

| [![](https://i.imgur.com/TUrgkzO.gif)](https://www.youtube.com/watch?v=MOhJRMqBWQk&feature=youtu.be) | [![](https://i.imgur.com/RKe3sNo.gif)](https://www.youtube.com/watch?v=KI9x1GZWRwE)

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Carlos Mastalli, carlos.mastalli@laas.fr<br />
With support from the Dynamic Legged Systems lab at Istituto Italiano di Tecnologia<br />**



## <img align="center" height="20" src="https://i.imgur.com/fjS3xIe.png"/> Dependencies

The algorithms are built primarily in C++. The library uses a number of the local dependencies, which some of them are optionals.

The wb-state-publisher is a ROS packages with the following required dependencies:
* [ros-control](https://github.com/ros-controls/ros_control)
* [dwl](https://github.com/robot-locomotion/dwl)
* [dwl-msgs](https://github.com/robot-locomotion/dwl-msgs)
* [ros-control](http://wiki.ros.org/ros_control)

The following dependencies are optional:
* [Doxygen](http://www.doxygen.org)



## <img align="center" height="20" src="https://i.imgur.com/x1morBF.png"/> Building

Before building the wb_state_publisher, you need to install ros-control and dwl's dependencies. Additionally you have to build dwl and dwl_msgs with catkin.

The wb_state_publisher is a catkin project which can be built as:

	cd your_ros_ws/
	catkin_make

