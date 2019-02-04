# RoboND-WhereAmI-Project-P6

Robot localization project.

## Abstract

Robot localization consists of determining where a mobile robot is located in its environment. Localization is one of the most fundamental competencies required by any autonomous robot and is an essential for making decisions.

Based on standard ROS packages ([AMCL](http://wiki.ros.org/amcl) and the [Navigation Stack](http://wiki.ros.org/navigation/Tutorials/RobotSetup)), the objective is to accurately localize a mobile robot inside a provided map in the [Gazebo](http://gazebosim.org/) and [RViz](http://wiki.ros.org/rviz) simulation environments. 

## Introduction

Robot localization is the process of determining where a mobile robot is located with respect to its environment. Localization is one of the most fundamental competencies required by an autonomous robot as the knowledge of the robot's own location is an essential precursor to making decisions about future actions.

This project explores the usage of standard ROS packages such as AMCL and Move Base to accurately localize and command a mobile robot inside a provided map in the Gazebo simulator.

Based on the specification, initially the `udacity_bot` was created using the SDF language definition; an XML file format used to describe all the elements in a simulation environment.

![udacity_bot in Rviz](./data/udacity_bot_rviz01.png)

The localization is solved using the AMCL package, while the navigation through the move base stack, where you can define a goal position for your robot in the map, and the robot will navigate to that goal position.

All the steps for an accuraty localization were followed and then a second bot called `ls_bot` was created changing it's sensors and exploring with a different robot structure.

![ls_bot in Rviz](./data/ls_bot_rviz01.png)

Each robot needed to be tuned separately.

## Background

Navigation is one of the most challenging competencies required of a mobile robot. Success in navigation requires success at the following:

 * `perception`: interpret robot's sensors to extract meaningful data
 * `localization`: determine its position within the environment
 * `cognition`: decide how to act to achieve its goals
 * `motion control`: the robot must modulate its actuators to achieve the desired goals.

Localization has received the greatest research attention during the past decades and is still being an active research area; significant advances have been made.

Integrating the usage of ROS packages Adaptive Monte Carlo Localization (AMCL) and the Navigation Stack, allowed the robots to navigate and localize in a particular environment or map.

### AMCL

The amcl package has a lot of parameters to select from. Different sets of parameters contribute to different aspects of the algorithm. Broadly speaking, they can be categorized into:

 * overall filter
 * laser
 * odometry
 
#### Overall Filter

 * `min_particles` and `max_particles` - As amcl dynamically adjusts its particles for every iteration, it expects a range of the number of particles as an input. A larger range, with a high maximum might be too computationally extensive for a low-end system.
 * `initial_pose` discarded on this project.
 * `update_min` as amcl relies on incoming laser scans. Upon receiving a scan, it checks the values for update_min_a and update_min_d and compares to how far the robot has moved. Based on this comparison it decides whether or not to perform a filter update or to discard the scan data. Discarding data could result in poorer localization results, and too many frequent filter updates for a fast moving robot could also cause computational problems.

#### Laser

There are two different types of models to consider under this - the `likelihood_field` and the `beam`. Each of these models defines how the laser rangefinder sensor estimates the obstacles in relation to the robot.

The `likelihood_field` model is usually more computationally efficient and reliable for an environment such as the one you are working with.

 * `laser_*_range`
 * `laser_max_beams`
 * `laser_z_hit` and `laser_z_rand`

Tuning of these parameters are experimental.

#### Odometry

 * `odom_model_type` tested `diff-corrected` and `diff`.
 * `odom_alphas` (1 through 4) define how much noise is expected from the robot's movements/motions as it navigates inside the map.

Important: The above set of parameters should help you get started, however they aren't the only ones that can improve your results.

### Navigation Stack

The move_base package help navigate the robot to the goal position by creating or calculating a path from the initial position to the goal, and the amcl package will localize the robot.

Move Base utilizes a costmap on top of the provided map. A costmap divides the map into a grid where a cell could represent free space or an obstacle. The `global costmap` is used to generate a long-term path for the robot, such as the path to the goal position from the robot's starting position.

The `local costmap` is used to generate a short-term path for the robot. For example, a path that attempts to align itself and the robot with the global path.

Parameters are centralizaed in the following files:

 * [local_costmap_params.yaml](ros/src/udacity_bot/config/local_costmap_params.yaml)
 * [global_costmap_params.yaml](ros/src/udacity_bot/config/global_costmap_params.yaml)
 * [costmap_common_params.yaml](ros/src/udacity_bot/config/costmap_common_params.yaml)
 * [base_local_planner_params.yaml](ros/src/udacity_bot/config/base_local_planner_params.yaml)

Modifications done

 * transform_tolerance
  
 * obstacle_range: 0.0
 * raytrace_range: to clear and update the free space in the costmap as the robot moves.
 * inflation_radius: determines the minimum distance between the robot geometry and the obstacles.

To execute the sample on udacity_bot

```sh
roslaunch udacity_bot udacity_world.launch
roslaunch udacity_bot amcl.launch
rosrun udacity_bot navigation_goal
```

To execute the sample on udacity_bot

```sh
roslaunch ls_bot sandbox_world.launch
roslaunch ls_bot amcl.launch
rosrun udacity_bot navigation_goal
```

## Results

ToDo


## Discussion

problem was about sim_time and controller_frequency to high, reduce it and now it is working fine

inflation radius: 0.2, pdist_scale: 2.5, gdist_scale: 1.0, occdist_scale: 0.01, sim_time: 2.0, obstacle range: 0.2, raytrace_range: 3.0, transform_tolerance: 0.5 and a controller_frequency of 0.5 in the amcl.launch file.

Used rqt_reconfigure tool to setup and see values dynamically.

reduced the local map size, tolerance, htz

increased the robot footprint slightly.

ToDo

## Conclusion / Future Work

ToDo

### Links:
 * [Initial repository](https://github.com/udacity/RoboND-Localization-Project)
 * [SDF format](http://sdformat.org/spec?ver=1.6&elem=sdf)
 * [Move Base](http://wiki.ros.org/move_base)
 * [costmap_2d](http://wiki.ros.org/costmap_2d)
 * [Navigation Stack Config](http://wiki.ros.org/navigation/Tutorials/RobotSetup)
 * [AMCL Parameters](http://wiki.ros.org/amcl#Parameters)
 * [This repository](https://github.com/ladrians/RoboND-WhereAmI-Project-P6)
 * [Project Rubric](https://review.udacity.com/#!/rubrics/1365/view)
 * [Mobile Robot Localization](http://www.cs.cmu.edu/~rasc/Download/AMRobots5.pdf)