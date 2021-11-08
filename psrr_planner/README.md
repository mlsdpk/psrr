# psrr_planner

## Overview (TODO)

## Usage (TODO)

## Published Topics

- ~\<name>\/psrr_planner/path_se2 (nav_msgs/Path)  
    The SE2 solution path of the self-reconfigurable robot.

- ~\<name>\/psrr_planner/path_nd (psrr_msgs/Path)  
    The n-dimensional geometric solution path of the self-reconfigurable robot. See [psrr_msgs/Path](https://github.com/roarLab/psrr/blob/master/psrr_msgs/msg/Path.msg).

- ~\<name>\/psrr_planner/path_footprints (jsk_recognition_msgs/PolygonArray)  
    The 2D polygonal solution path footprints of the self-reconfigurable robot. Used primarily for visualization purposes. See [jsk_recognition_msgs/PolygonArray](https://github.com/jsk-ros-pkg/jsk_recognition/blob/master/jsk_recognition_msgs/msg/PolygonArray.msg)

## Subscribed Topics

- ~\<name>\/move_base_simple/goal (geometry_msgs/PoseStamped)  
    The desired goal pose to be set. Current version only supports SE2 representation goal.

## Parameters (TODO)