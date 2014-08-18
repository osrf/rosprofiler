The ``rosprofiler`` package
===========================

This package provides tools for publishing information about the state of a ROS system as ROS topics.
There are currently two tools included, `rosprofiler`_ and `rosgrapher`_ .

rosprofiler
-----------

The ``rosprofiler`` node is designed to be run once on each machine running node processes in a ROS system.
It publishes topics ``ros_statistics_msgs/NodeStatistics`` on /node_statistics and ``ros_statistics_msgs/HostStatistics`` on /host_statistics.

The /enable_statistics rosparam must be set to true for this node to run.

