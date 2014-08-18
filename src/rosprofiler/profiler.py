# Copyright 2014 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
This module implements a ROS node which profiles the resource usage of each node
process being run on the machine, as well as the resources of the host itself.
This information is published on the topics /node_statistics and /host_statistics.

"""

import threading
import xmlrpclib

import rospy
import rosgraph
import rosnode

from ros_statistics_msgs.msg import NodeStatistics
from ros_statistics_msgs.msg import HostStatistics

from host_monitor import HostMonitor
from node_monitor import NodeMonitor


class Profiler(object):
    """ """
    def __init__(self, sample_rate=None, update_rate=None):
        self.sample_rate = sample_rate or rospy.Duration(0.1)
        self.update_rate = update_rate or rospy.Duration(2)
        rospy.init_node('profiler_%s' % rosgraph.network.get_host_name())
        self._master = rosgraph.Master('profiler_%s' % rosgraph.network.get_host_name())

        self._lock = threading.Lock()
        self._monitor_timer = None
        self._publisher_timer = None
        self._graphupdate_timer = None

        # Data Structure for collecting information about the host
        self._host_monitor = HostMonitor()

        self._node_publisher = rospy.Publisher('node_statistics', NodeStatistics, queue_size=10)
        self._host_publisher = rospy.Publisher('host_statistics', HostStatistics, queue_size=10)

        # Processes we are watching
        self._nodes = dict()

    def start(self):
        """ Starts the Profiler
        :raises: ROSInitException when /enable_statistics has not been set to True
        """
        # Make sure that /enable_statistics is set
        if not rospy.get_param('/enable_statistics', False):
            raise rospy.ROSInitException("Rosparam '/enable_statistics' has not been set to true. Aborting")

        if self._monitor_timer is not None:
            raise Exception("Monitor Timer already started!")
        if self._graphupdate_timer is not None:
            raise Exception("Graph Update Timer already started!")
        if self._publisher_timer is not None:
            raise Exception("Publisher Timer already started!")

        # Initialize process list
        self._update_node_list()
        # Make sure all nodes are starting out blank
        for node in self._nodes.values():
            node.reset()
        # Start Timers
        self._monitor_timer = rospy.Timer(self.sample_rate, self._collect_data)
        self._publisher_timer = rospy.Timer(self.update_rate, self._publish_data)
        self._graphupdate_timer = rospy.Timer(self.update_rate, self._update_node_list)

    def stop(self):
        timers = [self._monitor_timer, self._publisher_timer, self._graphupdate_timer]
        timers = [timer for timer in timers if timer is not None]
        for timer in timers:
            timer.shutdown()
        for timer in timers:
            timer.join()

    def _update_node_list(self, event=None):
        """ Contacts the master using xmlrpc to determine what processes to watch """
        nodenames = rosnode.get_nodes_by_machine(rosgraph.network.get_host_name())
        # Lock data structures while making changes
        with self._lock:
            # Remove Node monitors for processes that no longer exist
            for name in self._nodes.keys():
                if not self._nodes[name].is_running():
                    rospy.loginfo("Removing Monitor for '%s'" % name)
                    self._nodes.pop(name)
            # Add node monitors for nodes on this machine we are not already monitoring
            for name in nodenames:
                if name not in self._nodes:
                    rospy.loginfo("Adding Monitor for '%s'" % name)
                    try:
                        uri = self._master.lookupNode(name)
                        code, msg, pid = xmlrpclib.ServerProxy(uri).getPid('/NODEINFO')
                        node = NodeMonitor(name, uri, pid)
                    except rosgraph.masterapi.MasterError:
                        rospy.logerr("WARNING: MasterAPI Error trying to contact '%s', skipping" % name)
                        continue
                    except xmlrpclib.socket.error:
                        rospy.logerr("WANRING: XML RPC ERROR contacting '%s', skipping" % name)
                        continue
                    self._nodes[name] = node

    def _collect_data(self, event=None):
        """ Collects data about the host and nodes """
        with self._lock:
            # Collect data about the host
            self._host_monitor.update()
            # Collect data about the processes
            for node in self._nodes.values():
                if node.is_running():
                    node.update()

    def _publish_data(self, event=None):
        """ Publishes data about the host and processing running on it. """
        with self._lock:
            rospy.logdebug("Publish data")
            # Publish the Hosts Statistics
            self._host_publisher.publish(self._host_monitor.get_statistics())
            self._host_monitor.reset()

            # Publish Each Node's statistics
            for node in self._nodes.values():
                self._node_publisher.publish(node.get_statistics())
                node.reset()
