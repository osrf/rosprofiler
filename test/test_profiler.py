#!/usr/bin/env python
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
import sys
import unittest
import time
import rospy
import rostest

from ros_statistics_msgs.msg import NodeStatistics
from std_msgs.msg import String

# TODO:
# Test adding a node - see number of unique nodesstat msgs increase, check names
# Test node going away - see number of nuique nodestat msgs decrease, check names

PKG = 'rosprofiler'
NAME = 'test_profiler'

# These are the nodes we expect to receive information about
EXPECTED_NODES = ['/talker', '/listener', '/rosprofiler_test', '/rosout', '/test_profiler']

class TestProfiler(unittest.TestCase):
    def __init__(self, *args):
        super(TestProfiler, self).__init__(*args)
        # Start time - for calculating timeout
        self.start_time = None
        # Node statistics
        self.nodes = dict()
        # Message Counters
        self.chatter_msgs = 0
        self.node_statistics_msgs = 0

    def setUp(self):
        rospy.init_node(NAME, anonymous=True)
        rospy.Subscriber('/chatter', String, self.chatter_callback)   
        rospy.Subscriber('/node_statistics', NodeStatistics, self.node_statistics_callback)   
        rospy.sleep(1.0)
        self.start_time = rospy.get_rostime()

    def node_statistics_callback(self, data):
        self.node_statistics_msgs += 1
        self.nodes[data.node] = data

    def chatter_callback(self, data):
        self.chatter_msgs += 1

    def timeout(self, duration):
        """ Returns True if the time since setup is greater then the duration, else False.
        duration - time in seconds
        """
        return rospy.get_rostime() > (self.start_time + rospy.Duration(duration))

    def wait_for_data(self):
        """ Waits to receive statistics data """
        while not rospy.is_shutdown() and not self.timeout(10.0):
            if len(self.nodes.keys()) >= len(EXPECTED_NODES):
                break
            rospy.sleep(1.0)
        return

    def test_nodes_reported(self):
        """ Tests to make sure that we received statistics about the correct processes """
        self.wait_for_data()
        rospy.loginfo("Received %d messages on /node_statistics"%self.node_statistics_msgs)
        assert set(EXPECTED_NODES) == set(self.nodes.keys()), "Expected:%s Received:%s"%(EXPECTED_NODES,self.nodes.keys())

    def test_statistics_pids(self):
        """ Make sure each node has a unique PID """
        self.wait_for_data()
        pids = [node.pid for node in self.nodes.values()]
        assert len(pids) > 0, "No PIDS to evaluate"
        assert len(pids) == len(set(pids)), "Non-unique PID(s) in list %s"%pids


    def test_chatter(self):
        """ Test that chatter is talking """
        # Reset the msg counter and then wait for 1 second. You should have received something.
        received_msgs = self.chatter_msgs
        rospy.loginfo("Received %d messages on /chatter"%received_msgs)
        assert(received_msgs > 0)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestProfiler, sys.argv)

