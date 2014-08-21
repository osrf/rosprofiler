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

from ros_topology_msgs.msg import *

PKG = 'rosprofiler'
NAME = 'test_grapher'

# TODO: Check services

EXPECTED_NODES = dict()
talker1 = Node(name="/talker1")
talker1.publishes.append("/chatter")
talker1.publishes.append("/rosout")
talker1.connections.append(Connection(destination='/rosout',topic='/rosout',direction=2,transport="TCPROS"))
talker1.connections.append(Connection(destination='/listener1',topic='/chatter',direction=2,transport="TCPROS"))
talker1.connections.append(Connection(destination='/listener2',topic='/chatter',direction=2,transport="TCPROS"))
talker2 = Node(name="/talker2")
talker2.publishes.append("/chatter")
talker2.publishes.append("/rosout")
talker2.connections.append(Connection(destination='/rosout',topic='/rosout',direction=2,transport="TCPROS"))
talker2.connections.append(Connection(destination='/listener1',topic='/chatter',direction=2,transport="TCPROS"))
talker2.connections.append(Connection(destination='/listener2',topic='/chatter',direction=2,transport="TCPROS"))
listener1 = Node(name="/listener1")
listener1.publishes.append("/rosout")
listener1.subscribes.append("/chatter")
listener1.connections.append(Connection(destination='/rosout',topic='/rosout',direction=2,transport="TCPROS"))
listener1.connections.append(Connection(destination='/talker1',topic='/chatter',direction=1,transport="TCPROS"))
listener1.connections.append(Connection(destination='/talker2',topic='/chatter',direction=1,transport="TCPROS"))
listener2 = Node(name="/listener2")
listener2.publishes.append("/rosout")
listener2.subscribes.append("/chatter")
listener2.connections.append(Connection(destination='/rosout',topic='/rosout',direction=2,transport="TCPROS"))
listener2.connections.append(Connection(destination='/talker1',topic='/chatter',direction=1,transport="TCPROS"))
listener2.connections.append(Connection(destination='/talker2',topic='/chatter',direction=1,transport="TCPROS"))
rosout = Node(name="/rosout")
rosout.publishes.append("/rosout_agg")
rosout.subscribes.append("/rosout")
rosout.connections.append(Connection(destination='/talker1',topic='/rosout',direction=1,transport="TCPROS"))
rosout.connections.append(Connection(destination='/talker2',topic='/rosout',direction=1,transport="TCPROS"))
rosout.connections.append(Connection(destination='/listener1',topic='/rosout',direction=1,transport="TCPROS"))
rosout.connections.append(Connection(destination='/listener2',topic='/rosout',direction=1,transport="TCPROS"))
rosout.connections.append(Connection(destination='/test_grapher',topic='/rosout',direction=1,transport="TCPROS"))
rosout.connections.append(Connection(destination='/rosgrapher',topic='/rosout',direction=1,transport="TCPROS"))
grapher = Node(name="/rosgrapher")
grapher.publishes.append("/rosout")
grapher.publishes.append("/topology")
grapher.connections.append(Connection(destination='/rosout',topic='/rosout',direction=2,transport="TCPROS"))
grapher.connections.append(Connection(destination='/'+NAME,topic='/topology',direction=2,transport="TCPROS"))
tester = Node(name="/test_grapher")
tester.publishes.append("/rosout")
tester.subscribes.append("/topology")
tester.connections.append(Connection(destination='/rosout',topic='/rosout',direction=2,transport="TCPROS"))
tester.connections.append(Connection(destination='/rosgrapher',topic='/topology',direction=1,transport="TCPROS"))
EXPECTED_NODES['/talker1'] = talker1
EXPECTED_NODES['/talker2'] = talker2
EXPECTED_NODES['/listener1'] = listener1
EXPECTED_NODES['/listener2'] = listener2
EXPECTED_NODES['/rosout'] = rosout
EXPECTED_NODES['/rosgrapher'] = grapher
EXPECTED_NODES['/'+NAME] = tester

t_chatter = Topic(name="/chatter", type="std_msgs/String")
t_rosout = Topic(name="/rosout", type="rosgraph_msgs/Log")
t_rosout_agg = Topic(name="/rosout_agg", type="rosgraph_msgs/Log")
t_topology = Topic(name="/topology", type="ros_topology_msgs/Graph")
EXPECTED_TOPICS = [t_chatter, t_rosout, t_rosout_agg, t_topology]

class TestGrapher(unittest.TestCase):
    def __init__(self, *args):
        super(TestGrapher, self).__init__(*args)
        # Start time - for calculating timeout
        self.start_time = None
        self.graph = Graph()

    def setUp(self):
        rospy.init_node(NAME)
        rospy.Subscriber('/topology', Graph, self.callback)   
        self.wait_for_data(10.0)

    def callback(self, data):
        self.graph = data
       
    def wait_for_data(self, duration):
        """ Waits to receive statistics data """
        start_time = rospy.get_rostime()
        while not rospy.is_shutdown() and not (rospy.get_rostime() > (start_time + rospy.Duration(duration))):
            if len(self.graph.nodes) >= len(EXPECTED_NODES) and len(self.graph.topics) >= len(EXPECTED_TOPICS):
                return
            rospy.sleep(1.0)

    def test_nodes_publishers(self):
        for node in self.graph.nodes:
            assert node.name in EXPECTED_NODES, "%s not found!"%node.name
            testnode = EXPECTED_NODES[node.name]
            assert set(node.publishes) == set(testnode.publishes), "%s.publishes=%s, but should be %s"%(node.name,node.publishes,testnode.publishes)

    def test_nodes_subscribers(self):
        for node in self.graph.nodes:
            assert node.name in EXPECTED_NODES, "%s not found!"%node.name
            testnode = EXPECTED_NODES[node.name]
            assert set(node.subscribes) == set(testnode.subscribes), "%s.subscribes=%s, but should be %s"%(node.name,node.subscribes,testnode.subscribes)

    def test_nodes_connections_present(self):
        for node in self.graph.nodes:
            assert node.name in EXPECTED_NODES, "%s not found!"%node.name
            testnode = EXPECTED_NODES[node.name]
            for connection in node.connections:
                assert connection in testnode.connections, "Node %s has extra connection %s"%(node.name, connection)

    def test_nodes_connections_missing(self):
        for node in self.graph.nodes:
            assert node.name in EXPECTED_NODES, "%s not found!"%node.name
            testnode = EXPECTED_NODES[node.name]
            for connection in testnode.connections:
                assert connection in node.connections, "Node %s expected to find missing connection %s"%(node.name, connection)

    def test_nodes_present(self):
        for node in self.graph.nodes:
            assert node.name in EXPECTED_NODES.keys(), "Found extra node '%s'"%node.name

    def test_nodes_missing(self):
        for node_name in EXPECTED_NODES.keys():
            assert node_name in [n.name for n in self.graph.nodes], "Expected to find missing node '%s'"%node_name

    def test_topics_present(self):
        for topic in self.graph.topics:
            assert topic in EXPECTED_TOPICS, "Found extra topic '%s'"%topic

    def test_topics_missing(self):
        for topic in EXPECTED_TOPICS:
            assert topic in self.graph.topics, "Expected to find missing topic '%s'"%topic
 

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestGrapher, sys.argv)

