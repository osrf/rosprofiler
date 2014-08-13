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
import psutil
import numpy as np
from threading import *
import multiprocessing # for multiprocessing.cpu_count()
import math
import time

import rospy
import rosgraph
import rosnode
import xmlrpclib
from ros_statistics_msgs.msg import *

class Profiler(object):
    """ """
    def __init__(self, sample_rate=None, update_rate=None):
        self.sample_rate = sample_rate or rospy.Duration(0.1)
        self.update_rate = update_rate or rospy.Duration(2)
        rospy.init_node('profiler_%s'%rosgraph.network.get_host_name())
        self._master = rosgraph.Master('profiler_%s'%rosgraph.network.get_host_name())

        self._lock = Lock()
        self._monitor_timer = None
        self._publisher_timer = None
        self._graphupdate_timer = None

        # Data Structure for collecting information about the host
        self._host_monitor = HostMonitor()

        self._node_publisher = rospy.Publisher('node_statistics', NodeStatistics, queue_size = 10)
        self._host_publisher = rospy.Publisher('host_statistics', HostStatistics, queue_size = 10)

        # Processes we are watching
        self._nodes = dict()

    def start(self):

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
                    rospy.loginfo("Removing Monitor for '%s'"%name)
                    self._nodes.pop(name)
            # Add node monitors for nodes on this machine we are not already monitoring
            for name in nodenames:
                if not name in self._nodes:
                    rospy.loginfo("Adding Monitor for '%s'"%name)
                    try:
                        uri = self._master.lookupNode(name)
                        code, msg, pid = xmlrpclib.ServerProxy(uri).getPid('/NODEINFO')
                        node = NodeMonitor(name, uri, pid)
                    except rosgraph.masterapi.MasterError:
                        rospy.logerr("WARNING: MasterAPI Error trying to contact '%s', skipping"%name)
                        continue
                    except xmlrpclib.socket.error:
                        rospy.logerr("WANRING: XML RPC ERROR contacting '%s', skipping"%name)
                        continue
                    self._nodes[name] = node

    def _collect_data(self, event=None):
        """ Collects data about the host and nodes """
        with self._lock:
            # Collect data about the host
            self._host_monitor.update()
            # Collect data about the processes
            start = time.time()
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
         
class HostMonitor(object):
    """ Tracks cpu and memory information of the host. """
    def __init__(self):
        self._hostname = rosgraph.network.get_host_name()
        self._ipaddress = rosgraph.network.get_local_address()
        self._cpus_available = multiprocessing.cpu_count()

        self.cpu_load_log = list()
        self.phymem_used_log = list()
        self.phymem_avail_log = list()
        self.start_time = rospy.get_rostime()

    def update(self):
        """ Record information about the cpu and memory usage for this host into a buffer """
        self.cpu_load_log.append(psutil.cpu_percent(interval=0,percpu=True))
        self.phymem_used_log.append(psutil.used_phymem())
        self.phymem_avail_log.append(psutil.avail_phymem())

    def get_statistics(self):
        """ Returns HostStatistics() using buffered information. """
        statistics = HostStatistics()
        statistics.hostname = self._hostname
        statistics.ipaddress = self._ipaddress
        statistics.window_start = self.start_time
        statistics.window_stop = rospy.get_rostime()
        statistics.samples = len(self.cpu_load_log)

        if len(self.cpu_load_log) > 0:
            cpu_load_log = np.array(self.cpu_load_log)
            cpu_load_log = cpu_load_log.transpose()
            for cpu in range(self._cpus_available):
                statistics.cpu_load_mean.append(np.mean(cpu_load_log[cpu]))
                statistics.cpu_load_std.append(np.std(cpu_load_log[cpu]))
                statistics.cpu_load_max.append(np.max(cpu_load_log[cpu]))
        if len(self.phymem_used_log) > 0:
            phymem_used_log = np.array(self.phymem_used_log)
            statistics.phymem_used_mean = np.mean(phymem_used_log)
            statistics.phymem_used_std = np.std(phymem_used_log)
            statistics.phymem_used_max = np.max(phymem_used_log)
        if len(self.phymem_avail_log) > 0:
            phymem_avail_log = np.array(self.phymem_avail_log)
            statistics.phymem_avail_mean = np.mean(phymem_avail_log)
            statistics.phymem_avail_std = np.std(phymem_avail_log)
            statistics.phymem_avail_max = np.max(phymem_avail_log)
        return statistics

    def reset(self):
        """ Clears the information buffer. """
        self.cpu_load_log = list()
        self.phymem_used_log = list()
        self.phymem_avail_log = list()
        self.start_time = rospy.get_rostime()


class NodeMonitor(object):
    """ Tracks process statistics of a PID. """
    def __init__(self,name, uri, pid):
        self.node = name
        self.hostname = rosgraph.network.get_host_name()
        self.uri = uri
        self.pid = pid
        self._process = psutil.Process(int(self.pid))
        self._process_ok = True # This gets set to false if the process dies

        self.cpu_log = list()
        self.virt_log = list()
        self.res_log = list()
        self.num_threads = 0
        self.start_time = rospy.get_rostime()

    def is_running(self):
        if self._process is None:
            return False
        return self._process.is_running()

    def update(self):
        """ Record cpu and memory information about this procress into a buffer """
        try:
            self.cpu_log.append(self._process.get_cpu_percent(interval=0))
            virt, real = self._process.get_memory_info()
            self.virt_log.append(virt)
            self.res_log.append(real)
            self.num_threads = max(self.num_threads, self._process.get_num_threads())
        except psutil.NoSuchProcess:
            rospy.logwarn("Lost Node Monitor for '%s'"%self.node)
            self._process = None
            self._process_ok = False

    def get_statistics(self):
        """ Returns NodeStatistics() using information stored in the buffer.  """
        statistics = NodeStatistics()
        statistics.node = self.node
        statistics.host = self.hostname
        statistics.uri = self.uri
        statistics.pid = str(self.pid)
        statistics.samples = len(self.cpu_log)
        statistics.threads = self.num_threads
        statistics.window_start = self.start_time
        statistics.window_stop = rospy.get_rostime()

        if len(self.cpu_log) > 0:
            cpu_log = np.array(self.cpu_log)
            statistics.cpu_load_mean = np.mean(cpu_log)
            statistics.cpu_load_std = np.std(cpu_log)
            statistics.cpu_load_max = np.max(cpu_log)
        if len(self.virt_log) > 0:
            virt_log = np.array(self.virt_log)
            statistics.virt_mem_mean = np.mean(virt_log)
            statistics.virt_mem_std = np.std(virt_log)
            statistics.virt_mem_max = np.max(virt_log)
        if len(self.res_log) > 0:
            res_log = np.array(self.res_log)
            statistics.real_mem_mean = np.mean(res_log)
            statistics.real_mem_std = np.std(res_log)
            statistics.real_mem_max = np.max(res_log)
        return statistics

    def reset(self):
        """ Clears the statistics information stored in the buffer """
        self.cpu_log = list()
        self.virt_log = list()
        self.res_log = list()
        self.num_threads = 0
        self.start_time = rospy.get_rostime()

def custom_exception_handler(type, value, tb):
    """ Gracefully handles ROSInterruptExceptions in rospy.Timer """
    # NOTE: I was having trouble with Timer exiting with an exception, but
    # that problem doesn't seem to be happening anymore. I am leaving this
    # code here in case it is needed in the future.
    if type is rospy.ROSInterruptException:
        return 
    else:
        sys.__excepthook__(type, value, tb)

if __name__ == '__main__':
#     sys.excepthook = custom_exception_handler
    profiler = Profiler()
    try:
        profiler.start()
        rospy.spin()
    except rospy.ROSInitException as e:
        rospy.logerr(str(e))
        rospy.signal_shutdown(str(e))
    except rospy.ROSInterruptException:
        pass
    finally:
        profiler.stop()
