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

import rospy
import rosgraph

from ros_statistics_msgs.msg import NodeStatistics

class NodeMonitor(object):
    """ Tracks process statistics of a PID. """
    def __init__(self, name, uri, pid):
        """
        :param str name: the registered node name
        :param str uri: the xmlrpc uri of the node
        :param int pid: the process PID of this node
        """
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
        """ Returns if we are still monitoring the process of the PID.
        :rtype: bool
        """
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
        """ Returns NodeStatistics() using information stored in the buffer. 
        :returns: statistics information collected about the process
        :rtype: NodeStatistics
        """
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
