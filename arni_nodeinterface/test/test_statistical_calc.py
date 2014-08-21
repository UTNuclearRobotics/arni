#!/usr/bin/env python

import unittest
from arni_processing.host_statistics_handler import *
from arni_processing.host_status import *

import traceback
import rospy

PKG = 'arni_nodeinterface'


class TestStatisticsCalc(unittest.TestCase):

    def test_cpu(self):
        hs = HostStatus()
        cpu_usage = [1 , 2 , 3 , 4 , 5 ]
        t = hs.calc_stat_tuple(cpu_usage)
        self.assertEqual(t.max , 5)
        self.assertEqual(t.mean , 3)
        self.assertAlmostEqual(t.stddev, 1.58114, delta = 0.01)

    def test_cpu_core(self):
        hs = HostStatus()
        cpu_usage_core = [[1,2,3,4,5], [1,2,3,4,5]]
        ts = [hs.calc_stat_tuple(cpu_usage_core[0]) ,
                hs.calc_stat_tuple(cpu_usage_core[1])]
        for i in range(2):
            self.assertEqual(t[i].max , 5)
            self.assertEqual(t[i].mean , 3)
            self.assertAlmostEqual(t[i].stddev, 1.58114, delta = 0.01)

    def test_empty_list(self):
        hs = HostStatus()
        e_list = []
        tl = hs.calc_stat_tuple(hs._cpu_usage)
        self.assertEqual(tl, e_list)




if __name__ == '__main__':
    import rosunit
    mockSpecs = rospy.get_param(spec_namespace)
    rospy.delete_param(spec_namespace)
    setup_messages()

    rosunit.unitrun(PKG, 'test_statistical_calc', TestStatisticsCalc)
    


