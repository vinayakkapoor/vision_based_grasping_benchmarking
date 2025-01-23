#!/usr/bin/env python3

import rospy
from benchmarking_pipeline_module.benchmarking_pipeline import BenchmarkTest
from std_srvs.srv import Empty

if __name__ == "__main__":
    rospy.init_node("benchmark_test", log_level=rospy.INFO)

    use_cartesian = rospy.get_param("use_cartesian")
    over_head = rospy.get_param("over_head")
    sim_mode = rospy.get_param("sim_mode")
    benchmark_test = BenchmarkTest(use_cartesian=use_cartesian, over_head=over_head, sim_mode=sim_mode)

    rospy.spin()
    