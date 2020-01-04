#!/usr/bin/env python

import rospy
import dynamic_reconfigure.client

if __name__ == "__main__":
    rospy.init_node('dynamic_client', anonymous=True)

    client = dynamic_reconfigure.client.Client('/move_base/TebLocalPlannerROS')

    config = client.update_configuration({
        'max_vel_x' : 1.1,
    })

    # Now change parameters of

    client = dynamic_reconfigure.client.Client('/move_base/local_costmap/inflation_layer')

    config = client.update_configuration({
        'cost_scaling_factor' : 2.0,
    })
