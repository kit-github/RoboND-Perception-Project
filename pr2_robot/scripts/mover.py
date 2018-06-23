

import math
import rospy
from std_msgs.msg import  Float64



def mover(topic, msg_type, value_fn):

    pub_j1 = rospy.Publisher(topic, msg_type, queue_size=10)
    rospy.init_node('robot_mover')
    rate = rospy.Rate(10)
    start_time = 0

    while not start_time:
        start_time = rospy.Time.now().to_sec()

    while not rospy.is_shutdown():
        t = rospy.Time.now()-start_time
        pub_j1.publish(value_fn(t))
        rate.sleep()
