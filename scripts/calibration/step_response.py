import rospy

import math
from autoware_msgs.msg import ControlCommandStamped as Cmd

def talker():
    pub = rospy.Publisher('/ctrl_cmd', Cmd, queue_size=10)
    rospy.init_node('step_response', anonymous=True)
    rate=rospy.Rate(10)

    # Augmentation in each step
    augmentation = 0.05

    max_steering_angle = 26 / math.pi
    #  in each step
    num_of_iteration = 100

    index = 0
    cmd = Cmd()

    cmd.cmd.linear_velocity = 0.0   
    cmd.cmd.linear_acceleration = 0.0
    cmd.cmd.steering_angle = 0.0
    while not rospy.is_shutdown():

        cmd.header.stamp = rospy.get_rostime()
        pub.publish(cmd)
        if index >= num_of_iteration:
            if abs(cmd.cmd.steering_angle) > max_steering_angle:
                if abs(cmd.cmd.steering_angle + augmentation) > abs(cmd.cmd.steering_angle - augmentation): 
                    augmentation *= -1
            cmd.cmd.steering_angle += augmentation
            index = 0
        index += 1
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    



        

