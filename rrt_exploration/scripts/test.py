#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from kobuki_msgs.msg import BumperEvent
rospy.init_node( 'vel_bumper' )
pub = rospy.Publisher( '/mobile_base/commands/velocity', Twist, queue_size = 10 )

def main():
    sub = rospy.Subscriber( '/mobile_base/events/bumper', BumperEvent, callback )

def callback( bumper ):
    print ('aaaaa')
    print bumper
    back_vel = Twist() # make Twist type message
    back_vel.linear.x = -1.0
    rate = rospy.Rate(10.0)

    for i in rage(5):
        pub.publishe(back_vel)
        rate.sleep()

    pub.publish( vel ) #Publish
if __name__ == "__main__":
    main()