#!/usr/bin/env python
import rospy
from std_msgs.msg import LaserScan

# Filters out unnecessary points. points farther than the size of the table
# will be set to "None" 
def filter_out(message):
    out = []
    for i in range(0, len(message['ranges'])):
        if message['ranges'][i] is not None :
            if message['ranges'][i] > 3.6 :
                out.append(None)
            else:
                out.append(message['ranges'][i])
        else:
            out.append(message['ranges'][i])
    return out

def generate_filtered_message(message, filtered_data):
    out = message
    out['ranges'] = filtered_data
    return out

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

def talker():
    pub = rospy.Publisher('chatter', LaseScan, queue_size=10)
    rospy.init_node('filtered_scan', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(generate_filtered_message(message, filter_out(message)))
        rate.sleep()

if __name__ == '__main__':
    message = {}
     try:
        listener()
        talker()
    except rospy.ROSInterruptException:
        pass