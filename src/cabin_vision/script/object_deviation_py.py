#python vesion of object_deviation, more imformation refer to the object_deviation.cpp
#!/usr/bin/env python
import rospy
import yaml
import sys
from darknet_ros_msgs.msg import BoundingBoxes
from geometry_msgs.msg import PointStamped

import time

class ObjectDeviationPy():
    def __init__(self):
        self.properties_file = rospy.get_param("~properties_file","")
        with open(self.properties_file, 'rb') as f:
            data = yaml.load(f)
            self.image_width = data["image_width"]
            self.image_height = data["image_height"]
            f.close()
        #self.image_width = 800
        #self.image_height = 600
        print(self.image_width)
        self.deviation_msg = PointStamped()
        self.bounding_boxes_sub = rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, self.boundingBoxesCb)
        self.deviation_pub = rospy.Publisher("/cabin_vision/deviation", PointStamped, queue_size=5)

    def boundingBoxesCb(self, msg):
        self.deviation_msg.header.stamp = rospy.Time.now()
        self.deviation_msg.point.x = self.image_width / 2 - (msg.bounding_boxes[0].xmin + msg.bounding_boxes[0].xmax) / 2.0
        self.deviation_msg.point.y = self.image_height / 2 - (msg.bounding_boxes[0].ymin + msg.bounding_boxes[0].ymax) / 2.0
        self.deviation_pub.publish(self.deviation_msg)
        #rate = rospy.Rate(50)
        #rate.sleep()

if __name__ == '__main__':
    rospy.init_node("object_deviation_py")
    try:
        node = ObjectDeviationPy()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("caught exception")
    print("exiting")

