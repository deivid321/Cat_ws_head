#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Pose

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np
#from IPython import embed

tmp_X = []



bridge = CvBridge()

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.position)

    ## TODO data position will be your target


def callback2(data):
    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
      global tmp_X
      tmp_X = cv_image
    except CvBridgeError as e:
      print(e)

    (rows,cols,channels) = cv_image.shape
    if cols > 60 and rows > 60 :
      cv2.circle(cv_image, (50,50), 10, 255)

    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    ## TODO cv_image position will be your input

    
def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/head/joint_states", JointState, callback)
    rospy.Subscriber("/rrbot/camera1/image_raw", Image, callback2)

    pub = rospy.Publisher('/head/set_pose', Pose, queue_size=10)

    X = []

    #embed()
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        p = Pose()
	# loop
	p.position.x = -2
	p.position.z = 1
	# X.append(tmp_X)
        ## TODO set pose you want to set
        ## Set pose & publish
        pub.publish(p)
        rate.sleep()

    ## TODO publish pose read image and target -> save
    ## publish another pose, save latest images  and target -> save and so on



if __name__ == '__main__':
    listener()
