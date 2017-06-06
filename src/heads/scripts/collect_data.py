#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState, Image
from geometry_msgs.msg import Pose
import time

from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np

# from IPython import embed

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

    (rows, cols, channels) = cv_image.shape
    if cols > 60 and rows > 60:
        cv2.circle(cv_image, (50, 50), 10, 255)
    #print channels[0]
    #cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)
    ## TODO cv_image position will be your input


def listener():
    rospy.init_node('listener', anonymous=True)

    ##rospy.Subscriber("/head/joint_states", JointState, callback)
    rospy.Subscriber("/rrbot/camera1/image_raw", Image, callback2)

    pub = rospy.Publisher('/head/set_pose', Pose, queue_size=10)

    Y = []
    imgs = []
    # embed()
    f = open('yy.csv', 'w')
    x_out = open('xxx.txt', 'w')
    x = -3
    num = 0
    step = 0.5
    time.sleep(2)
    while x <= 3:
        y = -4
        while y <= -1:
            z = -1
            while z <= 3:
                p = Pose()
                p.position.y = y
                p.position.x = x
                p.position.z = z
                pub.publish(p)
                time.sleep(2)
                imgs.append(tmp_X)
                #print tmp_X
                cv2.imwrite('images/' + str(num).zfill(5)  + '.jpeg', tmp_X)
                #np.savetxt("outas.txt", tmp_X)
                rospy.loginfo("%d, %s, %s, %s", num, p.position.x, p.position.y, p.position.z)
                Y.append((num, p.position.x, p.position.y, p.position.z))
                tmpStr = "%d, %s, %s, %s\n" % (num, p.position.x, p.position.y, p.position.z)
                num = num + 1
                f.write(tmpStr)
                z = z + step
            y = y + step
        x = x + step

    X = np.array(imgs, dtype='float32')
    print X
    # X.append(tmp_X)
    ## TODO set pose you want to set
    ## Set pose & publish

    # print("xxxx "+x)

    ## TODO publish pose read image and target -> save
    ## publish another pose, save latest images  and target -> save and so on


if __name__ == '__main__':
    listener()
