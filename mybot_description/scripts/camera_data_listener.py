#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
from scipy import ndimage
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):

    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=10)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/mybot/camera1/image_raw", Image, self.callback)

  def dataFilter(self,data):

    img_x = cv2.cvtColor(data,cv2.COLOR_RGB2GRAY)

    roberts_cross_v = np.array( [[ 0, 0, 0 ], [ 0, 1, 0 ], [ 0, 0,-1 ]] )
    roberts_cross_h = np.array( [[ 0, 0, 0], [ 0, -1, 0 ], [ 0,0, 1 ]] )
    vertical = ndimage.convolve(img_x, roberts_cross_v)
    horizontal = ndimage.convolve(img_x, roberts_cross_h)

    output_image = np.sqrt(np.square(horizontal) + np.square(vertical))
    
    return output_image

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    (rows,cols,_) = cv_image.shape
#    if cols > 60 and rows > 60 :
#      cv2.circle(cv_image, (int(rows/2),int(cols/2)), 100, 25, 10)
#      cv2.putText(cv_image, "This is the first time I could do this shit",(10,50),cv2.FONT_HERSHEY_COMPLEX, 0.5, (0,255,0),2)
    cv2.imshow("Image window", self.dataFilter(cv_image))
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
    except CvBridgeError as e:
      print(e)


def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

