#!/usr/bin/env python
import rospy

import cv2

from cv_bridge import CvBridge, CvBridgeError

import time

import os
import numpy as np
#import imageio
#from coordinate_conversion import Trans

import numpy as np
import PIL
#from PIL import Image

from deeplab_model import DeepLabModel
import message_filters
from sensor_msgs.msg import Image ,PointCloud2, PointField
from coordinate_conversion import Trans

from std_msgs.msg import Header

import sensor_msgs.point_cloud2 as pc2
#import ros_numpy

class PointSegmentationNode(object):

    INPUT_TENSOR_NAME = 'ImageTensor:0'
    OUTPUT_TENSOR_NAME = 'SemanticPredictions:0'

    def __init__(self):
        super(PointSegmentationNode, self).__init__()

        # init the node
        rospy.init_node('slope')


        self._bridge = CvBridge()


        # Subscribe the mask_image, depth_image und rgb_image

        lidarl = message_filters.Subscriber('/filter_points', PointCloud2,self.slope_callback())
        # Advertise the result

        #self.pub = rospy.Publisher('/slope',   , queue_size=1)

        rospy.spin()




    def shutdown(self):
        """
        Shuts down the node
        """
        rospy.signal_shutdown("See ya!")


    def slope_callback(self,  cloud_msg ):
        print('-' * 40)
        #print(cloud_msg.header)

        points = []
        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"),skip_nans=True):
            #print (" x : %f  y: %f  z: %f" %(p[0],p[1],p[2]))
            points.append([p[0],p[1],p[2]])
        print(points[20])
        print(len(points))





def main():


    node = PointSegmentationNode()




if __name__ == '__main__':
    main()


