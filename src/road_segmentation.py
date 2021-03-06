#!/usr/bin/env python


import rospy


from sensor_msgs.msg import Image, PointCloud2

from cv_bridge import CvBridge, CvBridgeError

import time

import os
import numpy as np
import PIL

from deeplab_model import DeepLabModel

import cv2
from matplotlib import pyplot as plt
import tf2_ros
import tf2_py as tf2

import message_filters
from tftransform import TransformPointCloud
import time

class RoadSegmentationNode(object):

    INPUT_TENSOR_NAME = 'ImageTensor:0'
    OUTPUT_TENSOR_NAME = 'SemanticPredictions:0'

    def __init__(self, MODEL):
        super(RoadSegmentationNode, self).__init__()

        # init the node
        rospy.init_node('road_segmentation')
        self.model = MODEL

        self._bridge = CvBridge()


        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)

        # Advertise the result
        #self.pub_array = rospy.Publisher('/road_detection_array', DetectionArray , queue_size=1)
        self.pub_points = rospy.Publisher("road_pointcloud", PointCloud2)
        self.pub = rospy.Publisher('/road_detection', Image , queue_size=1)
        self.pub_mask = rospy.Publisher('road_mask', Image , queue_size=1)
        #self.pub = rospy.Publisher('people_detection', Image , queue_size=2)
        print('+' * 40)

        #get image from camera
        self.frame = message_filters.Subscriber('/sensor/camera/rect/color_front/left/image', Image)

        self.sub = message_filters.Subscriber('/preproc/lidar/velodyne/fl/cartesian/points', PointCloud2)
        self.count = 0

        ts = message_filters.ApproximateTimeSynchronizer( \
            [self.sub, self.frame], \
            200, \
            50)
        #print("'''''''''''''#'#######")
        ts.registerCallback(self.road_detection_callback)
        self.pp = TransformPointCloud()
        print("start!!!!!!!!!!!")




    def run_visualization(self,frame):
        """Inferences DeepLab model and visualizes result."""

        #original_im = PIL.Image.fromarray(frame)
        #original_im = original_im.crop((1000, 900, 3000, 1536))

        original_im2 = frame[936:1536,500:3500,:]

        res = cv2.resize(original_im2,dsize=(513,102),interpolation=cv2.INTER_CUBIC)

        seg_map = self.model.run(res)
        seg_map = np.array(seg_map,dtype='uint8')

        seg_map2 = cv2.resize(seg_map,(3000,600),interpolation = cv2.INTER_AREA) #2000,600   4096,1536
        #image_resize =np.resize(seg_map,[600,2000])
        #plt.figure()
        #plt.imshow(seg_map2)
        #plt.show()


        return original_im2, seg_map2


    def shutdown(self):
        """
        Shuts down the node
        """
        rospy.signal_shutdown("See ya!")



    def road_detection_callback(self, msg,data):
        """
        :param msg: pointcloud
        :param data: image
        :return:
        """
        #print('*' * 40)


        # get the result of image segmentation , output image and erosion(mask)
        time0 = time.time()
        cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')

        image, mask = self.run_visualization(cv_image)


        #print("time in segmentation", time1 - time0)

        kernel = np.ones((5, 5), np.uint8)
        local_instance_mask = np.array(mask) == 0
        #image = np.array(image,dtype='uint8')


        erosion = cv2.erode(local_instance_mask.astype('uint8'), kernel, iterations=1)
        #print("time_erosion", time.time() - time1)
        #res = cv2.bitwise_and(image,image, mask=erosion)
        #image2 = cv2.add(image,res)


        time1 = time.time()

        point_seg = self.pp.point_cloud_callback(msg,erosion)

        print("time in generate",time.time()-time0)

        #msg_im = self._bridge.cv2_to_imgmsg(np.array(image2), encoding='passthrough')
        #mask_im = self._bridge.cv2_to_imgmsg(np.array(erosion), encoding='passthrough')


        # print('msg_im_type{}'.format(type(msg_im)))
        #print(mask_im.header)
        print("done!!!!!!!!!!!!")
        #self.pub.publish(msg_im)
        #self.pub_mask.publish(mask_im)
        self.pub_points.publish(point_seg)

        print('I PUBLISHED road_detection_topic')


def main():
    time1 = time.time()
    os.environ["CUDA_VISIBLE_DEVICES"] = "1"
    MODEL = DeepLabModel(
        "/mrtstorage/users/chli/cityscapes/exp/train_on_train_set/train_fine/model/frozen_inference_graph.pb")

    node = RoadSegmentationNode(MODEL)

    rospy.spin()


if __name__ == '__main__':
    main()



  #print(type(image))