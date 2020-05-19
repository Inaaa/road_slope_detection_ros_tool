#!/usr/bin/env python


import rospy


from sensor_msgs.msg import Image

from cv_bridge import CvBridge, CvBridgeError

import time

import os
import numpy as np
import PIL

from deeplab_model import DeepLabModel

import cv2
from matplotlib import pyplot as plt

class RoadSegmentationNode(object):

    INPUT_TENSOR_NAME = 'ImageTensor:0'
    OUTPUT_TENSOR_NAME = 'SemanticPredictions:0'

    def __init__(self, MODEL):
        super(RoadSegmentationNode, self).__init__()

        # init the node
        rospy.init_node('road_segmentation')
        self.model = MODEL

        self._bridge = CvBridge()



        # Advertise the result
        #self.pub_array = rospy.Publisher('/road_detection_array', DetectionArray , queue_size=1)
        self.pub = rospy.Publisher('/road_detection', Image , queue_size=1)
        self.pub_mask = rospy.Publisher('road_mask', Image , queue_size=1)
        #self.pub = rospy.Publisher('people_detection', Image , queue_size=2)
        print('+' * 40)

        #get image from camera
        self.frame = rospy.Subscriber('/sensor/camera/rect/color_front/left/image',\
            Image, self.road_detection_callback,queue_size=1)
        #print(self.frame)
        #print('~' * 40)



    def run_visualization(self,frame):
        """Inferences DeepLab model and visualizes result."""




        original_im = PIL.Image.fromarray(frame)
        original_im = original_im.crop((1000, 900, 3000, 1536))
        seg_map = self.model.run(original_im)


        return original_im, seg_map


    def shutdown(self):
        """
        Shuts down the node
        """
        rospy.signal_shutdown("See ya!")

    def road_detection_callback(self, data):
        #print('*' * 40)
        print(data.header)
        cv_image = self._bridge.imgmsg_to_cv2(data, 'bgr8')

        image, mask = self.run_visualization(cv_image)
        time1 = time.time()
        kernel = np.ones((5, 5), np.uint8)
        local_instance_mask = np.array(mask) == 0
        image = np.array(image,dtype='uint8')


        erosion = cv2.erode(local_instance_mask.astype('uint8'), kernel, iterations=1)
        print("time_erosion", time.time() - time1)
        res = cv2.bitwise_and(image,image, mask=erosion)
        image2 = cv2.add(image,res)

        print(time.time()-time1)
        #plt.figure()
        #plt.imshow(image2)
        #plt.imshow(erosion,alpha=0.5)
        #plt.show()



        """ 
        dim = (480, 640)
        image = np.array(image)
        mask = np.array(mask, dtype='uint8')
        res_image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
        res_mask = cv2.resize(mask, dim, interpolation=cv2.INTER_AREA)

        print('image_shape{}'.format(res_image.shape))
        print('mask_shape {}'.format(res_mask.shape))

        res = cv2.bitwise_and(res_image, res_image, mask=res_mask)
        # res_resized = cv2.resize(res,dim,interpolation = cv2.INTER_AREA)
        """





        msg_im = self._bridge.cv2_to_imgmsg(np.array(image2), encoding='passthrough')
        mask_im = self._bridge.cv2_to_imgmsg(np.array(erosion), encoding='passthrough')
        # print('msg_im_type{}'.format(type(msg_im)))
        print(mask_im.header)
        self.pub.publish(msg_im)
        self.pub_mask.publish(mask_im)

        print('I PUBLISHED road_detection_topic')


def main():
    time1 = time.time()
    os.environ["CUDA_VISIBLE_DEVICES"] = "3"
    MODEL = DeepLabModel(
        "/mrtstorage/users/chli/cityscapes/exp/train_on_train_set/train_fine/model/frozen_inference_graph.pb")

    node = RoadSegmentationNode(MODEL)

    rospy.spin()


if __name__ == '__main__':
    main()



  #print(type(image))