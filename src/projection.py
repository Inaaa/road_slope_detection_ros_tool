#!/usr/bin/env python
import time

from coordinate_conversion import Trans

import rospy

import numpy as np

import message_filters

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs import point_cloud2

from sensor_msgs.msg import Image ,PointCloud2, PointField

import time

from std_msgs.msg import Header

import sensor_msgs.point_cloud2 as pc2

import matplotlib.pyplot as plt

import struct


class RoadPCL(object):

    def __init__(self):
        super(RoadPCL, self).__init__()

        # init the node
        rospy.init_node('point_segmentation', anonymous=True)


        self._bridge = CvBridge()

        # Subscribe the mask_image, depth_image und rgb_image
        sub_pcl = message_filters.Subscriber('/lidarl_new', PointCloud2)
        sub_mask = message_filters.Subscriber('/road_mask', Image)
        #print('sub_pcl{}'.format(sub_pcl))

        # Advertise the result

        self.pub = rospy.Publisher('road_pcl', PointCloud2, queue_size=1)
        self.pub_points = rospy.Publisher('points_seg', PointCloud2, queue_size=1)

        # Create the message filter
        ts = message_filters.ApproximateTimeSynchronizer( \
            [sub_pcl,sub_mask], \
            200, \
            50)

        ts.registerCallback(self.road_callback)


        # spin
        rospy.spin()

    def shutdown(self):
        """
        shuts down the node
        """
        rospy.signal_shutdown("see you later")

    def generate_disparity_from_velo(self,points ,labels):
        pc_velo = points[:,:3]
        a =Trans()
        pts_2d = a.project_lidar_to_image(pc_velo)
        pc_velo2 = a.project_lidar_to_vehicle(pc_velo)
        fov_inds = (pts_2d[:, 0] < 3000 - 1) & (pts_2d[:, 0] >= 1000) & \
                   (pts_2d[:, 1] < 2400 - 1) & (pts_2d[:, 1] >= 900)
        fov_inds = fov_inds & (pc_velo[:, 0] > 0) & (pc_velo[:, 0] < 30)
        imgfov_pc_velo = pc_velo[fov_inds, :]  # points in image
        imgfov_pc_velo2 = pc_velo2[fov_inds, :]
        m =0
        for i in range(len(fov_inds)):
            if fov_inds[i]:

                points[i, 4] = 125
                m=m+1

        print("m=", m)

        imgfov_pts_2d = pts_2d[fov_inds, :]
        x2 = imgfov_pc_velo[:, 0]



        imgfov_pts_2d = np.round(imgfov_pts_2d).astype(int)  # segmented position in image.(pixel)
        road_point = []
        
        road_point2 = []

        print("i max = ", imgfov_pts_2d.shape[0])
        for i in range(imgfov_pts_2d.shape[0]):
            if int(imgfov_pts_2d[i, 1]) < 1500:
                if labels[int(imgfov_pts_2d[i, 1] - 900), int(imgfov_pts_2d[i, 0] - 1000)] == 0:

                    road_point.append(imgfov_pc_velo[i, :])
                    road_point2.append(imgfov_pc_velo2[i, :3])
                    
            else:
                if labels[589, int(imgfov_pts_2d[i, 0] - 1000)] == 0:
                    road_point.append(imgfov_pc_velo[i, :])
                    road_point2.append(imgfov_pc_velo2[i, :3])

        road_point = np.array(road_point)
        
        
        road_point2 = np.array(road_point2)
        return road_point2, points



    def road_callback(self,pcl,data):

        cv_image = self._bridge.imgmsg_to_cv2(data,'passthrough')
        print("pcl_width = ", pcl.width)
        print("pcl_height = ", pcl.height)
        a = pcl.data
        print(len(pcl.data))
        point = []
        for p in pc2.read_points(pcl, field_names = ("x", "y", "z"), skip_nans=True):
            
            point.append([p[0],p[1],p[2],0,0,0])
        #points = np.fromstring(pcl.data, dtype=np.uint8).reshape([-1,3])



        points = np.array(point)
        print(type(points))

        print(points.shape)
        road_points, points_seg = self.generate_disparity_from_velo(points,cv_image)

        points_seg = self.pointcloudrgb_ge(points_seg)
        self.pub_points.publish(points_seg)

        pcl=self.pointcloud_ge(road_points)
        self.pub.publish(pcl)
        print('Good generate pcl')


    def pointcloudrgb_ge(self, points_seg):
        '''
        msg = masked rgb image
        depth = depth image

        '''
        #convert massage to numpy array

        pointcloud = points_seg
        points =[]
        for i in range(pointcloud.shape[0]):
            x = pointcloud[i,0]
            y = pointcloud[i,1]
            z = pointcloud[i,2]
            r = 0
            g = pointcloud[i,4]

            b = 0
            a = 255
            #rgb = struct.unpack('I',struct.pack('BBBB',r,g,b,a))[0]
            rgb = struct.unpack('f', struct.pack('i', 0xff0000))[0]
            point =[x,y,z,rgb]
            points.append(point)

        print("type_points_seg", type(pointcloud))

        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  PointField('rgb', 12, PointField.UINT32, 1),
                  ]

        header =Header()
        header.frame_id = "vehicle"

        point_generate = point_cloud2.create_cloud(header, fields, points)
        point_generate.header.stamp = rospy.Time.now()

        return point_generate


    def pointcloud_ge(self, road_points):
        '''
        msg = masked rgb image
        depth = depth image

        '''
        #convert massage to numpy array

        pointcloud = road_points

        print("type_pointcloudxyz", type(pointcloud))
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  ]

        header =Header()
        header.frame_id = "vehicle"

        point_generate = point_cloud2.create_cloud(header, fields, pointcloud)
        point_generate.header.stamp = rospy.Time.now()

        return point_generate

def main():

    print("!!!!!!!!!!!!!!!")

    node = RoadPCL()


if __name__ == '__main__':
    main()