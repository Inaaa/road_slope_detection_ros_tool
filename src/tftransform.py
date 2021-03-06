import roslib

import rospy
from sensor_msgs.msg import PointCloud2, Image, PointField
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
#from transform_point_cloud.cfg import LookupTransformConfig
import tf2_ros
import tf2_py as tf2

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import TransformStamped
from coordinate_conversion import Trans

import message_filters

from cv_bridge import CvBridge, CvBridgeError

import sensor_msgs.point_cloud2 as pc2
import numpy as np

from std_msgs.msg import Header
import struct
import time


class TransformPointCloud(object):

    def __init__(self):
        super(TransformPointCloud, self).__init__()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        self.target_frame =  'front_color_rect'
        self.count=0

        '''
        rospy.init_node('transform_point_cloud')
        self._bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("point_cloud_transformed", PointCloud2, queue_size=2)

        #self.sub = rospy.Subscriber("/preproc/lidar/velodyne/fl/cartesian/points", PointCloud2,
         #                           self.point_cloud_callback, queue_size=2)
        #self.sub_image = rospy.Subscriber("/sensor/camera/rect/color_front/left/image", Image,self.image_callback)





        # Subscribe the mask_image, depth_image und rgb_image
        self.sub = message_filters.Subscriber('/preproc/lidar/velodyne/fl/cartesian/points', PointCloud2)
        self.sub_mask = message_filters.Subscriber('/sensor/camera/rect/color_front/left/image', Image)
        print("****************")
        #print('sub_pcl{}'.format(sub_pcl))

        # Advertise the result

        #self.pub2 = rospy.Publisher('road_pcl', PointCloud2, queue_size=1)
        #self.pub_points = rospy.Publisher('points_seg', PointCloud2, queue_size=1)

        # Create the message filter
        ts = message_filters.ApproximateTimeSynchronizer( \
            [self.sub, self.sub_mask], \
            200, \
            50)
        print("'''''''''''''#'#######")
        ts.registerCallback(self.point_cloud_callback)
        print("222222222222'''''''''''''#'#######")

        rospy.spin()
        #self.sub_camerainfo = rospy.Subscriber("/sensor/camera/rect/color_front/left/camera_info",)

        #self.dr_server = Server(LookupTransformConfig, self.dr_callback)
    '''
    def image_callback(self, msg):
        print("image_frame_id = ", msg.header.frame_id)
        self.image_frame = msg.header.frame_id

    def segment_pointcloud(self, pc_velo, labels):

        a = Trans()
        pts_2d = a.project_camera_to_image(pc_velo)
        #pts_2d = a.project_lidar_to_image(pc_velo)
        fov_inds = (pts_2d[:, 0] < 3500 - 1) & (pts_2d[:, 0] >= 500) & \
                   (pts_2d[:, 1] < 1536 - 1) & (pts_2d[:, 1] >= 936)
        #fov_inds = fov_inds & (pc_velo[:, 0] > 0) & (pc_velo[:, 0] < 30)
        fov_inds = fov_inds
        imgfov_pc_velo = pc_velo[fov_inds, :]  # points in image


        imgfov_pts_2d = pts_2d[fov_inds, :]
        x2 = imgfov_pc_velo[:, 0]

        imgfov_pts_2d = np.round(imgfov_pts_2d).astype(int)  # segmented position in image.(pixel)
        road_point = []



        print("i max = ", imgfov_pts_2d.shape[0])
        for i in range(imgfov_pts_2d.shape[0]):
            if int(imgfov_pts_2d[i, 1]) < 1536:
                if labels[int(imgfov_pts_2d[i, 1] - 936), int(imgfov_pts_2d[i, 0] - 500)] == 1:
                    road_point.append(imgfov_pc_velo[i, :])

            else:
                if labels[589, int(imgfov_pts_2d[i, 0] - 500)] == 1:
                    road_point.append(imgfov_pc_velo[i, :])


        road_point = np.array(road_point)

        return road_point




    def point_cloud_callback(self, msg, data):
        """

        :param msg: points , type=msg.PointCloud2
        :param data: mask image ,type=np.array
        :return: points, type= msg.PointCloud2
        """


        # tramform from lidar tp camera
        timea = time.time()
        #print("!!!!!!*******+")
        lookup_time = msg.header.stamp + rospy.Duration(10)
        target_frame = self.target_frame
        source_frame = msg.header.frame_id

        if  self.count==0:
            try:
                self.trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(),
                                                        rospy.Duration(10)) #lookup_time,
                print(self.trans.transform.translation)
                print(self.trans.transform.rotation)
            except tf2.LookupException as ex:
                rospy.logwarn(str(lookup_time.to_sec()))
                rospy.logwarn(ex)
                return
            except tf2.ExtrapolationException as ex:
                rospy.logwarn(str(lookup_time.to_sec()))
                rospy.logwarn(ex)
                return
            self.count +=1

        timeb = time.time()

        cloud_out = do_transform_cloud(msg, self.trans)
        print("!!!!!!!!*****")
        print("time in lookup = ",timeb-timea )

        #  tranform from camre to image
        #time0 = time.time()

        point = list(pc2.read_points(cloud_out,field_names = ("x", "y", "z"), skip_nans=True))

        #time1 = time.time()

        #print("time append",time1-time0)
        points = np.array(point)

        cloud_segment = self.segment_pointcloud(points,data)
        time2 = time.time()
        #print("coordinate translate",time2-time1)


        #print("generate filter points1111111111")

        cloud_seg=self.pointcloud_ge(cloud_segment)
        #print("coordinate translate",time.time()-time2)

        print("generate filter points")
        return cloud_seg

    def pointcloud_ge(self, road_points):
        '''
        msg = masked rgb image
        depth = depth image

        '''
        # convert massage to numpy array

        pointcloud = road_points

        print("type_pointcloudxyz", type(pointcloud))
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                  PointField('y', 4, PointField.FLOAT32, 1),
                  PointField('z', 8, PointField.FLOAT32, 1),
                  ]

        header = Header()
        #header.frame_id = 'sensor/lidar/velodyne/fl'
        header.frame_id = 'front_color_rect'

        point_generate = pc2.create_cloud(header, fields, pointcloud)
        point_generate.header.stamp = rospy.Time.now()
        print("3333333333333")

        return point_generate





        #self.pub.publish(cloud_out)
'''
if __name__ == '__main__':

	transform_point_cloud = TransformPointCloud()
'''