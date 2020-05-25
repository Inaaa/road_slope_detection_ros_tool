
import time

from coordinate_conversion import CarMakerTrans

import rospy

import numpy as np

import message_filters

from cv_bridge import CvBridge, CvBridgeError

#from sensor_msgs import point_cloud2

from sensor_msgs.msg import Image ,PointCloud2, PointField



from std_msgs.msg import Header

import sensor_msgs.point_cloud2 as pc2



class RoadPCL(object):

    def __init__(self):
        super(RoadPCL, self).__init__()

        # init the node
        rospy.init_node('point_segmentation', anonymous=True)


        self._bridge = CvBridge()

        # Subscribe the mask_image, depth_image und rgb_image
        sub_pcl = message_filters.Subscriber('/lidar', PointCloud2)
        sub_mask = message_filters.Subscriber('FL_front_color/image_raw', Image)
        #print('sub_pcl{}'.format(sub_pcl))

        # Advertise the result

        self.pub = rospy.Publisher('road_pointcloud', PointCloud2, queue_size=1)


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

    def generate_disparity_from_velo(self,pc_velo ,labels):
        a =CarMakerTrans()
        pts_2d = a.project_lidar_to_image(pc_velo)
        pc_velo2 = a.project_lidar_to_vehicle(pc_velo)
        print(labels.shape)
        fov_inds = (pts_2d[:, 0] < 1200) & (pts_2d[:, 0] >= 0) & \
                   (pts_2d[:, 1] < 800) & (pts_2d[:, 1] >= 0)
        fov_inds = (pc_velo[:, 0] > 0) & (pc_velo[:, 0] < 40)
        imgfov_pc_velo = pc_velo[fov_inds, :]  # points in image
        imgfov_pc_velo2 = pc_velo2[fov_inds, :]

        imgfov_pts_2d = pts_2d[fov_inds, :]
        x2 = imgfov_pc_velo[:, 0]



        imgfov_pts_2d = np.round(imgfov_pts_2d).astype(int)  # segmented position in image.(pixel)
        road_point = []
        road_pos = []
        road_point2 = []

        print("i max = ", imgfov_pts_2d.shape[0])
        for i in range(imgfov_pts_2d.shape[0]):

            if np.any(labels[int(imgfov_pts_2d[i, 1]), int(imgfov_pts_2d[i, 0])] == 125) :
                road_point.append(imgfov_pc_velo[i, :])
                road_point2.append(imgfov_pc_velo2[i, :3])

            elif  np.any(labels[int(imgfov_pts_2d[i, 1]), int(imgfov_pts_2d[i, 0])] == 255) :
                road_point.append(imgfov_pc_velo[i, :])
                road_point2.append(imgfov_pc_velo2[i, :3])



        road_point = np.array(road_point)
        x = road_point[:, 0]
        x1 = road_pos[:, 1]
        road_point2 = np.array(road_point2)
        return road_point2



    def road_callback(self,pcl,data):

        cv_image = self._bridge.imgmsg_to_cv2(data,'bgr8')
        points = np.fromstring(pcl.data, dtype=np.uint8).reshape([-1,3])

        print(type(points))

        print(points.shape)
        road_points = self.generate_disparity_from_velo(points,cv_image)

        pcl=self.pointcloud_ge(road_points)
        self.pub.publish(pcl)
        print('Good generate pcl')


    def pointcloud_ge(self, road_points):
        '''
        msg = masked rgb image
        depth = depth image

        '''
        #convert massage to numpy array

        pointcloud = road_points

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
