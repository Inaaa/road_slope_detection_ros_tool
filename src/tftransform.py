import roslib

import rospy
from sensor_msgs.msg import PointCloud2, Image
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
#from transform_point_cloud.cfg import LookupTransformConfig
import tf2_ros
import tf2_py as tf2

from dynamic_reconfigure.server import Server
from geometry_msgs.msg import TransformStamped
from coordinate_conversion import Trans


class TransformPointCloud:
    def __init__(self):
        print("!!!!!!!!")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10))
        self.tl = tf2_ros.TransformListener(self.tf_buffer)
        self.pub = rospy.Publisher("point_cloud_transformed", PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber("/preproc/lidar/velodyne/fl/cartesian/points", PointCloud2,
                                    self.point_cloud_callback, queue_size=2)
        self.sub2 = rospy.Subscriber("/sensor/camera/rect/color_front/left/image", Image,self.image_callback)

        #self.sub_camerainfo = rospy.Subscriber("/sensor/camera/rect/color_front/left/camera_info",)

        #self.dr_server = Server(LookupTransformConfig, self.dr_callback)

    def image_callback(self, msg):
        #print("image_frame_id = ", msg.header.frame_id)
        self.image_frame = msg.header.frame_id

    def generate_disparity_from_velo(self, pc_velo, labels):

        a = Trans()
        pts_2d = a.project_camera_to_image(pc_velo)
        pc_velo2 = a.project_lidar_to_vehicle(pc_velo)
        fov_inds = (pts_2d[:, 0] < 3000 - 1) & (pts_2d[:, 0] >= 1000) & \
                   (pts_2d[:, 1] < 2400 - 1) & (pts_2d[:, 1] >= 900)
        fov_inds = fov_inds & (pc_velo[:, 0] > 0) & (pc_velo[:, 0] < 30)
        imgfov_pc_velo = pc_velo[fov_inds, :]  # points in image
        imgfov_pc_velo2 = pc_velo2[fov_inds, :]

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
        return road_point2,

    def point_cloud_callback(self, msg):

        print("!!!!!!*******+")
        lookup_time = msg.header.stamp + rospy.Duration(10)
        target_frame = self.image_frame
        source_frame = msg.header.frame_id
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time,
                                                    rospy.Duration(10))
        except tf2.LookupException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        except tf2.ExtrapolationException as ex:
            rospy.logwarn(str(lookup_time.to_sec()))
            rospy.logwarn(ex)
            return
        cloud_out = do_transform_cloud(msg, trans)



        self.pub.publish(cloud_out)

if __name__ == '__main__':
	rospy.init_node('transform_point_cloud')
	transform_point_cloud = TransformPointCloud()
	rospy.spin()