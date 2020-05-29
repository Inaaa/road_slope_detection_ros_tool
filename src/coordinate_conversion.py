import numpy as np
from autolab_core import RigidTransform

class Trans():
    def __init__(self):

        self.P = [160.794342169, 0.0, 600.0, 0.0, 0.0, 160.794342169, 400.0, 0.0, 0.0, 0.0, 1.0, 0.0 ]
        self.P = np.reshape(self.P, [3, 4])
        self.R0 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.R0 = np.reshape(self.R0, [3, 3])

    def cart2hom(self, pts_3d):
        ''' Input: nx3 points in Cartesian
            Oupput: nx4 points in Homogeneous by pending 1
        '''
        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
        return pts_3d_hom


    def trans_matrix(self,rotation_quaternion,trans, points=[1,1,1]):
        T_qua2rota = RigidTransform(rotation_quaternion, points)
        rot = T_qua2rota.rotation
        #trans_matrix3_4 = np.hstack((rot, trans))
        trans_matrix = np.vstack(((np.hstack((rot, trans))), [0, 0, 0, 1]))
        return trans_matrix

    def lidarfl_to_vehicle(self):
        orientation = {'y': 0.0, 'x': 0.0, 'z': 0.0, 'w': 1.0}
        rotation_quaternion = np.asarray([orientation['w'], orientation['x'], orientation['y'], orientation['z']])
        trans = [[4.0], [2.5], [2.5]]

        lidarfl_to_vehicle = self.trans_matrix(rotation_quaternion, trans)
        return lidarfl_to_vehicle


    def camera_to_vehicle(self):
        rotation_quaternion =[-0.431015333916, -0.560203194005, 0.560649476536, 0.43135869971]  #front_color_rect
        trans=[[4.813],[0.905],[0.910]]

        camera_to_vehicle= self.trans_matrix(rotation_quaternion, trans)
        print("camera to vehicle", np.linalg.inv(camera_to_vehicle))
        return camera_to_vehicle

    def vehicle_to_camera(self):
        rotation_quaternion =[0.431015333916, 0.560203194005, -0.560649476536, 0.43135869971]  #front_color_rect
        trans=[[ 0.35885647669],[0.913889114605],[4.88345397391]]
        vehicle_to_camera= self.trans_matrix(rotation_quaternion, trans)
        print("vahicle to camera", np.linalg.inv(vehicle_to_camera))
        return vehicle_to_camera

    def lidarfl_to_camera(self):
        lidarfl_to_vehicle = self.lidarfl_to_vehicle()
        #camera_to_vehicle  = self.camera_to_vehicle()
        vehicle_to_camera = self.vehicle_to_camera()
        return np.dot(lidarfl_to_vehicle, vehicle_to_camera)
    
    def project_lidar_to_camera(self,pc_velo):
        pts_3d_velo = self.cart2hom(pc_velo)  # nx4
        matrix =self.lidarfl_to_camera()
        return np.dot(pts_3d_velo, np.transpose(matrix))





    def project_lidar_to_image(self,pc_velo):
        pts_3d_ref = self.project_lidar_to_camera(pc_velo)
        pts_3d_ref = pts_3d_ref[:,0:3]
        pts_3d_rect = np.transpose(np.dot(self.R0, np.transpose(pts_3d_ref)))
        pts_3d = pts_3d_rect

        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))

        pts_3d_rect = pts_3d_hom

        #print("self.p = \n", self.P)

        pts_2d = np.dot(pts_3d_rect, np.transpose(self.P))  # nx3
        pts_2d[:, 0] /= pts_2d[:, 2]
        pts_2d[:, 1] /= pts_2d[:, 2]
        result = pts_2d[:, 0:2]
        return result

    def project_vehicle_to_image(self,pc_velo):
        
        pts_3d_ref = pc_velo
        pts_3d_ref = pts_3d_ref[:,0:3]
        pts_3d_rect = np.transpose(np.dot(self.R0, np.transpose(pts_3d_ref)))
        pts_3d = pts_3d_rect

        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))

        pts_3d_rect = pts_3d_hom

        print("self.p = \n", self.P)

        pts_2d = np.dot(pts_3d_rect, np.transpose(self.P))  # nx3
        pts_2d[:, 0] /= pts_2d[:, 2]
        pts_2d[:, 1] /= pts_2d[:, 2]
        result = pts_2d[:, 0:2]
        return result

    def project_camera_to_image(self, pts_3d):

        n = pts_3d.shape[0]
        pts_4d = np.hstack((pts_3d, np.ones((n, 1))))
        #print("self.p = \n", self.P)
        pts_2d = np.transpose(np.dot(self.P, np.transpose(pts_4d)) ) # nx3

        pts_2d[:, 0] /= pts_2d[:, 2]
        pts_2d[:, 1] /= pts_2d[:, 2]
        result = pts_2d[:, 0:2]
        return result



    '''
    def project_camera_to_image(self,pc_velo):
        pts_3d_ref = pc_velo 
        pts_3d_ref = pts_3d_ref[:,0:3]
        pts_3d_rect = np.transpose(np.dot(self.R0, np.transpose(pts_3d_ref)))
        pts_3d = pts_3d_rect

        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))

        pts_3d_rect = pts_3d_hom

        print("self.p = \n", self.P)

        pts_2d = np.dot(pts_3d_rect, np.transpose(self.P))  # nx3
        pts_2d[:, 0] /= pts_2d[:, 2]
        pts_2d[:, 1] /= pts_2d[:, 2]
        result = pts_2d[:, 0:2]
        return result

    '''




a =Trans()
#print(a.lidarfl_to_camera())
a.camera_to_vehicle()
a.vehicle_to_camera()


