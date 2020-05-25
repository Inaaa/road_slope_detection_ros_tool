import numpy as np
from autolab_core import RigidTransform

class Trans():
    def __init__(self):

        self.P = [160.794342, 0.0, 600.0, -0.0, 0.0, 160.794342, 400.0, -0.0, 0.0, 0.0, 1.0, 0.0 ]
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
        trans_matrix3_4 = np.hstack((rot, trans))
        trans_matrix = np.vstack(((np.hstack((rot, trans))), [0, 0, 0, 1]))
        return trans_matrix, trans_matrix3_4


    def lidarfl_to_vehicle(self):
        orientation = {'y': 0, 'x': 0, 'z': 0, 'w': 1}
        rotation_quaternion = np.asarray([orientation['w'], orientation['x'], orientation['y'], orientation['z']])
        trans = [[4], [2.5], [0]]

        lidarfl_to_vehicle,lidarfl_to_vehicle2 = self.trans_matrix(rotation_quaternion, trans)
        return lidarfl_to_vehicle,lidarfl_to_vehicle2


    def project_lidar_to_vehicle(self,pc_velo):
        pts_3d_velo = self.cart2hom(pc_velo)
        matrix ,_= self.lidarfl_to_vehicle()
        return np.dot(pts_3d_velo,np.transpose(matrix))


    def project_lidar_to_image(self,pc_velo):
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






#a =Trans()
#print(a.lidarfl_to_camera())



