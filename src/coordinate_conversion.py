import numpy as np
from autolab_core import RigidTransform
class Trans():
    def __init__(self):

        self.P = [1700.0, 0.0, 2047.0, -0.0, 0.0, 1700.0, 1015.0, -0.0, 0.0, 0.0, 1.0, 0.0 ]
        self.P = np.reshape(self.P, [3, 4])
        self.R0 = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.R0 = np.reshape(self.R0, [3, 3])
        self.f_u = self.P[0, 0]

    def trans_matrix(self,rotation_quaternion,trans, points=[1,1,1]):
        T_qua2rota = RigidTransform(rotation_quaternion, points)
        rot = T_qua2rota.rotation
        trans_matrix3_4 = np.hstack((rot, trans))
        trans_matrix = np.vstack(((np.hstack((rot, trans))), [0, 0, 0, 1]))
        return trans_matrix, trans_matrix3_4
    def lidarfl_to_vehicle(self):
        orientation = {'y': -0.000730548134847, 'x': 0.000366438796854, 'z': 0.0407824679138, 'w': 0.999167712926}
        rotation_quaternion = np.asarray([orientation['w'], orientation['x'], orientation['y'], orientation['z']])
        trans = [[1.46054280291], [0.557635077922], [1.49213371609]]

        lidarfl_to_vehicle,lidarfl_to_vehicle2 = self.trans_matrix(rotation_quaternion, trans)
        return lidarfl_to_vehicle,lidarfl_to_vehicle2

    def map_to_vehicle(self):
        rotation_quaternion = [0.533606787485, -0.0 , -0.0, 0.845732697931]
        trans = [[12252.3399045],[-58041.396854],[-0.0]]
        map_to_vehicle, map_to_vehicle2 = self.trans_matrix(rotation_quaternion, trans)
        return map_to_vehicle, map_to_vehicle2

    def camera_to_vehicle(self):
        rotation_quaternion =[0.511507, -0.475892, 0.481761, -0.528955]  #front_color_rect
        trans=[[1.827064],[0.254145],[1.247421]]

        #rotation_quaternion =[0.665085,-0.745494,0.024856,0.035815]  #left
        #trans=[[2.239265],[0.968105],[0.770655]]

        #rotation_quaternion = [0.511599384707, -0.476520387498, 0.481607392478, -0.528439875718]  ##(W,X,Y,Z)  stereo_front_left_rect
        #rotation_quaternion =[0.510227778268, -0.479599719025, 0.484929413309, -0.523922876681] ##(W,X,Y,Z)  stereo_front_left
        #trans =[[1.82781872652],[0.317004405987],[1.24662527497]]
        camera_to_vehicle,camera_to_vehicle2 = self.trans_matrix(rotation_quaternion, trans)
        return camera_to_vehicle,camera_to_vehicle2

    def lidarfl_to_camera(self):
        lidarfl_to_vehicle,_ = self.lidarfl_to_vehicle()
        camera_to_vehicle ,_ = self.camera_to_vehicle()
        return np.dot(lidarfl_to_vehicle, np.linalg.inv(camera_to_vehicle))

    def cart2hom(self, pts_3d):
        ''' Input: nx3 points in Cartesian
            Oupput: nx4 points in Homogeneous by pending 1
        '''
        n = pts_3d.shape[0]
        pts_3d_hom = np.hstack((pts_3d, np.ones((n, 1))))
        return pts_3d_hom

    def project_lidar_to_camera(self,pc_velo):
        pts_3d_velo = self.cart2hom(pc_velo)  # nx4
        matrix =self.lidarfl_to_camera()
        return np.dot(pts_3d_velo, np.transpose(matrix))
    def project_lidar_to_vehicle(self,pc_velo):
        pts_3d_velo = self.cart2hom(pc_velo)
        matrix ,_= self.lidarfl_to_vehicle()
        return np.dot(pts_3d_velo,np.transpose(matrix))
    def project_map_to_vehicle(self, points):
        points = self.cart2hom(points)
        matrix, _ = self.map_to_vehicle()
        return np.dot(points, np.transpose(matrix))

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

    def project_camera_to_image2(self, pts_3d_ref):
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


    def project_camera_to_image(self, pts_3d):

        n = pts_3d.shape[0]
        pts_4d = np.hstack((pts_3d, np.ones((n, 1))))
        #print("self.p = \n", self.P)
        pts_2d = np.transpose(np.dot(self.P, np.transpose(pts_4d)) ) # nx3

        pts_2d[:, 0] /= pts_2d[:, 2]
        pts_2d[:, 1] /= pts_2d[:, 2]
        result = pts_2d[:, 0:2]
        return result



    def project_lidar_to_image2(self, pc_velo):
        pts_4d_velo = self.cart2hom(pc_velo)
        _, lidarfl_to_vehicle2 = self.lidarfl_to_vehicle()
        pts_3d_vehi= np.dot(pts_4d_velo, np.transpose(lidarfl_to_vehicle2))
        pts_4d_vehi = self.cart2hom(pts_3d_vehi)
        _, camera_to_vehicle2 = self.camera_to_vehicle()
        camera_to_vehicle2 = np.linalg.inv(camera_to_vehicle2)

        pts_3d_ref = np.dot(pts_4d_vehi, np.transpose(camera_to_vehicle2))

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






a =Trans()
print(a.lidarfl_to_camera())



