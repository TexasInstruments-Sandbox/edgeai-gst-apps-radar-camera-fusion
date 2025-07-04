#!/usr/bin/python3
#  Copyright (C) 2025 Texas Instruments Incorporated - http://www.ti.com/
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#    Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
#    Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the
#    distribution.
#
#    Neither the name of Texas Instruments Incorporated nor the names of
#    its contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



import time, copy
import cv2
import numpy as np
import copy
import math
from radar import radar_constants as radar_const
import camera_constants

class RadarPointcloudProjector():
    
    imx219_1640x1232_camera_info = {
        'height_pix': 1232,
        'width_pix': 1640,
        'pix_width_um' : 2.24, #https://www.electronicsdatasheets.com/download/5721ed8ce34e24fd697a913a.pdf; double size for 2x2 binning mode
        'pix_height_um' : 2.24, #https://www.electronicsdatasheets.com/download/5721ed8ce34e24fd697a913a.pdf
        'cam_focal_length_mm' : 2.8, #depends on lens; https://www.amazon.com/dp/B082W4ZSM9
    }

    def __init__(self, sensor='imx219_1640x1232', camera_info={}, intrinsics_matrix=None, cam_to_radar_offset=[0,0,0,0,0,0], mirror=False):
        '''
        projectcameraion info must include:
            height in pixels, 
            width in pixels
            width of a pixel in microns (micrometers)
            height of a pixel in microns
            focal length in millimeters

        alternately, provide an intrinsics matrix (3x3 matrix, see here: https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html)

        cam_to_radar_offset is 6 element array of x,y,z in meters and roll, pitch, yaw in radians describing the difference in position+orientation of the radar and camera. 
            In a global plane, location_camera = location_radar + radar_cam_offset
            X correspond to sensor width, Y to sensor height, and Z is a distance orthogonal to the camera plane
            - positive X shall radar position is right of the camera (from image sensor's outward view) by some distance in meters
            - positive Y shall radar position is beblow camera by distance in meters
            - positive Z shall radar position is in front of camera
            We will assume angles are all 0 for simplicity
        '''
        if sensor == 'imx219_1640x1232':
            self.camera_info = RadarPointcloudProjector.imx219_1640x1232_camera_info

        if intrinsics_matrix is None:

            focal_len_pix_width = self.camera_info['cam_focal_length_mm'] / ( self.camera_info['pix_width_um'] / 1000 )
            focal_len_pix_height = self.camera_info['cam_focal_length_mm'] / ( self.camera_info['pix_height_um'] / 1000 )
            center_x = self.camera_info['width_pix'] / 2 #this is a best guess! intrinsic calibration with opencv is recommended
            center_y = self.camera_info['height_pix'] / 2

            #This will be transpose of canonical matrix format to do later calculations on row-wise 3D points
            self.intrinsic_matrix = np.asarray(([focal_len_pix_width, 0, center_x],[0, focal_len_pix_height, center_y],[0, 0, 1])).T

        else:
            self.intrinsic_matrix = intrinsics_matrix
        # print(self.intrinsic_matrix)

        self.extrinsic_matrix = self.build_extrinsic_matrix(cam_to_radar_offset)
        self.mirror_pointcloud = mirror

        # print(self.extrinsic_matrix)

    def build_extrinsic_matrix(self, cam_to_radar_offset):
        '''
        :param cam_to_radar_offset: [x,y,z, alpha, beta, gamma]. First 3 are distances in meters, last 3 are angles in radians

        x: distance radar is to the right (+) or left (-)of the camera
        y: distance radar is above (+) or below (-) the camera
        z: distance radar is to the in front of (+) or behind (-)
        alpha(pitch): angle along x axis (left-right, parallel to plane of camera/image sensor). Positive is tilted up, negative tilted down w.r.t. camera
        beta(yaw): angle along y axis (up-down), parallel to plan eof camera/image sensor). Positive is tilted toward  the left, negative is tilted toward the right
        gamma(roll): angle along the z (distance) axis. positive is CCW about the principal ray, negative is CW

        Angles are for the radar w.r.t. to the camera. If radar is tilted in postive direction for an axis, then value should be positive. 
        Note that the pitch roll and yaw (and the axes they correspond to) is subject to intepretation by different conventions. 
        There are also multiple possible orderings for applying these angular rotations that results in the final rotation matrix used within the extrinsic matrix

        Refs: http://www.euclideanspace.com/maths/algebra/matrix/orthogonal/rotation/index.htm
           Seems to show rotations based on column vectors, so matrices are generated in transpose to apply for row-vectors
        '''
        print('build extrinsic matrix')
        translation = np.asarray([cam_to_radar_offset[0:3]])

        rotation_angles = cam_to_radar_offset[-3:]
        alpha, beta, gamma = rotation_angles

        # about X axis, pitch, alpha
        rotation_pitch = np.asarray([ [1, 0, 0],   
            [0, math.cos(alpha), -math.sin(alpha)],   
            [0, math.sin(alpha), math.cos(alpha)]]   
            )
        #about Y axis, yaw, beta
        rotation_yaw = np.asarray([ [math.cos(beta), 0, math.sin(beta)],   
            [0, 1, 0],   
            [-math.sin(beta), 0, math.cos(beta)]]  
            )
        #about Z axis, roll, gamma
        rotation_roll = np.asarray([ [math.cos(gamma), -math.sin(gamma), 0],   
            [math.sin(gamma), math.cos(gamma), 0],   
            [0, 0, 1]]   
            )

        # print(rotation_yaw)
        # print(rotation_pitch)
        # print(rotation_roll)

        # ordering matters! typical is to do heading (yaw), attitude (pitch), then bank (roll)
        # We will do pointclouds as row vectors, so need to do R * PC_vector = PC_vector * R_yaw * R_pitch * R_roll 
        rotation = np.matmul(rotation_yaw, np.matmul(rotation_pitch, rotation_roll))

        self.extrinsic_matrix = np.append(rotation, translation, axis=0)
        print("extrinsics:")
        print(self.extrinsic_matrix) #should be 4,3q
        return self.extrinsic_matrix



    def __call__(self, pointcloud_frames, output_frame_shape=(720,1280,3), remove_out_of_bounds_points=True):

        all_pointclouds = np.concatenate((pointcloud_frames), axis=0)
        numpoints = all_pointclouds.shape[0]

        print("Run pointcloud processing for %d points" % numpoints)
        t1 = time.time()
        pointcloud = np.zeros((numpoints, 5))

        frame_h, frame_w, channels = output_frame_shape
        norm_scales = np.asarray([frame_w / self.camera_info['width_pix'], frame_h / self.camera_info['height_pix']])

        pix_min = np.asarray([0,0])
        pix_max = np.asarray([frame_w, frame_h])

        if numpoints > 0:

            distances = np.linalg.norm(all_pointclouds[:,0:3], axis=1) #take L2 norm across x,y,z for magnitude of distance
            doppler = all_pointclouds[:,3]

            projected_points = self.project_points_from_radar_to_camera_2d(copy.copy(all_pointclouds), normalization_scales=norm_scales)

            if self.mirror_pointcloud:
                projected_points[:,0] = frame_w - projected_points[:,0]


            if remove_out_of_bounds_points:
                x_oob = (projected_points[:,0] < 0) |  (projected_points[:,0] >= frame_w)
                y_oob = (projected_points[:,1] < 0) | (projected_points[:,1] >= frame_w)

                good_points = ~(x_oob | y_oob) #NOR; unfortunately no single-op for thisin python
                print(sum(good_points))
            else:
                projected_points = np.clip(projected_points, a_min=pix_min, a_max=pix_max)
        
            pointcloud[:,0:2] = projected_points
            pointcloud[:,2] = all_pointclouds[:,2] # Z distance
            pointcloud[:,3] = distances # radial distance
            pointcloud[:,4] = doppler # positive is moving away from the radar. In meters/sec

            # print('first 3 postprocessed points')
            # print(pointcloud[0:3,:]) #print a few points

            if remove_out_of_bounds_points:
                pointcloud = pointcloud[good_points,:]

            t2 = time.time()
        return pointcloud

    def distort_pointcloud_projections(self, projected_points, k_coeff=camera_constants.IMX219_LENS_RADIAL_DISTORTION, p_coeff=camera_constants.IMX219_LENS_TANGENTIAL_DISTORTION, s_coeff=camera_constants.IMX219_LENS_THIN_PRISM_DISTORTION, fudge_factors=[camera_constants.IMX219_DEMO_FUDGE_FACTOR_X, camera_constants.IMX219_DEMO_FUDGE_FACTOR_Y]) : 
        '''
        https://learnopencv.com/understanding-lens-distortion/
        '''
        distorted_points = np.zeros(projected_points.shape)
        r = np.linalg.norm(projected_points[:,0:2], axis=1)
        
        x = projected_points[:,0]
        y = projected_points[:,1]
        r2 = np.power(r, 2)
        r4 = np.power(r2, 2)
        r6 = np.power(r2, 3)

        k1, k2, k3, k4, k5, k6 = k_coeff
        p1, p2 = p_coeff
        s1, s2, s3, s4 = s_coeff

        radial_distortion = (1 + k1*r2 + k2*r4 + k3*r6) / (1 + k4*r2 + k5*r4 + k6*r6)

        tangential_distortion_x = 2*p1*x*y + p2*(r2+2*np.power(x, 2))
        tangential_distortion_y = 2*p2*x*y + p1*(r2+2*np.power(y, 2)) 

        thin_prism_x = s1 * r2 + s2 * r4
        thin_prism_y = s3 * r2 + s4 * r4

        fudge_factor_x = x * fudge_factors[0]
        fudge_factor_y = y * fudge_factors[1]

        distorted_points[:,0] = x * radial_distortion + tangential_distortion_x + thin_prism_x + fudge_factor_x
        distorted_points[:,1] = y * radial_distortion + tangential_distortion_y + thin_prism_y + fudge_factor_y

        
        return distorted_points 


    def draw_pointcloud_baseline(frame, pointcloud):
        '''
        starter function for visualizing points on frame; use as a basis point
        '''
        
        background_circle_color = (255, 255, 255)
        offset_circle_color = (0, 0, 0)

        for pc in pointcloud:
            size = 10/pc[2]
            size = int(max(min(size,15),2))

            # good to use two circles with high contrast to show more easily on all backgrounds
            cv2.circle(frame, (int(pc[0]), int(pc[1])), size, background_circle_color, -1)
            cv2.circle(frame, (int(pc[0]), int(pc[1])), size-1, offset_circle_color, -1)


        return frame 


    def project_points_from_radar_to_camera_2d(self, pointcloud, normalization_scales=[1,1], print_intermediate=False):
        '''
        https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html
        Unlike CV doc above, we'll be representing points as rows to work better with structure of pointcloud input
           This convention is important when considering the extrinsic matrix

        Point_camera = Point_world * Extrinsic * intrinsic = [u,v,s], where 'u' and 'v' are pixel locations that must be renormalized by psuedo-distance 's'
            where 'world' is the radar's reference point
        '''

        world_points = pointcloud[:,:3] #N,3 <- N,7 datastructure
        world_points[:,1] *= -1 #invert y in world coordinates to end up right-hand-rule compliant coordinate system. 
        # Will also orient Y such that down is increasing in value, consistent with display convention (UPPER left is origin (0,0))
        if print_intermediate:
            print('world points, inversion')
            print(world_points)


        world_points = np.append(world_points, np.ones( (world_points.shape[0],1) ), axis=1) #add ones so translation portion of extrinsic is applied

        #points no 3D in phsyical units (meters), but from camera's coordinate reference
        camera_coord_points = np.matmul(world_points, self.extrinsic_matrix)

        if print_intermediate:
            print('cam coord points, post inversion')
            print(camera_coord_points)

        normalized_points = np.divide(camera_coord_points[:,0:2], camera_coord_points[:,2][:,np.newaxis]) # normalize by psuedo distance scale 's'

        if print_intermediate:
            print('normalized points')
            print(normalized_points)
        #distortion
        distorted_points = np.ones(camera_coord_points.shape)
        distorted_points[:,0:2] = self.distort_pointcloud_projections(normalized_points)

        if print_intermediate:
            print('distorted_points ')
            print(distorted_points)

        projected_points = np.matmul(distorted_points, self.intrinsic_matrix)
        projected_points = projected_points[:,:2]

        #normalize to the frame we'll be visualizing
        projected_points *= normalization_scales

        return projected_points



def draw_pointcloud_baseline(frame, pointcloud):
    '''
    starter function for visualizing points on frame; use as a basis point
    '''
    
    background_circle_color = (255, 255, 255)
    offset_circle_color = (0, 0, 0)

    for pc in pointcloud:
        size = 10/pc[2]
        size = int(max(min(size,15),2))

        # good to use two circles with high contrast to show more easily on all backgrounds
        cv2.circle(frame, (int(pc[0]), int(pc[1])), size, background_circle_color, -1)
        cv2.circle(frame, (int(pc[0]), int(pc[1])), size-1, offset_circle_color, -1)


    return frame 

def test_pointcloud_points_for_standard_HW_setup():
    '''
    Assumes the hardware setup guide was followed with IMX219 and IWR6843 mounted to a 3d printed holder inlcuded in this repo as STL
    '''
    frame_w_test, frame_h_test = 1280, 720
    cam_info_w_test, cam_info_h_test = 1640, 1232
    norm_scales_test = np.asarray([frame_w_test / cam_info_w_test, frame_h_test / cam_info_h_test])
    print(norm_scales_test)
    #X,Y,Z points in meters. Z must be positive
    test_pointcloud_points = np.asarray([
        [0, 0, 1],
        [1, 0, 1],
        [0, 1, 1],
        [1, 1, 1],
        [-1, -1, 1],
        [-1.5, -1.5, 1],
        [-0.5, -0.5, 1],        
        [1.5, 1.5, 1],
        [0.5, 0.5, 1],
    ] )   

    print('input points: ')
    print(test_pointcloud_points)

    cam_offset_dist = [camera_constants.IMX219_DEMO_OFFSET_X,  camera_constants.IMX219_DEMO_OFFSET_Y, camera_constants.IMX219_DEMO_OFFSET_Z] 
    pointcloud_processor = RadarPointcloudProjector(sensor='imx219_1640x1232', cam_to_radar_offset=cam_offset_dist, mirror=True)

    projected_points = pointcloud_processor.project_points_from_radar_to_camera_2d(test_pointcloud_points, norm_scales_test, True)

    print('projected_points')
    print(projected_points)


if __name__ == '__main__':
    test_pointcloud_points_for_standard_HW_setup()
