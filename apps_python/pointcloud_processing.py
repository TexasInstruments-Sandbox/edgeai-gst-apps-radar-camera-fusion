#  Copyright (C) 2024 Texas Instruments Incorporated - http://www.ti.com/
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
import debug
import radar

class ProcessRadarPointcloud():
    
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
            self.camera_info = ProcessRadarPointcloud.imx219_1640x1232_camera_info

        if intrinsics_matrix is None:

            focal_len_pix_width = self.camera_info['cam_focal_length_mm'] / (self.camera_info['pix_width_um'] / 1000 )
            focal_len_pix_height = self.camera_info['cam_focal_length_mm'] / (self.camera_info['pix_height_um'] / 1000 )
            center_x = self.camera_info['width_pix'] / 2 #this is a best guess! intrinsic calibration with opencv is recommended
            center_y = self.camera_info['height_pix'] / 2

            #This will be transpose of canonical matrix format to make later calculations a bit easier
            self.intrinsic_matrix = np.asarray(([focal_len_pix_width, 0, center_x],[0, focal_len_pix_height, center_y],[0, 0, 1])).T

        else:
            self.intrinsic_matrix = intrinsics_matrix
        # print(self.intrinsic_matrix)

        self.cam_to_radar_offset = cam_to_radar_offset
        self.distance_offset = np.asarray(cam_to_radar_offset[0:3])
        self.angle_offset = np.asarray(cam_to_radar_offset[3:])
        #TODO: add rotations into the extrinsic matrix
        self.extrinsic_matrix = np.asarray(([1,0, 0, self.distance_offset[0]],[0,1,0, self.distance_offset[1]], \
            [0,0,1, self.distance_offset[2]], [0,0,0,1]))
        self.mirror_pointcloud = mirror

        # print(self.extrinsic_matrix)


    def __call__(self, pointcloud_frames, output_frame_shape=(720,1280,3), remove_out_of_bounds_points=True):
        print("Run pointcloud processing")
        t1 = time.time()
        all_pointclouds = np.concatenate((pointcloud_frames), axis=0)
        numpoints = all_pointclouds.shape[0]

        print(f'{numpoints} points to process')
        # print('first 3 preprocessed points')
        # print(all_pointclouds[0:3,:])
        frame_h,frame_w,_ = output_frame_shape

        norm_w = frame_w / self.camera_info['width_pix']
        norm_h = frame_h / self.camera_info['height_pix']
        norm_scales = np.asarray([norm_w, norm_h])

        pix_min = np.asarray([0,0])
        pix_max = np.asarray([frame_w, frame_h])

        distances = np.linalg.norm(all_pointclouds[:,0:3], axis=1) #take L2 norm across x,y,z for magnitude of distance
        doppler = all_pointclouds[:,3]

        projected_points = self.project_points_from_radar_to_camera_2d(copy.copy(all_pointclouds), normalization_scales=norm_scales)

        if self.mirror_pointcloud:
            projected_points[:,0] = frame_w - projected_points[:,0]


        if remove_out_of_bounds_points:
            x_oob = (projected_points[:,0] < 0) |  (projected_points[:,0] >= frame_w)
            y_oob = (projected_points[:,1] < 0) | (projected_points[:,1] >= frame_w)

            good_points = ~(x_oob | y_oob) #NOR; unfortunately no single-op for this
        else:
            projected_points = np.clip(projected_points, a_min=pix_min, a_max=pix_max)
        


        pointcloud = np.zeros((numpoints, 5))
        pointcloud[:,0:2] = projected_points
        pointcloud[:,2] = all_pointclouds[:,2]
        pointcloud[:,3] = distances
        pointcloud[:,4] = doppler

        # print('first 3 postprocessed points')
        # print(pointcloud[0:3,:]) #print a few points

        if remove_out_of_bounds_points:
            pointcloud = pointcloud[good_points,:]

        t2 = time.time()
        print(t2-t1)
        return pointcloud


    def draw_pointcloud_baseline(self, frame, pointcloud):
        '''
        RG starter function; use as a basis point
        '''
        
        background_circle_color = (255, 255, 255)
        offset_circle_color = (0, 0, 0)

        for pc in pointcloud:
            #FIXME do something more interesting with the points
            cv2.circle(frame, (int(pc[0]), int(pc[1])), 3, background_circle_color, -1)
            cv2.circle(frame, (int(pc[0]), int(pc[1])), 2, offset_circle_color, -1)


        return frame 


    def project_points_from_radar_to_camera_2d(self, pointcloud, normalization_scales):
        '''
        https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html
        '''        
        t1 = time.time()

        points = pointcloud[:,:3] #N,3 <- N,7 datastructure

        points[:,1] *= -1 #invert y, since radar considers distance up but pixels increase downward

        # projected_points = np.matmul(self.intrinsic_matrix,  points.T)
        #transposed version 
        projected_points = np.matmul(points, self.intrinsic_matrix)
        # projected_points = projected_points[0:2,:] / projected_points[2,:] # normalize by psuedo distance scale
        projected_points = np.divide(projected_points[:,0:2], projected_points[:,2][:,np.newaxis]) # normalize by psuedo distance scale
        
        #normalize to the frame we'll be visualizing
        projected_points *= normalization_scales
        t2 = time.time()

        # exit()

        return projected_points
