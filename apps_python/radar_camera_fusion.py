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

from draw_resources import ColorPalate
import random
import cv2
import numpy as np
import copy
import debug
import math
from scipy.stats import gaussian_kde
from scipy.signal import find_peaks
from norfair.tracker import TrackedObject
from norfair.drawing.color import Palette
from draw_resources import ColorPalate

# CONSTANTS/PARAMETERS
# Functional parameters
MAX_WORKING_DISTANCE_LIMIT = 4      # The maximum considered distance for pointcloud, if negative no limit [Meter]
MIN_WORKING_DISTANCE_LIMIT = -1     # The minimum considered distance for pointcloud, if negative no limit [Meter]
RESTRICTED_DISTANCE = 1.5             # Distance of the restriceted area. If smaller objects changes color to red [Meter]
MIN_POINTS_FOR_DISTANCE = 4         # Minimum number of points to be considered to calculate distance      
DISTANCE_PERCENT_THRESHOLD = 0.1    # Percentage of acceptable change in distance. If change is bigger, it has to stay for a number of DISTANCE_COUNT_THRESHOLD
DISTANCE_COUNT_THRESHOLD = 5        # Number of times (frames) the big change of distance has to be sustained before actaul distance is changed.
# Drawing parameters
CIRCLE_BASE_R = 4                   # Radius of smallest circle drawn to represent a point 
DSITANCE_DRAW_LEVELS = 6            # Number of distance levels represeted by the size of the drawn circle
CIRCLE_THICKNESS = 2                # Border thickness of circle representing point moving backword 

# Debug/Devilopment
DISTANCE_METHOD = "median"             # Select the method used to calculate the distance. [mean, median, mode, all]



class RadarCameraFusion:
    """
    Keep a list of tracked objects, associate pointcloud with objects, calcuated distance based on pointcloud
    """
    def __init__(self):
        self.distance_tracker_list = dict()

    def update(self, tracked_objects):
        """
        update list of distance objects.
        Parameters:
            tracked_objects: list of tracked_objects.
        """
        list_id = []
        #update list of distance objects, add objects if does not exist
        for obj in tracked_objects:
            list_id.append(obj.id)
            if obj.id in self.distance_tracker_list:
                self.distance_tracker_list[obj.id].update_bbox(obj.estimate)
            else:
                self.distance_tracker_list[obj.id] = FusionObject(id = obj.id, bbox = obj.estimate, label=obj.label, color=Palette.choose_color(obj.id))

        # keep only new or updated objects
        self.distance_tracker_list = {key: self.distance_tracker_list[key] 
            for key in self.distance_tracker_list if key in list_id}

    def filter_pointcloud(self, pointcloud):
        """
            Filter points with distance or coordinates out of the range specified by the parameter setup up at the begining of this file.
            Arge: 
                pointcloud: array of points cloud 
            return:
                pointcloud: array of points cloud after filtering
        """
        if MAX_WORKING_DISTANCE_LIMIT > 0:
            z_mask_max = pointcloud[:,3] < MAX_WORKING_DISTANCE_LIMIT
            pointcloud = pointcloud[z_mask_max]
        if MIN_WORKING_DISTANCE_LIMIT > 0:
            z_mask_min = pointcloud[:,3] > MIN_WORKING_DISTANCE_LIMIT
            pointcloud = pointcloud[z_mask_min]
        return(pointcloud) 

    def associate_pointcloud(self, pointcloud):
        """
        Associate pointcloud with the detected objects based on the overlap between point cloud and bounding box
        Args:
            pointcloud: array of point cloud from the radar
        """

        # filter point cloud
        f_pointcloud = self.filter_pointcloud(pointcloud)

        for key in self.distance_tracker_list:
            x_mask = np.logical_and(f_pointcloud[:,0] > self.distance_tracker_list[key].bbox[0,0], f_pointcloud[:,0] < self.distance_tracker_list[key].bbox[1,0])
            y_mask = np.logical_and(f_pointcloud[:,1] > self.distance_tracker_list[key].bbox[0,1], f_pointcloud[:,1] < self.distance_tracker_list[key].bbox[1,1])

            self.distance_tracker_list[key].pointcloud = f_pointcloud[x_mask & y_mask]

    def calculate_distance(self):
        """
        Cacluate the distance of an object based on the associated pointcloud
        """
        for key in self.distance_tracker_list:
            
            if len(self.distance_tracker_list[key].pointcloud) > MIN_POINTS_FOR_DISTANCE:
                
                mean = np.mean(self.distance_tracker_list[key].pointcloud[:,3])
                median = np.median(self.distance_tracker_list[key].pointcloud[:,3])
                
                if DISTANCE_METHOD == "mode" or DISTANCE_METHOD == "all":
                    try:
                        kde = gaussian_kde(self.distance_tracker_list[key].pointcloud[:,3])
                        # Generate a range of x values for plotting
                        points_diff = max(self.distance_tracker_list[key].pointcloud[:,3]) - min(self.distance_tracker_list[key].pointcloud[:,3])
                        x = np.linspace(min(self.distance_tracker_list[key].pointcloud[:,3]), max(self.distance_tracker_list[key].pointcloud[:,3]), 100)
                        # # Evaluate the KDE at each x value
                        y = kde.evaluate(x)
                        y = kde(x)
                        mode = x[np.argmax(y)]
                    
                    except:
                        print("pointcloud d = ", self.distance_tracker_list[key].pointcloud[:,3])
                        mode = median
                else:
                    mode = median 

                self.distance_tracker_list[key].update_distance(mean, median, mode)

    def find_nearest(self, array, value):
        array = np.asarray(array)
        idx = (np.abs(array - value)).argmin()
        return array[idx]

    def distance_multimodal(self):
        """
        Use multimodal statistics to calculate distance. 
        FIXME: delete function or correct missing 'key' variable /RG
        """
        try:
            kde = gaussian_kde(self.distance_tracker_list[key].pointcloud[:,3])
            # Generate a range of x values for plotting
            points_diff = max(self.distance_tracker_list[key].pointcloud[:,3]) - min(self.distance_tracker_list[key].pointcloud[:,3])
            x = np.linspace(min(self.distance_tracker_list[key].pointcloud[:,3]), max(self.distance_tracker_list[key].pointcloud[:,3]), 100)
            # # Evaluate the KDE at each x value
            y = kde.evaluate(x)
            y = kde(x)
            if self.distance_tracker_list[key].d_mode is None:
                mode = x[np.argmax(y)]
            else:
                peak_distance = points_diff/100
                peak_distance = max(peak_distance, 1)
                peaks_idx, _ = find_peaks(y, distance=peak_distance)
                print("peaks_ids = ", peaks_idx)
                peaks = x[peaks_idx]
                mode = self.find_nearest(peaks, self.distance_tracker_list[key].d_mode)
        except:
            print("pointcloud d = ", self.distance_tracker_list[key].pointcloud[:,3])
            mode = np.median(self.distance_tracker_list[key].pointcloud[:,3])

        return mode
                
    def safety_check(self):
        """
        Check if the object is outside of restricted area based on their distance.
        """
        for key in self.distance_tracker_list:                
            if self.distance_tracker_list[key].d_median is not None and self.distance_tracker_list[key].d_median < RESTRICTED_DISTANCE:
                self.distance_tracker_list[key].is_restrict = True
            else:
                self.distance_tracker_list[key].is_restrict = False


    def draw_distance(self, frame):
        """
        draw distance of object
        Args:
            frame (numpy array): a three dimensional array representing the frame (image).
            text_size:
            text_thickness:
        Returns:
            numpy array: a three dimensional array of the frame with timeing data.
        """
        text_size = 1.5
        text_thickness = 3
        # text_color = Palette.choose_color(key)
        for key in self.distance_tracker_list:
            if self.distance_tracker_list[key].is_restrict:
                text_color = ColorPalate.DANGER.value
            else:
                text_color = self.distance_tracker_list[key].color
            # text = "D: Mean:{:.2f}".format(obj.d_mean)
            # print("distance m = ", obj.d_mean)
            d_mean_text = "{:.2f}".format(self.distance_tracker_list[key].d_mean) if self.distance_tracker_list[key].d_mean is not None else "x.xx"
            d_median_text = "{:.2f}".format(self.distance_tracker_list[key].d_median) if self.distance_tracker_list[key].d_median is not None else "x.xx"
            d_mode_text = "{:.2f}".format(self.distance_tracker_list[key].d_mode) if self.distance_tracker_list[key].d_mode is not None else "x.xx"
            
            # text = "D_mean:{:.2f} D_med:{:.2f} D_mod:{:.2f}".format(self.distance_tracker_list[key].d_mean, self.distance_tracker_list[key].d_median, self.distance_tracker_list[key].d_median)
            match DISTANCE_METHOD:
                case "mean":
                    text = "Distance:" + d_mean_text +"M"
                case "median":
                    text = "Distance:" + d_median_text +"M"
                case "mode":
                    text = "Distance:" + d_mode_text +"M"
                case "all":
                    text = "D_mean:" + d_mean_text + " D_med:" + d_median_text + " D_mod:" + d_mode_text
                case _:
                    text = "D_mean:" + d_mean_text + " D_med:" + d_median_text + " D_mod:" + d_mode_text

            # text = "Distance:" + d_median_text + "M"

            box = self.distance_tracker_list[key].bbox

            coordinates = np.mean(np.array(box), axis=0)

            (text_w, text_h),_ = cv2.getTextSize(str("Distance:x.xxM"), cv2.FONT_HERSHEY_SIMPLEX, text_size, text_thickness*3)

            coordinates[0] = int(coordinates[0] - text_w/2)

            frame_height,frame_width,_ = frame.shape

            coordinates[0] = min(frame_width-text_w,coordinates[0])

            coordinates[0] = max(0,coordinates[0])

            coordinates[1] = int(box[0,1]-6)

            coordinates[1] = max(coordinates[1], text_h)

            # cv2.putText(
            # frame,
            # text,
            # (coordinates.astype(int)),
            # cv2.FONT_HERSHEY_SIMPLEX,
            # text_size,
            # text_color,
            # text_thickness
            # )


            for i, line in enumerate(text.split(' ')):

                (text_w, text_h),_ = cv2.getTextSize(line, cv2.FONT_HERSHEY_DUPLEX, text_size, text_thickness*3)
                x = int(coordinates[0])
                y = int((i)*(text_h+10)) + int(coordinates[1])
                cv2.putText(frame, line,(x,y) , cv2.FONT_HERSHEY_DUPLEX, text_size, ColorPalate.WHITE.value, text_thickness*3)
                cv2.putText(frame, line,(x,y) , cv2.FONT_HERSHEY_DUPLEX, text_size, text_color, text_thickness)


    def draw_pointcloud(self, frame):
        """
        Draw point clouds on frame for each objects.
        """

        for key in self.distance_tracker_list:
            if self.distance_tracker_list[key].is_restrict:
                circle_color = ColorPalate.DANGER.value
            else:
                circle_color = self.distance_tracker_list[key].color

            for point in self.distance_tracker_list[key].pointcloud:
                point_xy = (int(point[0]), int(point[1]))
                
                rev_d = DSITANCE_DRAW_LEVELS-point[3]
                rev_d = math.ceil(max(1,rev_d))
                circle_r = int(CIRCLE_BASE_R) * rev_d

                # use doppler for circle fill
                if point[4] > 0:
                    thickness = CIRCLE_THICKNESS
                else:
                    thickness = -1
                cv2.circle(
                    frame,
                    point_xy,
                    radius=circle_r,
                    color=circle_color,
                    thickness=thickness)

    def draw_bounding_box(self, frame):
        """
        Draw danger box around objects in wrong area
        """
        for key in self.distance_tracker_list:
            if self.distance_tracker_list[key].is_restrict:
                box_color = ColorPalate.DANGER.value
            else:
                box_color = self.distance_tracker_list[key].color
                
            box = self.distance_tracker_list[key].bbox

            cv2.rectangle(frame, tuple(box[0,:].astype(int)), tuple(box[1,:].astype(int)), box_color, 2)



class FusionObject:
    """
    Object with both vision and radar data
    """
    def __init__(self, id, bbox, label=None, color=None):
        """
        Initialize . 
        Parameters:
            ...
        """
        # parameters
        self.id = id
        self.bbox = bbox
        self.label = label
        if label is None:
            self.label = "People"
        else:
            self.label = label

        if color is None:
            self.color = random.choice(list(ColorPalate)).value
        else:
            self.color = color

        self.is_restrict = False
            
        self.pointcloud = None

        self.d_mean = None
        self.d_median = None
        self.d_mode = None

        # change request counters
        self.d_mean_crc = 0
        self.d_median_crc = 0
        self.d_mode_crc = 0

    def update_bbox(self, bbox):
        self.bbox = bbox

    def update_distance(self, d_mean, d_median, d_mode):
        """
        Updated the old distance measurement but if the change is very high, reject.
        """
        # check percentaed of change
        if self.d_mean is None :
            self.d_mean = d_mean
        else:
            d_mean_cp = self.d_mean - d_mean/self.d_mean

            if abs(d_mean_cp) < DISTANCE_PERCENT_THRESHOLD:
                self.d_mean = d_mean
                self.d_mean_crc = 0
            else:
                if d_mean_cp > 0:
                    self.d_mean_crc += 1
                else:
                    self.d_mean_crc -= 1

            if abs(self.d_mean_crc) > DISTANCE_COUNT_THRESHOLD:
                self.d_mean = d_mean
                self.d_mean_crc = 0


        if self.d_median is None :
            self.d_median = d_median
        else:
            d_median_cp = self.d_median - d_median/self.d_median

            if abs(d_median_cp) < DISTANCE_PERCENT_THRESHOLD:
                self.d_median = d_median
                self.d_median_crc = 0
            else:
                if d_median_cp > 0:
                    self.d_median_crc += 1
                else:
                    self.d_median_crc -= 1

            if abs(self.d_median_crc) > DISTANCE_COUNT_THRESHOLD:
                self.d_median = d_median
                self.d_median_crc = 0


        if self.d_mode is None :
            self.d_mode = d_mode
        else:
            d_mode_cp = self.d_mode - d_mode/self.d_mode

            if abs(d_mode_cp) < DISTANCE_PERCENT_THRESHOLD:
                self.d_mode = d_mode
                self.d_mode_crc = 0
            else:
                if d_mode_cp > 0:
                    self.d_mode_crc += 1
                else:
                    self.d_mode_crc -= 1

            if abs(self.d_mode_crc) > DISTANCE_COUNT_THRESHOLD:
                self.d_mode = d_mode
                self.d_mode_crc = 0



        


