 
#  Copyright (C) 2021 Texas Instruments Incorporated - http://www.ti.com/
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
from norfair.tracker import TrackedObject
from norfair.drawing.color import Palette
from draw_resources import ColorPalate


class DetectedObject:
    """
    Object Detection information
    """
    def __init__(self, bbox, score, id, label, color=None):
        """
        Initialize . 
        Parameters:
            ...
        """
        # Variables
        self.bbox = bbox
        self.score = score
        self.id = id
        self.label = label
        if color is None:
            self.color = random.choice(list(ColorPalate)).value
        else:
            self.color = color
            
        self.pointcloud = None

        self.d_mean = 0
        self.d_median = 0
        self.d_mode = 0

class DistanceTracker:
    """
    Keep a list of tracked objects, associate pointcloud with objects, calcuated distance based on pointcloud
    """
    def __init__(self):
        self.distance_tracker_list = dict()

        # constants
        self.circle_r_base = 2
        self.thickness = -1
        self.MIN_POINTS_FOR_DISTANCE = 4
        self.RESTRICTED_DISTANCE = 1

    def update(self, tracked_objects):
        """
        update list of distance object.
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
                self.distance_tracker_list[obj.id] = DistanceObject(id = obj.id, bbox = obj.estimate, label=obj.label, color=Palette.choose_color(obj.id))

        # keep only new of updated objects
        self.distance_tracker_list = {key: self.distance_tracker_list[key] 
            for key in self.distance_tracker_list if key in list_id}


    def associate_pointcloud(self, pointcloud):
        """
        Associate point cloud with the detected objects based on the overlap between point cloud and bounding box
        Args:
            pointcloud: array of point cloud from the radar
        """

        for key in self.distance_tracker_list:
            # obj.pointcloud = [
            #     [pt for pt in pointcloud if pt[0]>obj.bbox[0,0] and pt[0]<obj.bbox[1,0] and pt[1]>obj.bbox[0,1] and pt[1]<obj.bbox[1,1]]
            #         for pt in pointcloud
            # ]

            x_mask = np.logical_and(pointcloud[:,0] > self.distance_tracker_list[key].bbox[0,0], pointcloud[:,0] < self.distance_tracker_list[key].bbox[1,0])
            y_mask = np.logical_and(pointcloud[:,1] > self.distance_tracker_list[key].bbox[0,1], pointcloud[:,1] < self.distance_tracker_list[key].bbox[1,1])

            self.distance_tracker_list[key].pointcloud = pointcloud[x_mask & y_mask]

    def calculate_distance(self):
        """
        Cacluate the distance of an object based on the associated pointcloud
        """
        for key in self.distance_tracker_list:
            
            if len(self.distance_tracker_list[key].pointcloud) > self.MIN_POINTS_FOR_DISTANCE:
                self.distance_tracker_list[key].d_mean = np.mean(self.distance_tracker_list[key].pointcloud[:,3])
                self.distance_tracker_list[key].d_median = np.median(self.distance_tracker_list[key].pointcloud[:,3])
                # self.distance_tracker_list[key].d_mode = np.mode(self.distance_tracker_list[key].pointcloud[:,3])
                # claculate distance based on mode of distribution
                
                # if len(self.distance_tracker_list[key].pointcloud[:,3])>2:
                
                #     # Create a KDE

                #     try:
                #         kde = gaussian_kde(self.distance_tracker_list[key].pointcloud[:,3])
                #         # Generate a range of x values for plotting
                #         x = np.linspace(min(self.distance_tracker_list[key].pointcloud[:,3]), max(self.distance_tracker_list[key].pointcloud[:,3]), 100)
                #         # # Evaluate the KDE at each x value
                #         y = kde.evaluate(x)
                #         # # Find the mode
                #         mode = x[np.argmax(y)]
                #         self.distance_tracker_list[key].d_mode = mode

                #         self.distance_tracker_list[key].d_mode = np.median(self.distance_tracker_list[key].pointcloud[:,3])
                #     except:
                #         print("pointcloud d = ", self.distance_tracker_list[key].pointcloud[:,3])
                # else:
                #     self.distance_tracker_list[key].d_mode = np.median(self.distance_tracker_list[key].pointcloud[:,3])

                self.distance_tracker_list[key].d_mode = np.median(self.distance_tracker_list[key].pointcloud[:,3])

            # else:
            #     self.distance_tracker_list[key].d_mean =0
            #     self.distance_tracker_list[key].d_median =0
            #     self.distance_tracker_list[key].d_mode =0

    def safety_check(self):
        """
        Check if the object is outside of restricted area based on their distance.
        """
        for key in self.distance_tracker_list:
            if self.distance_tracker_list[key].d_median is not None and self.distance_tracker_list[key].d_median < self.RESTRICTED_DISTANCE:
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

            text = "D_mean:" + d_mean_text + " D_med:" + d_median_text + " D_mod:" + d_mode_text

            box = self.distance_tracker_list[key].bbox

            coordinates = np.mean(np.array(box), axis=0)

            (text_w, text_h),_ = cv2.getTextSize(str("D_mean:x.xx"), cv2.FONT_HERSHEY_SIMPLEX, text_size, text_thickness)

            coordinates[0] = int(coordinates[0] - text_w/2)

            _,frame_width,_ = frame.shape

            coordinates[0] = min(frame_width-text_w,coordinates[0])

            coordinates[0] = max(0,coordinates[0])

            coordinates[1] = int(box[0,1])

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

                (text_w, text_h),_ = cv2.getTextSize(line, cv2.FONT_HERSHEY_DUPLEX, text_size, text_thickness)
                x = int(coordinates[0])
                y = int((i+1)*(text_h+5) + 10) + int(coordinates[1])
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
                
                rev_d = 8-point[3]
                rev_d = math.ceil(max(1,rev_d))
                circle_r = int(self.circle_r_base) * rev_d

                cv2.circle(
                    frame,
                    point_xy,
                    radius=circle_r,
                    color=circle_color,
                    thickness=self.thickness)


    def draw_safety_box(self, frame):
        """
        Draw danger box around objects in wrong area
        """
        for key in self.distance_tracker_list:
            if self.distance_tracker_list[key].is_restrict:
                box_color = ColorPalate.DANGER.value
                box = self.distance_tracker_list[key].bbox

                cv2.rectangle(frame, tuple(box[0,:].astype(int)), tuple(box[1,:].astype(int)), box_color, 2)



class DistanceObject:
    """
    Detected object with distance information information
    """
    def __init__(self, id, bbox, label=None, color=None):
        """
        Initialize . 
        Parameters:
            ...
        """
        # Variables
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

    def update_bbox(self, bbox):
        self.bbox = bbox

class DetectionBoxDraw:
    """
    Object detection to be drawn
    """
    def __init__(self, bbox, score, id, label, status):
        """
        Initialize . 
        Parameters:
            ...
        """
        # Variables
        self.bbox = bbox
        self.score = score
        self.id = id
        self.label = label
        self.status = status


        


