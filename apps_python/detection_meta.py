 
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
from norfair.tracker import TrackedObject
from norfair.drawing.color import Palette
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
                self.distance_tracker_list[obj.id] = DistanceObject(obj.id, obj.estimate)

        # keep only new of updated objects
        self.distance_tracker_list = {key: self.distance_tracker_list[key] 
            for key in self.distance_tracker_list if key in list_id}


        def associate_pointcloud(pointcloud):
            

    # def calculate_distance(self):

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
            
        self.pointcloud = None

        self.d_mean = 0
        self.d_median = 0
        self.d_mode = 0

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


        


