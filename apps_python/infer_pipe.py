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

import numpy as np
from time import time
import threading
import utils
import debug
from post_process import PostProcess
from pointcloud_projection import RadarPointcloudProjector

from collections import deque
import radar.radar_constants as radar_const
import camera_constants
import queue

class InferPipe:
    """
    Class to abstract the threading of multiple inference pipelines
    """

    def __init__(self, sub_flow, gst_pipe, pointcloud_queue=None, mirror_pointcloud=False, track_distance=True):
        """
        Constructor to create an InferPipe object.
        Args:
            sub_flow: sub_flow configuration
            gst_pipe: gstreamer pipe object
        """
        self.sub_flow = sub_flow
        self.gst_pipe = gst_pipe
        self.gst_pre_inp = gst_pipe.get_src(sub_flow.gst_pre_src_name, sub_flow.flow.id)
        self.gst_sen_inp = gst_pipe.get_src(sub_flow.gst_sen_src_name, sub_flow.flow.id)
        self.run_time = sub_flow.model.run_time
        self.post_proc = PostProcess.get(sub_flow, track_distance)

        self.gst_post_out = gst_pipe.get_sink(
            sub_flow.gst_post_sink_name,
            sub_flow.sensor_width,
            sub_flow.sensor_height,
            sub_flow.input.fps,
        )
        self.param = sub_flow.model
        self.pre_proc_debug = None
        self.infer_debug = None

        if sub_flow.debug_config:
            if sub_flow.debug_config.pre_proc:
                self.pre_proc_debug = debug.Debug(sub_flow.debug_config, "pre")
            if sub_flow.debug_config.inference:
                self.infer_debug = debug.Debug(sub_flow.debug_config, "infer")

        self.pipeline_thread = threading.Thread(target=self.pipeline)
        self.stop_thread = False

        self.mirror_pointcloud = mirror_pointcloud

        if pointcloud_queue is not None:
            self.use_radar = True
            self.pointcloud_queue = pointcloud_queue
            self.pointcloud_frames = deque(maxlen=camera_constants.PERSISTENCE_FRAMES) 

            #cam_offset_dist = [0,  -0.025, -0.01] #default guess -- RG
            cam_offset_dist = [camera_constants.IMX219_DEMO_OFFSET_X,  camera_constants.IMX219_DEMO_OFFSET_Y, camera_constants.IMX219_DEMO_OFFSET_Z] #default guess -- RG
            cam_offset_angles = [camera_constants.IMX219_DEMO_ANGLE_RADIANS_PITCH,camera_constants.IMX219_DEMO_ANGLE_RADIANS_YAW, camera_constants.IMX219_DEMO_ANGLE_RADIANS_ROLL] # pitch, yaw, roll
            cam_offset_dist.extend(cam_offset_angles)
            
            self.pointcloud_processor = RadarPointcloudProjector(sensor='imx219_1640x1232', cam_to_radar_offset=cam_offset_dist, mirror=self.mirror_pointcloud)
            self.track_distance = track_distance
        else:
            self.use_radar = False
            self.pointcloud_queue = None
            self.pointcloud_frames = None
            self.pointcloud_processor = None


    def start(self):
        """
        Start the pipeline
        """
        self.pipeline_thread.start()

    def stop(self):
        """
        Stop the pipeline
        """
        self.stop_thread = True

    def pipeline(self):
        """
        Callback function for pipeline thread
        """
        while self.stop_thread == False:
            # capture and pre-process
            input_img = self.gst_pipe.pull_tensor(
                self.gst_pre_inp,
                self.sub_flow.input.loop,
                self.sub_flow.model.crop[0],
                self.sub_flow.model.crop[1],
                self.sub_flow.model.data_layout,
                self.sub_flow.model.input_tensor_types[0],
            )
            if type(input_img) == type(None):
                break

            if self.pre_proc_debug:
                self.pre_proc_debug.log(str(input_img.flatten()))

            if self.use_radar:
                # print('pull pointcloud from queue')
                try:
                    pointcloud = self.pointcloud_queue.get_nowait()
                    #swap y and z; z should be distance and y height for standard camera model
                    pointcloud[:,[1,2]] = pointcloud[:, [2,1]]
                    if len(self.pointcloud_frames) == camera_constants.PERSISTENCE_FRAMES:
                        self.pointcloud_frames.popleft()
                    
                    self.pointcloud_frames.append(pointcloud)

                except queue.Empty: 
                    # print('queue empty, keep going')
                    pass
                except IndexError: 
                    # print('index error; keep going')
                    pass

            # Inference
            start = time()
            result = self.run_time(input_img)
            end = time()
            self.sub_flow.report.report_proctime("dl-inference", (end - start))

            if self.infer_debug:
                self.infer_debug.log(str(result))

            # post-process
            frame = self.gst_pipe.pull_frame(self.gst_sen_inp, self.sub_flow.input.loop)
            if type(frame) == type(None):
                break
            # out_frame = self.post_proc(frame, result, pointcloud)
            if self.use_radar and len(self.pointcloud_frames) > 0:
                t1 = time()
                pointcloud = self.pointcloud_processor(list(self.pointcloud_frames), out_frame.shape)

                out_frame = self.post_proc(frame, result, pointcloud)

                # out_frame  = self.pointcloud_processor.draw_pointcloud_baseline(out_frame, pointcloud)
                t2 = time()

                # print(f'Pointcloud processing + visualization took {t2-t1} s')
            else:
                out_frame = frame
                
            self.gst_pipe.push_frame(out_frame, self.gst_post_out)
            # Increment frame count
            self.sub_flow.report.report_frame()

        self.stop_thread = True
        self.gst_pipe.send_eos(self.gst_post_out)