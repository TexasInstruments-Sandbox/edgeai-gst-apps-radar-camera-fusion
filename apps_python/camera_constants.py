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

import math

PERSISTENCE_FRAMES = 1

#extrinsic calibration values. Think of these are moving from the Radar POV into camera POV. X points to the right, Y points up, and Z points back INTO the radar/camera (Z must be this way to allow right-hand-rule relationships)
#distances in meters
IMX219_DEMO_OFFSET_X = 0.000 # distance radar is to the right of the camera
IMX219_DEMO_OFFSET_Y = 0.018 # distance radar is below the camera
IMX219_DEMO_OFFSET_Z = 0.001 # distance radar is in front of the camera

#angles in radians; all follow right-hand-rule axes, where Z points into the front of the radar/camera along the principal ray
IMX219_DEMO_ANGLE_RADIANS_YAW = -3 * math.pi/180 # angle about Y axis (right,left). Positive is radar turn right w.r.t. camera plane/lens
IMX219_DEMO_ANGLE_RADIANS_PITCH = 6.5 * math.pi/180 #angle about X axis (up,down). Positive is radar angled down w.r.t. camera plane/lens. 6 is good.
IMX219_DEMO_ANGLE_RADIANS_ROLL = 1 * math.pi/180 #angle about Z axis (clockwise, counter-clockwise). Positive is radar turned counter clockwise w.r.t. camera plane/lens, looking in the direction of principal ray. If looking head-on at radar-camera, flip this angle  (your POV is then opposite principal ray)

#basis: https://learnopencv.com/understanding-lens-distortion/
##values are on normalized points from [-1,1]
IMX219_DEMO_LENS_DISTORTION_K1 = -0.115   #radial, numerator, 2nd polynomial
IMX219_DEMO_LENS_DISTORTION_K2 = 0.0   #radial, numerator, 4th poly
IMX219_DEMO_LENS_DISTORTION_K3 = 0   #radial, numerator, 6th poly

IMX219_DEMO_LENS_DISTORTION_K4 = 0.1 #radial, denominator, 2nd poly
IMX219_DEMO_LENS_DISTORTION_K5 = 0   #radial, denominator, 4th poly
IMX219_DEMO_LENS_DISTORTION_K6 = 0   #radial, denominator, 6th poly

IMX219_DEMO_LENS_DISTORTION_P1 = 0   #tangential, 1st order
IMX219_DEMO_LENS_DISTORTION_P2 = 0   #tangential, 2nd order

IMX219_DEMO_LENS_DISTORTION_S1 = 0   #thin prism, X, 2nd order
IMX219_DEMO_LENS_DISTORTION_S2 = 0   #thin prism, X, 4th order
IMX219_DEMO_LENS_DISTORTION_S3 = 0   #thin prism, Y, 2nd order
IMX219_DEMO_LENS_DISTORTION_S4 = 0   #thin prism, Y, 4th order

IMX219_DEMO_FUDGE_FACTOR_X = 0 #nonconventional X fixes; intentionally not radial
IMX219_DEMO_FUDGE_FACTOR_Y = 0#-0.3 #nonconventional X fixes; intentionally not radial to resolve non-radial error



IMX219_LENS_RADIAL_DISTORTION = [IMX219_DEMO_LENS_DISTORTION_K1, IMX219_DEMO_LENS_DISTORTION_K2, IMX219_DEMO_LENS_DISTORTION_K3, IMX219_DEMO_LENS_DISTORTION_K4, IMX219_DEMO_LENS_DISTORTION_K5, IMX219_DEMO_LENS_DISTORTION_K6]
IMX219_LENS_TANGENTIAL_DISTORTION = [IMX219_DEMO_LENS_DISTORTION_P1, IMX219_DEMO_LENS_DISTORTION_P2]
IMX219_LENS_THIN_PRISM_DISTORTION = [IMX219_DEMO_LENS_DISTORTION_S1, IMX219_DEMO_LENS_DISTORTION_S2, IMX219_DEMO_LENS_DISTORTION_S3, IMX219_DEMO_LENS_DISTORTION_S4]