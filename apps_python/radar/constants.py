
UART_MAGIC_WORD = bytearray(b'\x02\x01\x04\x03\x06\x05\x08\x07')


# Defined TLV's
MMWAVE_OUTPUT_MSG_DETECTED_POINTS                      = 1
MMWAVE_OUTPUT_MSG_RANGE_PROFILE                        = 2
MMWAVE_OUTPUT_MSG_NOISE_PROFILE                        = 3
MMWAVE_OUTPUT_MSG_AZIMUT_STATIC_HEAT_MAP               = 4
MMWAVE_OUTPUT_MSG_RANGE_DOPPLER_HEAT_MAP               = 5
MMWAVE_OUTPUT_MSG_STATS                                = 6
MMWAVE_OUTPUT_MSG_DETECTED_POINTS_SIDE_INFO            = 7
MMWAVE_OUTPUT_MSG_AZIMUT_ELEVATION_STATIC_HEAT_MAP     = 8
MMWAVE_OUTPUT_MSG_TEMPERATURE_STATS                    = 9

# IWRL6432 Out-of-box demo packets
MMWAVE_OUTPUT_EXT_MSG_DETECTED_POINTS                  = 301
MMWAVE_OUTPUT_EXT_MSG_RANGE_PROFILE_MAJOR              = 302
MMWAVE_OUTPUT_EXT_MSG_RANGE_PROFILE_MINOR              = 303
MMWAVE_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MAJOR      = 304
MMWAVE_OUTPUT_EXT_MSG_RANGE_AZIMUT_HEAT_MAP_MINOR      = 305
MMWAVE_OUTPUT_MSG_EXT_STATS                            = 306
MMWAVE_OUTPUT_EXT_MSG_PRESENCE_INFO                    = 307
MMWAVE_OUTPUT_EXT_MSG_TARGET_LIST                      = 308
MMWAVE_OUTPUT_EXT_MSG_TARGET_INDEX                     = 309
MMWAVE_OUTPUT_EXT_MSG_MICRO_DOPPLER_RAW_DATA           = 310
MMWAVE_OUTPUT_EXT_MSG_MICRO_DOPPLER_FEATURES           = 311
MMWAVE_OUTPUT_EXT_MSG_RADAR_CUBE_MAJOR                 = 312
MMWAVE_OUTPUT_EXT_MSG_RADAR_CUBE_MINOR                 = 313
MMWAVE_OUTPUT_EXT_MSG_POINT_CLOUD_INDICES              = 314
MMWAVE_OUTPUT_EXT_MSG_ENHANCED_PRESENCE_INDICATION     = 315
MMWAVE_OUTPUT_EXT_MSG_ADC_SAMPLES                      = 316
MMWAVE_OUTPUT_EXT_MSG_CLASSIFIER_INFO                  = 317
MMWAVE_OUTPUT_EXT_MSG_RX_CHAN_COMPENSATION_INFO        = 318

MMWAVE_OUTPUT_MSG_GESTURE_FEATURES_6432               = 350
MMWAVE_OUTPUT_MSG_GESTURE_CLASSIFIER_6432             = 351
MMWAVE_OUTPUT_MSG_GESTURE_PRESENCE_x432               = 352
MMWAVE_OUTPUT_MSG_GESTURE_PRESENCE_THRESH_x432        = 353
MMWAVE_OUTPUT_MSG_GESTURE_CLASSIFIER_PROB_6432        = 354

MMWAVE_OUTPUT_MSG_SPHERICAL_POINTS                     = 1000
MMWAVE_OUTPUT_MSG_TRACKERPROC_3D_TARGET_LIST           = 1010
MMWAVE_OUTPUT_MSG_TRACKERPROC_TARGET_INDEX             = 1011
MMWAVE_OUTPUT_MSG_TRACKERPROC_TARGET_HEIGHT            = 1012
MMWAVE_OUTPUT_MSG_COMPRESSED_POINTS                    = 1020
MMWAVE_OUTPUT_MSG_PRESCENCE_INDICATION                 = 1021
MMWAVE_OUTPUT_MSG_OCCUPANCY_STATE_MACHINE              = 1030
MMWAVE_OUTPUT_MSG_SURFACE_CLASSIFICATION               = 1031

MMWAVE_OUTPUT_MSG_VITALSIGNS                           = 1040

MMWAVE_OUTPUT_MSG_GESTURE_FEATURES_6843                = 1050
MMWAVE_OUTPUT_MSG_GESTURE_OUTPUT_PROB_6843             = 1051

# Expected minimums and maximums to bound the range of colors used for coloring points
SNR_EXPECTED_MIN = 5
SNR_EXPECTED_MAX = 40
SNR_EXPECTED_RANGE = SNR_EXPECTED_MAX - SNR_EXPECTED_MIN
DOPPLER_EXPECTED_MIN = -30
DOPPLER_EXPECTED_MAX = 30
DOPPLER_EXPECTED_RANGE = DOPPLER_EXPECTED_MAX - DOPPLER_EXPECTED_MIN


## Application level constants
import math
PERSISTENCE_FRAMES = 1

#extrinsic calibration values. Think of these are moving from the Radar POV into camera POV. X points to the right, Y points up, and Z points back INTO the radar/camera (Z must be this way to allow right-hand-rule relationships)
#distances in meters
IMX219_DEMO_OFFSET_X = 0.000 # distance radar is to the left of the camera
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
IMX219_DEMO_FUDGE_FACTOR_Y = -0.3 #nonconventional X fixes; intentionally not radial to resolve non-radial error



IMX219_LENS_RADIAL_DISTORTION = [IMX219_DEMO_LENS_DISTORTION_K1, IMX219_DEMO_LENS_DISTORTION_K2, IMX219_DEMO_LENS_DISTORTION_K3, IMX219_DEMO_LENS_DISTORTION_K4, IMX219_DEMO_LENS_DISTORTION_K5, IMX219_DEMO_LENS_DISTORTION_K6]
IMX219_LENS_TANGENTIAL_DISTORTION = [IMX219_DEMO_LENS_DISTORTION_P1, IMX219_DEMO_LENS_DISTORTION_P2]
IMX219_LENS_THIN_PRISM_DISTORTION = [IMX219_DEMO_LENS_DISTORTION_S1, IMX219_DEMO_LENS_DISTORTION_S2, IMX219_DEMO_LENS_DISTORTION_S3, IMX219_DEMO_LENS_DISTORTION_S4]