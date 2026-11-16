import math
import cv2
import numpy as np

HALF = 0.5
HFOV_DEG = 54.5
VFOV_DEG = 42.0
HALF_HFOV_RAD = math.radians(HFOV_DEG * HALF)
HALF_VFOV_RAD = math.radians(VFOV_DEG * HALF)
TAN_HALF_HFOV = math.tan(HALF_HFOV_RAD)
TAN_HALF_VFOV = math.tan(HALF_VFOV_RAD)

POLLEN_SPHERE_RADIUS_M = 0.0355
DEFAULT_CAMERA_PITCH_DEG = -22.0
DEFAULT_CAMERA_HEIGHT_M = 0.285
DEFAULT_CAMERA_FORWARD_OFFSET_M = 0.165
DEFAULT_CAMERA_SIDEWAYS_OFFSET_M = 0.0

DEFAULT_H_MIN = 14.0
DEFAULT_H_MAX = 27.0
DEFAULT_S_MIN = 185.0
DEFAULT_V_MIN = 72.0

HUE_MIN_BOUND = 0.0
HUE_MAX_BOUND = 179.0
MIN_HUE_SPAN = 4.0
MIN_SAT_FLOOR = 60.0
SAT_MAX_BOUND = 255.0
SAT_MAX_INT = 255
MIN_VAL_FLOOR = 35.0
VAL_MAX_BOUND = 255.0
VAL_MAX_INT = 255

EROSION_STEPS = 1
DILATION_STEPS = 2
MORPH_KERNEL_SIZE = 3
AREA_MIN_PERCENT = 0.4
AREA_MAX_PERCENT = 85.0
ASPECT_MIN = 0.6
ASPECT_MAX = 1.55
CONVEXITY_MIN_PERCENT = 30.0
CONVEXITY_MAX_PERCENT = 95.0

RETICLE_SIZE_PX = 40
TWO_INT = 2
RETICLE_HALF_PX = RETICLE_SIZE_PX // TWO_INT
PERCENTILE_Q1 = 25.0
PERCENTILE_MEDIAN = 50.0
PERCENTILE_Q3 = 75.0
IQR_MULTIPLIER = 2.5

LLROBOT_IDX_PITCH_DEG = 0
LLROBOT_IDX_CAM_HEIGHT_M = 1
LLROBOT_IDX_CAM_FWD_M = 2
LLROBOT_IDX_CAM_SIDE_M = 3
LLROBOT_IDX_CAL_TRIGGER = 4
LLROBOT_IDX_H_MIN = 5
LLROBOT_IDX_H_MAX = 6
LLROBOT_IDX_SV_PACKED = 7
LLROBOT_MIN_CAMERA_LEN = 4
LLROBOT_MIN_CAL_LEN = 5
LLROBOT_MIN_HSV_LEN = 8
SV_PACK_SCALE = 1000.0
CALIBRATE_TRIGGER_THRESHOLD = 0.5

MIN_VALID_CAMERA_HEIGHT_M = 0.01
MIN_VALID_FORWARD_M = 0.0
RAY_EPSILON = 1.0e-6
ZERO_FLOAT = 0.0
ZERO_INT = 0
ONE_INT = 1
CHANNEL_H = 0
CHANNEL_S = 1
CHANNEL_V = 2
PERCENT_SCALE = 100.0
VALID_FLAG = 1.0
INVALID_FLAG = 0.0
UNSET_DISTANCE_SQ = float("inf")

RETICLE_COLOR_BGR = (0, 255, 255)
CONTOUR_COLOR_BGR = (0, 255, 0)
CENTER_MARKER_COLOR_BGR = (255, 0, 0)
RETICLE_THICKNESS_PX = 2
CONTOUR_THICKNESS_PX = 2
CONTOUR_DRAW_ALL_IDX = -1
CENTER_MARKER_RADIUS_PX = 4
FILLED_CIRCLE_THICKNESS = -1

MORPH_KERNEL = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)
EMPTY_CONTOUR = np.array([[]])

_active_h_min = DEFAULT_H_MIN
_active_h_max = DEFAULT_H_MAX
_active_s_min = DEFAULT_S_MIN
_active_v_min = DEFAULT_V_MIN
_last_override_tuple = None


def clamp_value(value, low_bound, high_bound):
    if value < low_bound:
        return low_bound
    if value > high_bound:
        return high_bound
    return value


def project_pixel_to_robot_nwu(
    cx,
    cy,
    image_width,
    image_height,
    camera_pitch_deg,
    camera_height_m,
    camera_forward_offset_m,
    camera_sideways_offset_m,
):
    if image_width <= ZERO_FLOAT or image_height <= ZERO_FLOAT:
        return False, ZERO_FLOAT, ZERO_FLOAT

    half_w = float(image_width) * HALF
    half_h = float(image_height) * HALF
    tan_ax = ((float(cx) - half_w) / half_w) * TAN_HALF_HFOV
    tan_ay = ((half_h - float(cy)) / half_h) * TAN_HALF_VFOV

    pitch_rad = math.radians(float(camera_pitch_deg))
    cos_p = math.cos(pitch_rad)
    sin_p = math.sin(pitch_rad)

    rx = cos_p - sin_p * tan_ay
    ry = -tan_ax
    rz = sin_p + cos_p * tan_ay

    if rz >= -RAY_EPSILON:
        return False, ZERO_FLOAT, ZERO_FLOAT

    height_delta_m = float(camera_height_m) - POLLEN_SPHERE_RADIUS_M
    ray_scale = -height_delta_m / rz
    if ray_scale <= ZERO_FLOAT:
        return False, ZERO_FLOAT, ZERO_FLOAT

    dx_fwd_m = float(camera_forward_offset_m) + ray_scale * rx
    dy_left_m = float(camera_sideways_offset_m) + ray_scale * ry
    return True, dx_fwd_m, dy_left_m


def calibrate_hsv_from_patch(hsv_patch):
    if hsv_patch is None or hsv_patch.size == ZERO_INT:
        return DEFAULT_H_MIN, DEFAULT_H_MAX, DEFAULT_S_MIN, DEFAULT_V_MIN

    h_channel = hsv_patch[:, :, CHANNEL_H].astype(np.float64)
    s_channel = hsv_patch[:, :, CHANNEL_S].astype(np.float64)
    v_channel = hsv_patch[:, :, CHANNEL_V].astype(np.float64)

    q1_h, median_h, q3_h = np.percentile(
        h_channel, [PERCENTILE_Q1, PERCENTILE_MEDIAN, PERCENTILE_Q3]
    )
    q1_s, median_s, q3_s = np.percentile(
        s_channel, [PERCENTILE_Q1, PERCENTILE_MEDIAN, PERCENTILE_Q3]
    )
    q1_v, median_v, q3_v = np.percentile(
        v_channel, [PERCENTILE_Q1, PERCENTILE_MEDIAN, PERCENTILE_Q3]
    )

    iqr_h = q3_h - q1_h
    iqr_s = q3_s - q1_s
    iqr_v = q3_v - q1_v

    h_min_upper_limit = HUE_MAX_BOUND - MIN_HUE_SPAN
    h_min = clamp_value(median_h - IQR_MULTIPLIER * iqr_h, HUE_MIN_BOUND, h_min_upper_limit)
    h_max = clamp_value(median_h + IQR_MULTIPLIER * iqr_h, h_min + MIN_HUE_SPAN, HUE_MAX_BOUND)
    s_min = clamp_value(median_s - IQR_MULTIPLIER * iqr_s, MIN_SAT_FLOOR, SAT_MAX_BOUND)
    v_min = clamp_value(median_v - IQR_MULTIPLIER * iqr_v, MIN_VAL_FLOOR, VAL_MAX_BOUND)

    return float(h_min), float(h_max), float(s_min), float(v_min)


def unpack_sv_override(sv_packed):
    s_val = math.floor(float(sv_packed) / SV_PACK_SCALE)
    v_val = float(sv_packed) - s_val * SV_PACK_SCALE
    s_clamped = clamp_value(s_val, MIN_SAT_FLOOR, SAT_MAX_BOUND)
    v_clamped = clamp_value(v_val, MIN_VAL_FLOOR, VAL_MAX_BOUND)
    return float(s_clamped), float(v_clamped)


def runPipeline(image, llrobot):
    global _active_h_min, _active_h_max, _active_s_min, _active_v_min, _last_override_tuple

