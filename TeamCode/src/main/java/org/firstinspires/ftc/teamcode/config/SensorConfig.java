package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.kodikas.ftclib.math.MathUtil;

public final class SensorConfig {

    public static final double PINPOINT_FORWARD_POD_SIDEWAYS_OFFSET_MM = -84.0;
    public static final double PINPOINT_STRAFE_POD_FORWARD_OFFSET_MM = -168.0;
    public static final double PINPOINT_FOUR_BAR_TICKS_PER_MM = 19.894367218017578;
    public static final double PINPOINT_SWINGARM_TICKS_PER_MM = 13.262911796569824;
    public static final GoBildaPinpointDriver.GoBildaOdometryPods PINPOINT_POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_FORWARD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final GoBildaPinpointDriver.EncoderDirection PINPOINT_STRAFE_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public static final double PINPOINT_DEFAULT_YAW_SCALAR = 1.0;
    public static final double PINPOINT_ESD_MAX_STEP_M_PER_TICK = 0.20;
    public static final boolean PINPOINT_VELOCITIES_ARE_FIELD_FRAME = true;

    public static final int LIMELIGHT_PIPELINE_APRILTAG = 0;
    public static final int LIMELIGHT_PIPELINE_POLLEN = 1;
    public static final int LIMELIGHT_POLL_RATE_HZ = 60;
    public static final double LIMELIGHT_CAMERA_FORWARD_OFFSET_M = 0.165;
    public static final double LIMELIGHT_CAMERA_SIDEWAYS_OFFSET_M = 0.0;
    public static final double LIMELIGHT_CAMERA_HEIGHT_M = 0.285;
    public static final double LIMELIGHT_HFOV_DEG = 54.5;
    public static final double LIMELIGHT_VFOV_DEG = 42.0;
    public static final boolean LIMELIGHT_BOTPOSE_CENTER_ORIGIN = true;

    public static final double LIMELIGHT_SERVO_MIN_US = 500.0;
    public static final double LIMELIGHT_SERVO_DATUM_US = 1500.0;
    public static final double LIMELIGHT_SERVO_MAX_US = 2500.0;
    public static final double LIMELIGHT_SERVO_DATUM_NORMALIZED = 0.50;
    public static final double LIMELIGHT_SERVO_DATUM_OFFSET_DEG = 0.0;
    public static final double LIMELIGHT_SERVO_SPAN_DEG = 180.0;
    public static final double LIMELIGHT_SERVO_DIRECTION_SIGN = 1.0;
    public static final double LIMELIGHT_SERVO_MIN_LIMIT = 0.05;
    public static final double LIMELIGHT_SERVO_MAX_LIMIT = 0.95;
    public static final boolean LIMELIGHT_SERVO_REVERSED = false;
    public static final double LIMELIGHT_APRILTAG_UP_PITCH_DEG = 12.0;
    public static final double LIMELIGHT_POLLEN_DOWN_PITCH_DEG = -22.0;

    public static final double VISION_PITCH_BLANKOUT_SEC = 0.120;
    public static final double VISION_MAX_OMEGA_DEG_PER_SEC = 115.0;
    public static final double VISION_MAX_OMEGA_RAD_PER_SEC =
            Math.toRadians(VISION_MAX_OMEGA_DEG_PER_SEC);
    public static final double VISION_ADVISORY_ALPHA = 0.06;
    public static final double VISION_MAX_NUDGE_M = 0.015;
    public static final double VISION_MAX_OUTLIER_M = 0.45;
    public static final double VISION_MIN_TAG_AREA_PERCENT = 0.12;
    public static final double VISION_MAX_LATENCY_MS = 120.0;
    public static final long VISION_MAX_STALENESS_MS = 150L;

    public static final double POLLEN_SPHERE_RADIUS_M = 0.0355;
    public static final double POLLEN_DEFAULT_H_MIN = 14.0;
    public static final double POLLEN_DEFAULT_H_MAX = 27.0;
    public static final double POLLEN_DEFAULT_S_MIN = 185.0;
    public static final double POLLEN_DEFAULT_V_MIN = 72.0;

    public static final int LLPYTHON_LENGTH = 8;
    public static final int LLPYTHON_IDX_VALID = 0;
    public static final int LLPYTHON_IDX_DX_FWD = 1;
    public static final int LLPYTHON_IDX_DY_LEFT = 2;
    public static final int LLPYTHON_IDX_COUNT = 3;
    public static final int LLPYTHON_IDX_H_MIN = 4;
    public static final int LLPYTHON_IDX_H_MAX = 5;
    public static final int LLPYTHON_IDX_S_MIN = 6;
    public static final int LLPYTHON_IDX_V_MIN = 7;

    public static final int LLROBOT_LENGTH = 8;
    public static final int LLROBOT_IDX_PITCH_DEG = 0;
    public static final int LLROBOT_IDX_CAM_HEIGHT_M = 1;
    public static final int LLROBOT_IDX_CAM_FWD_M = 2;
    public static final int LLROBOT_IDX_CAM_SIDE_M = 3;
    public static final int LLROBOT_IDX_CAL_TRIGGER = 4;
    public static final int LLROBOT_IDX_H_MIN = 5;
    public static final int LLROBOT_IDX_H_MAX = 6;
    public static final int LLROBOT_IDX_SV_PACKED = 7;
    public static final double LLROBOT_SV_PACK_SCALE = 1000.0;

    public static final double COLOR_SENSOR_POLL_INTERVAL_SEC = 0.025;
    public static final float COLOR_SENSOR_GAIN = 2.5f;
    public static final double POLLEN_ENTER_DISTANCE_MM = 36.0;
    public static final double POLLEN_EXIT_DISTANCE_MM = 50.0;
    public static final double POLLEN_DEBOUNCE_SEC = 0.085;
    public static final double POLLEN_MIN_HUE_DEG = 25.0;
    public static final double POLLEN_MAX_HUE_DEG = 85.0;
    public static final double POLLEN_MIN_SATURATION = 0.28;

    private static final double MIN_SAFE_DENOMINATOR = 1.0e-6;

    private SensorConfig() {
    }

    public static double pitchDegToServoNormalized(
            double pitchDeg,
            double datumOffsetDeg,
            double spanDeg,
            double dirSign) {
        double denom = dirSign * spanDeg;
        if (Math.abs(denom) <= MIN_SAFE_DENOMINATOR) {
            return LIMELIGHT_SERVO_DATUM_NORMALIZED;
        }
        double normalized = LIMELIGHT_SERVO_DATUM_NORMALIZED + ((pitchDeg - datumOffsetDeg) / denom);
        return MathUtil.clamp(normalized, LIMELIGHT_SERVO_MIN_LIMIT, LIMELIGHT_SERVO_MAX_LIMIT);
    }

    public static double servoNormalizedToPitchDeg(
            double servoNormalized,
            double datumOffsetDeg,
            double spanDeg,
            double dirSign) {
        return datumOffsetDeg + dirSign * (servoNormalized - LIMELIGHT_SERVO_DATUM_NORMALIZED) * spanDeg;
    }
}
