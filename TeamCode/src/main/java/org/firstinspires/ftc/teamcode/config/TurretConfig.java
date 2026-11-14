package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.kodikas.ftclib.ballistics.BallisticProfile;

public final class TurretConfig {

    public static final double TURRET_SERVO_MAX_DEG = 322.00;
    public static final double TURRET_GEAR_RATIO = 2.53;
    public static final double SYMMETRIC_HALF_DIVISOR = 2.0;
    public static final double TURRET_PHYSICAL_HARD_STOP_DEG =
            TURRET_SERVO_MAX_DEG / (SYMMETRIC_HALF_DIVISOR * TURRET_GEAR_RATIO);
    public static final double TURRET_SOFT_LIMIT_DEG = 55.20;
    public static final double TURRET_ASSIST_ONSET_DEG = 50.60;
    public static final double TURRET_BLIND_SPOT_STOW_DEG = 64.40;
    public static final double TURRET_CENTER_PWM = 0.50;
    public static final double TURRET_SERVO_MIN_PWM = 0.0;
    public static final double TURRET_SERVO_MAX_PWM = 1.0;
    public static final boolean TURRET_SERVO_REVERSED = false;
    public static final double TURRET_SERVO_PHASE_LAG_SEC = 0.032;
    public static final double TURRET_PIVOT_OFFSET_X_M = -0.051;
    public static final double TURRET_PIVOT_OFFSET_Y_M = 0.0;
    public static final double TURRET_ASSIST_KP = 2.94;
    public static final double TURRET_MAX_ASSIST_OMEGA_RAD_PER_SEC = 3.22;
    public static final double TURRET_ESTIMATED_MAX_SLEW_DEG_PER_SEC = 202.40;
    public static final double TURRET_ESTIMATED_MAX_SLEW_RAD_PER_SEC =
            Math.toRadians(TURRET_ESTIMATED_MAX_SLEW_DEG_PER_SEC);

    public static final double HOOD_PWM_MIN_US = 460.00;
    public static final double HOOD_PWM_MAX_US = 2300.00;
    public static final double HOOD_SAFE_MIN_US = 552.00;
    public static final double HOOD_SAFE_MAX_US = 2208.00;
    public static final double HOOD_MIN_CLAMP = 0.05;
    public static final double HOOD_MAX_CLAMP = 0.87;
    public static final boolean HOOD_SERVO_REVERSED = false;

    public static final int LUT_KNOT_COUNT = 4;
    public static final double[] DEFAULT_LUT_DISTANCES_M = new double[] {0.83, 1.52, 2.25, 3.08};
    public static final double[] DEFAULT_LUT_HOOD_PITCHES = new double[] {0.15, 0.37, 0.61, 0.81};
    public static final double[] DEFAULT_LUT_TOF_SEC = new double[] {0.17, 0.30, 0.45, 0.61};
    public static final double[] DEFAULT_LUT_SCORING_POWERS = new double[] {0.70, 0.75, 0.81, 0.86};

    public static final double FLYWHEEL_WARM_IDLE_POWER = 0.51;

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
