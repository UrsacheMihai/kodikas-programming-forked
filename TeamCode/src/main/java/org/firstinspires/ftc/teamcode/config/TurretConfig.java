package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.kodikas.ftclib.ballistics.BallisticProfile;

public final class TurretConfig {

    public static final double TURRET_SERVO_MAX_DEG = 350.0;
    public static final double TURRET_GEAR_RATIO = 2.75;
    public static final double SYMMETRIC_HALF_DIVISOR = 2.0;
    public static final double TURRET_PHYSICAL_HARD_STOP_DEG =
            TURRET_SERVO_MAX_DEG / (SYMMETRIC_HALF_DIVISOR * TURRET_GEAR_RATIO);
    public static final double TURRET_SOFT_LIMIT_DEG = 60.0;
    public static final double TURRET_ASSIST_ONSET_DEG = 55.0;
    public static final double TURRET_BLIND_SPOT_STOW_DEG = 70.0;
    public static final double TURRET_CENTER_PWM = 0.50;
    public static final double TURRET_SERVO_MIN_PWM = 0.0;
    public static final double TURRET_SERVO_MAX_PWM = 1.0;
    public static final boolean TURRET_SERVO_REVERSED = false;
    public static final double TURRET_SERVO_PHASE_LAG_SEC = 0.035;
    public static final double TURRET_PIVOT_OFFSET_X_M = -0.055;
    public static final double TURRET_PIVOT_OFFSET_Y_M = 0.0;
    public static final double TURRET_ASSIST_KP = 3.2;
    public static final double TURRET_MAX_ASSIST_OMEGA_RAD_PER_SEC = 3.5;
    public static final double TURRET_ESTIMATED_MAX_SLEW_DEG_PER_SEC = 220.0;
    public static final double TURRET_ESTIMATED_MAX_SLEW_RAD_PER_SEC =
            Math.toRadians(TURRET_ESTIMATED_MAX_SLEW_DEG_PER_SEC);

    public static final double HOOD_PWM_MIN_US = 500.0;
    public static final double HOOD_PWM_MAX_US = 2500.0;
    public static final double HOOD_SAFE_MIN_US = 600.0;
    public static final double HOOD_SAFE_MAX_US = 2400.0;
    public static final double HOOD_MIN_CLAMP = 0.05;
    public static final double HOOD_MAX_CLAMP = 0.95;
    public static final boolean HOOD_SERVO_REVERSED = false;

    public static final int LUT_KNOT_COUNT = 4;
    public static final double[] DEFAULT_LUT_DISTANCES_M = new double[] {0.90, 1.65, 2.45, 3.35};
    public static final double[] DEFAULT_LUT_HOOD_PITCHES = new double[] {0.16, 0.40, 0.66, 0.88};
    public static final double[] DEFAULT_LUT_TOF_SEC = new double[] {0.19, 0.33, 0.49, 0.66};
    public static final double[] DEFAULT_LUT_SCORING_POWERS = new double[] {0.76, 0.82, 0.88, 0.94};

    public static final double FLYWHEEL_WARM_IDLE_POWER = 0.55;
    public static final double FLYWHEEL_MAX_RPM = 6000.0;
    public static final DcMotor.RunMode FLYWHEEL_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.ZeroPowerBehavior FLYWHEEL_ZERO_POWER = DcMotor.ZeroPowerBehavior.FLOAT;
    public static final DcMotorSimple.Direction L_TURRET_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction R_TURRET_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final double FLYWHEEL_BOOST_SETTLE_SEC = 0.18;

    public static final double AIM_ENTER_TOLERANCE_DEG = 1.8;
    public static final double AIM_HOLD_TOLERANCE_DEG = 3.5;
    public static final double TURRET_STOW_TOLERANCE_DEG = 2.5;
    public static final double TURRET_STOW_TOLERANCE_RAD = Math.toRadians(TURRET_STOW_TOLERANCE_DEG);

    private TurretConfig() {
    }

    public static BallisticProfile createDefaultBallisticProfile() {
        BallisticProfile profile = new BallisticProfile(
                DEFAULT_LUT_DISTANCES_M,
                DEFAULT_LUT_HOOD_PITCHES,
                DEFAULT_LUT_TOF_SEC,
                DEFAULT_LUT_SCORING_POWERS,
                FLYWHEEL_WARM_IDLE_POWER,
                FLYWHEEL_MAX_RPM);
        profile.setHoodSafetyClamps(HOOD_MIN_CLAMP, HOOD_MAX_CLAMP);
        return profile;
    }
}
