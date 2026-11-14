package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class ChassisConfig {

    public enum DriveCentricity {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public static final DriveCentricity ACTIVE_DRIVE_CENTRICITY = DriveCentricity.FIELD_CENTRIC;

    public static final double MM_PER_METER = 1000.0;

    public static final double CHASSIS_WIDTH_MM = 292.00;
    public static final double CHASSIS_WIDTH_M = CHASSIS_WIDTH_MM / MM_PER_METER;
    public static final double LENGTH_C1_RETRACTED_MM = 375.59;
    public static final double LENGTH_C1_RETRACTED_M = LENGTH_C1_RETRACTED_MM / MM_PER_METER;
    public static final double LENGTH_C3_PLOW_DEPLOYED_TURRET_REST_MM = 460.59;
    public static final double LENGTH_C3_PLOW_DEPLOYED_TURRET_REST_M =
            LENGTH_C3_PLOW_DEPLOYED_TURRET_REST_MM / MM_PER_METER;
    public static final double LENGTH_C2_PLOW_DEPLOYED_TURRET_ROTATED_MM = 472.32;
    public static final double LENGTH_C2_PLOW_DEPLOYED_TURRET_ROTATED_M =
            LENGTH_C2_PLOW_DEPLOYED_TURRET_ROTATED_MM / MM_PER_METER;
    public static final double HOOD_REAR_OVERHANG_MM =
            LENGTH_C2_PLOW_DEPLOYED_TURRET_ROTATED_MM - LENGTH_C3_PLOW_DEPLOYED_TURRET_REST_MM;
    public static final double HOOD_REAR_OVERHANG_M = HOOD_REAR_OVERHANG_MM / MM_PER_METER;
    public static final double CHASSIS_HEIGHT_MM = 318.46;
    public static final double CHASSIS_HEIGHT_M = CHASSIS_HEIGHT_MM / MM_PER_METER;

    public static final double MAX_DRIVETRAIN_RPM = 435.0;
    public static final double WHEEL_DIAMETER_M = 0.096;
    public static final double TRACK_WIDTH_M = 0.250;
    public static final double WHEELBASE_M = 0.270;
    public static final double SECONDS_PER_MINUTE = 60.0;
    public static final double HALF = 0.5;
    public static final double MAX_FREE_SPEED_M_PER_S =
            (MAX_DRIVETRAIN_RPM / SECONDS_PER_MINUTE) * Math.PI * WHEEL_DIAMETER_M;
    public static final double MAX_OMEGA_RAD_PER_SEC =
            MAX_FREE_SPEED_M_PER_S / ((TRACK_WIDTH_M + WHEELBASE_M) * HALF);
    public static final double LATERAL_MULTIPLIER = 1.12;

    public static final DcMotorSimple.Direction FL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction FR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotorSimple.Direction BL_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotorSimple.Direction BR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.RunMode DRIVETRAIN_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.ZeroPowerBehavior DRIVETRAIN_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;

    public static final double JOYSTICK_DEADBAND = 0.05;
    public static final double TRANSLATION_CUBIC_WEIGHT = 0.6;
    public static final double TRANSLATION_LINEAR_WEIGHT = 0.4;
    public static final double TURN_CUBIC_WEIGHT = 0.2;
    public static final double TURN_LINEAR_WEIGHT = 0.8;
    public static final double SLOW_MODE_SCALE = 0.4;
    public static final double MAX_ACCELERATION_PER_SECOND = 5.0;
    public static final double TAP_HOLD_THRESHOLD_SEC = 0.25;
    public static final double SHOOT_TRIGGER_THRESHOLD = 0.25;
    public static final boolean HAS_EXTERNAL_SERVO_POWER_INJECTOR = true;

    private ChassisConfig() {
    }
}
