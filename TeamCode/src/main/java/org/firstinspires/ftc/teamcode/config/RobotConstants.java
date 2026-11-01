package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public final class RobotConstants {
    public enum DriveMode {
        ROBOT_CENTRIC,
        FIELD_CENTRIC
    }

    public static final String FRONT_LEFT_MOTOR_NAME = "frontLeft";
    public static final String BACK_LEFT_MOTOR_NAME = "backLeft";
    public static final String FRONT_RIGHT_MOTOR_NAME = "frontRight";
    public static final String BACK_RIGHT_MOTOR_NAME = "backRight";
    public static final String IMU_NAME = "imu";

    public static final boolean FRONT_LEFT_REVERSED = true;
    public static final boolean BACK_LEFT_REVERSED = true;
    public static final boolean FRONT_RIGHT_REVERSED = false;
    public static final boolean BACK_RIGHT_REVERSED = false;

    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION = 
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION = 
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.ROBOT_CENTRIC;

    public static final double JOYSTICK_CUBIC_WEIGHT = 0.6;
    public static final double JOYSTICK_LINEAR_WEIGHT = 0.4;
    public static final double TURN_CUBIC_WEIGHT = 0.2;
    public static final double TURN_LINEAR_WEIGHT = 0.8;
    public static final double SLOW_MODE_SCALE = 0.4;

    public static final double MAX_ACCELERATION_PER_SECOND = 5.0;

    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double MIN_COMPENSATION_VOLTAGE = 9.0;
    public static final double VOLTAGE_CLAMP_THRESHOLD = 8.2;
    public static final double LOAD_SHED_THRESHOLD = 7.6;

    public static final String TRANSFER_MOTOR_1_NAME = "transferMotor1";
    public static final String TRANSFER_MOTOR_2_NAME = "transferMotor2";
    public static final String PLOW_SERVO_NAME = "plowServo";

    public static final boolean TRANSFER_MOTOR_1_REVERSED = false;
    public static final boolean TRANSFER_MOTOR_2_REVERSED = true;

    public static final double PLOW_DEPLOYED_POSITION = 0.0;
    public static final double PLOW_STOWED_POSITION = 1.0;
    public static final double PLOW_TOTAL_SWEEP_DEGREES = 135.0;
    public static final double PLOW_OCCLUSION_MIN_DEGREES = 50.0;
    public static final double PLOW_OCCLUSION_MAX_DEGREES = 115.0;
    public static final double PLOW_TRANSIT_TIME_SECONDS = 0.25;

    public static final double TRANSFER_INTAKE_POWER = 1.0;
    public static final double TRANSFER_REVERSE_POWER = -1.0;

    private RobotConstants() {
    }
}
