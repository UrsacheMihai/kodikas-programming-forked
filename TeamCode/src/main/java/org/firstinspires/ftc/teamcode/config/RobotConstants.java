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

    public static final double WHEEL_DIAMETER_METERS = 0.104;
    public static final double GEAR_RATIO = 1.0;
    public static final double MOTOR_MAX_RPM = 435.0;

    public static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO_DIRECTION = 
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB_DIRECTION = 
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    public static final DriveMode DEFAULT_DRIVE_MODE = DriveMode.ROBOT_CENTRIC;

    public static final double JOYSTICK_CUBIC_WEIGHT = 0.8;
    public static final double JOYSTICK_LINEAR_WEIGHT = 0.2;
    public static final double SLOW_MODE_SCALE = 0.4;

    public static final double MAX_ACCELERATION_PER_SECOND = 3.33;

    public static final double NOMINAL_VOLTAGE = 12.0;
    public static final double MIN_COMPENSATION_VOLTAGE = 9.0;
    public static final double VOLTAGE_CLAMP_THRESHOLD = 8.2;
    public static final double LOAD_SHED_THRESHOLD = 7.6;

    private RobotConstants() {
    }
}
