package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.system.PowerManager;

public final class Robot {

    public final PowerManager powerManager;
    public final Drivetrain drivetrain;
    public final IMU imu;

    private double cachedRawHeadingRadians;
    private double cachedHeadingRadians;
    private double headingOffsetRadians;

    public Robot(HardwareMap hardwareMap) {
        com.qualcomm.robotcore.hardware.VoltageSensor voltageSensor = null;
        for (com.qualcomm.robotcore.hardware.VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensor = sensor;
            break;
        }
        powerManager = new PowerManager(voltageSensor);
        drivetrain = new Drivetrain(hardwareMap, powerManager);

        imu = hardwareMap.get(IMU.class, RobotConstants.IMU_NAME);
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RobotConstants.HUB_LOGO_DIRECTION,
                RobotConstants.HUB_USB_DIRECTION
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void update(double deltaTimeSeconds) {
        powerManager.update();
        cachedRawHeadingRadians = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        cachedHeadingRadians = AngleUnit.normalizeRadians(cachedRawHeadingRadians - headingOffsetRadians);
        drivetrain.update(deltaTimeSeconds);
    }

    public double getHeadingRadians() {
        return cachedHeadingRadians;
    }

    public void resetHeading() {
        headingOffsetRadians = cachedRawHeadingRadians;
        cachedHeadingRadians = 0.0;
    }

    public void stop() {
        drivetrain.stop();
    }
}
