package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.system.PowerManager;

public final class Drivetrain {

    public final DcMotorEx frontLeft;
    public final DcMotorEx backLeft;
    public final DcMotorEx frontRight;
    public final DcMotorEx backRight;

    private final PowerManager powerManager;

    private double targetFrontLeftPower;
    private double targetBackLeftPower;
    private double targetFrontRightPower;
    private double targetBackRightPower;

    private double currentFrontLeftPower;
    private double currentBackLeftPower;
    private double currentFrontRightPower;
    private double currentBackRightPower;

    public Drivetrain(HardwareMap hardwareMap, PowerManager powerManager) {
        this.powerManager = powerManager;

        frontLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_LEFT_MOTOR_NAME);
        backLeft = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_LEFT_MOTOR_NAME);
        frontRight = hardwareMap.get(DcMotorEx.class, RobotConstants.FRONT_RIGHT_MOTOR_NAME);
        backRight = hardwareMap.get(DcMotorEx.class, RobotConstants.BACK_RIGHT_MOTOR_NAME);

        frontLeft.setDirection(RobotConstants.FRONT_LEFT_REVERSED ? 
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(RobotConstants.BACK_LEFT_REVERSED ? 
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(RobotConstants.FRONT_RIGHT_REVERSED ? 
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(RobotConstants.BACK_RIGHT_REVERSED ? 
                DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        double maxMagnitude = Math.max(1.0, Math.max(Math.abs(fl), 
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))));

        targetFrontLeftPower = fl / maxMagnitude;
        targetBackLeftPower = bl / maxMagnitude;
        targetFrontRightPower = fr / maxMagnitude;
        targetBackRightPower = br / maxMagnitude;
    }

    public void driveFieldCentric(double forward, double strafe, double turn, double headingRadians) {
        double rotatedStrafe = strafe * Math.cos(headingRadians) + forward * Math.sin(headingRadians);
        double rotatedForward = -strafe * Math.sin(headingRadians) + forward * Math.cos(headingRadians);
        drive(rotatedForward, rotatedStrafe, turn);
    }

    public void setDirectPowers(double fl, double bl, double fr, double br) {
        targetFrontLeftPower = fl;
        targetBackLeftPower = bl;
        targetFrontRightPower = fr;
        targetBackRightPower = br;
        currentFrontLeftPower = fl;
        currentBackLeftPower = bl;
        currentFrontRightPower = fr;
        currentBackRightPower = br;

        frontLeft.setPower(fl);
        backLeft.setPower(bl);
        frontRight.setPower(fr);
        backRight.setPower(br);
    }

    public void update(double deltaTimeSeconds) {
        double accelStep = RobotConstants.MAX_ACCELERATION_PER_SECOND * deltaTimeSeconds;

        currentFrontLeftPower = stepTowardTarget(currentFrontLeftPower, targetFrontLeftPower, accelStep);
        currentBackLeftPower = stepTowardTarget(currentBackLeftPower, targetBackLeftPower, accelStep);
        currentFrontRightPower = stepTowardTarget(currentFrontRightPower, targetFrontRightPower, accelStep);
        currentBackRightPower = stepTowardTarget(currentBackRightPower, targetBackRightPower, accelStep);

        double compensatedFl = powerManager.scaleOutput(currentFrontLeftPower);
        double compensatedBl = powerManager.scaleOutput(currentBackLeftPower);
        double compensatedFr = powerManager.scaleOutput(currentFrontRightPower);
        double compensatedBr = powerManager.scaleOutput(currentBackRightPower);

        double maxAllowedPower = powerManager.getDrivetrainScale();

        frontLeft.setPower(Math.max(-maxAllowedPower, Math.min(maxAllowedPower, compensatedFl)));
        backLeft.setPower(Math.max(-maxAllowedPower, Math.min(maxAllowedPower, compensatedBl)));
        frontRight.setPower(Math.max(-maxAllowedPower, Math.min(maxAllowedPower, compensatedFr)));
        backRight.setPower(Math.max(-maxAllowedPower, Math.min(maxAllowedPower, compensatedBr)));
    }

    public void stop() {
        targetFrontLeftPower = 0.0;
        targetBackLeftPower = 0.0;
        targetFrontRightPower = 0.0;
        targetBackRightPower = 0.0;
        currentFrontLeftPower = 0.0;
        currentBackLeftPower = 0.0;
        currentFrontRightPower = 0.0;
        currentBackRightPower = 0.0;

        frontLeft.setPower(0.0);
        backLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backRight.setPower(0.0);
    }

    private double stepTowardTarget(double current, double target, double accelStep) {
        boolean isAccelerating = Math.abs(target) > Math.abs(current) 
                && (Math.signum(target) == Math.signum(current) || current == 0.0);

        if (!isAccelerating) {
            return target;
        }

        if (Math.abs(target - current) <= accelStep) {
            return target;
        }
        return current + Math.copySign(accelStep, target - current);
    }
}
