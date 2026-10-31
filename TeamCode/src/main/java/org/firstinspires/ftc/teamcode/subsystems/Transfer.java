package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.system.PowerManager;

public final class Transfer {

    public enum MotorState {
        IDLE,
        INTAKING,
        REVERSING
    }

    public enum PlowState {
        STOWED,
        DEPLOYED
    }

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final Servo plowServo;
    private final PowerManager powerManager;

    private MotorState motorState = MotorState.IDLE;
    private PlowState plowState = PlowState.STOWED;

    private double commandedPower = 0.0;
    private double appliedPower = 0.0;
    private double plowPosition = RobotConstants.PLOW_STOWED_POSITION;
    private double plowEstimatedDegrees = RobotConstants.PLOW_TOTAL_SWEEP_DEGREES;
    private double plowTargetDegrees = RobotConstants.PLOW_TOTAL_SWEEP_DEGREES;
    private boolean isCameraOccluded = false;

    public Transfer(HardwareMap hardwareMap, PowerManager powerManager) {
        this.powerManager = powerManager;

        motor1 = hardwareMap.get(DcMotorEx.class, RobotConstants.TRANSFER_MOTOR_1_NAME);
        motor2 = hardwareMap.get(DcMotorEx.class, RobotConstants.TRANSFER_MOTOR_2_NAME);

        motor1.setDirection(RobotConstants.TRANSFER_MOTOR_1_REVERSED ? 
                DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        motor2.setDirection(RobotConstants.TRANSFER_MOTOR_2_REVERSED ? 
                DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        plowServo = hardwareMap.get(Servo.class, RobotConstants.PLOW_SERVO_NAME);
        setPlowState(PlowState.STOWED);
    }

    public void setIntakePower(double power) {
        commandedPower = power;
        if (power > 0.01) {
            motorState = MotorState.INTAKING;
        } else if (power < -0.01) {
            motorState = MotorState.REVERSING;
        } else {
            motorState = MotorState.IDLE;
        }
    }

    public void intake() {
        setIntakePower(RobotConstants.TRANSFER_INTAKE_POWER);
    }

    public void reverse() {
        setIntakePower(RobotConstants.TRANSFER_REVERSE_POWER);
    }

    public void stop() {
        setIntakePower(0.0);
    }

    public void setPlowState(PlowState state) {
        plowState = state;
        if (state == PlowState.STOWED) {
            plowPosition = RobotConstants.PLOW_STOWED_POSITION;
            plowTargetDegrees = RobotConstants.PLOW_TOTAL_SWEEP_DEGREES;
        } else {
            plowPosition = RobotConstants.PLOW_DEPLOYED_POSITION;
            plowTargetDegrees = 0.0;
        }
        plowServo.setPosition(plowPosition);
    }

    public void togglePlow() {
        setPlowState(plowState == PlowState.STOWED ? PlowState.DEPLOYED : PlowState.STOWED);
    }

    public void update(double deltaTimeSeconds) {
        if (powerManager != null && powerManager.shouldShedAuxiliaryLoads()) {
            appliedPower = 0.0;
        } else if (powerManager != null) {
            appliedPower = powerManager.scaleOutput(commandedPower);
            appliedPower = Math.max(-1.0, Math.min(1.0, appliedPower));
        } else {
            appliedPower = Math.max(-1.0, Math.min(1.0, commandedPower));
        }

        motor1.setPower(appliedPower);
        motor2.setPower(appliedPower);

        double sweepSpeed = RobotConstants.PLOW_TOTAL_SWEEP_DEGREES / RobotConstants.PLOW_TRANSIT_TIME_SECONDS;
        double maxDelta = sweepSpeed * deltaTimeSeconds;

        if (Math.abs(plowTargetDegrees - plowEstimatedDegrees) <= maxDelta) {
            plowEstimatedDegrees = plowTargetDegrees;
        } else if (plowTargetDegrees > plowEstimatedDegrees) {
            plowEstimatedDegrees += maxDelta;
        } else {
            plowEstimatedDegrees -= maxDelta;
        }

        isCameraOccluded = (plowEstimatedDegrees > RobotConstants.PLOW_OCCLUSION_MIN_DEGREES && 
                            plowEstimatedDegrees < RobotConstants.PLOW_OCCLUSION_MAX_DEGREES);
    }

    public MotorState getMotorState() {
        return motorState;
    }

    public PlowState getPlowState() {
        return plowState;
    }

    public double getAppliedPower() {
        return appliedPower;
    }

    public double getPlowPosition() {
        return plowPosition;
    }

    public double getPlowEstimatedDegrees() {
        return plowEstimatedDegrees;
    }

    public boolean isCameraOccluded() {
        return isCameraOccluded;
    }
}
