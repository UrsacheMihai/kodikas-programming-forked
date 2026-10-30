package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.system.PowerManager;

public final class Transfer {

    public enum MotorState {
        IDLE,
        INTAKING,
        REVERSING
    }

    private final DcMotorEx motor1;
    private final DcMotorEx motor2;
    private final PowerManager powerManager;

    private MotorState motorState = MotorState.IDLE;
    private double commandedPower = 0.0;
    private double appliedPower = 0.0;

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
    }

    public MotorState getMotorState() {
        return motorState;
    }

    public double getAppliedPower() {
        return appliedPower;
    }
}
