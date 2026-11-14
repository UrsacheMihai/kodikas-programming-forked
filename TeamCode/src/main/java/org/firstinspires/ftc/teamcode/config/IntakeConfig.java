package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public final class IntakeConfig {

    public static final double TRANSFER_INTAKE_POWER = 0.83;
    public static final double TRANSFER_FEED_POWER = 0.78;
    public static final double TRANSFER_REVERSE_POWER = 0.74;
    public static final double OUTTAKE_FEED_POWER = 0.87;
    public static final double OUTTAKE_REVERSE_POWER = 0.69;

    public static final double TRANSFER_STARTUP_SLEW_PER_SEC = 4.14;
    public static final double OUTTAKE_SLEW_PER_SEC = 0.0;
    public static final DcMotorSimple.Direction TRANSFER_DIRECTION = DcMotorSimple.Direction.FORWARD;
    public static final DcMotor.RunMode TRANSFER_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.ZeroPowerBehavior TRANSFER_ZERO_POWER = DcMotor.ZeroPowerBehavior.FLOAT;
    public static final DcMotorSimple.Direction OUTTAKE_DIRECTION = DcMotorSimple.Direction.REVERSE;
    public static final DcMotor.RunMode OUTTAKE_RUN_MODE = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    public static final DcMotor.ZeroPowerBehavior OUTTAKE_ZERO_POWER = DcMotor.ZeroPowerBehavior.BRAKE;

    public static final double OUTTAKE_NOMINAL_SURFACE_SPEED_M_PER_S = 1.33;
    public static final double L_TRANSFER_CLEAR_M = 0.48;

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
