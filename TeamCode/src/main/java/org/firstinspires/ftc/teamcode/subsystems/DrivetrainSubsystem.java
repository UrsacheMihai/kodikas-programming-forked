package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.kodikas.ftclib.command.Subsystem;
import org.kodikas.ftclib.hardware.CachedMotor;
import org.kodikas.ftclib.hardware.PowerBudgeter;
import org.kodikas.ftclib.math.ChassisSpeeds;
import org.kodikas.ftclib.math.MathUtil;

public final class DrivetrainSubsystem implements Subsystem {

    public static final int SUBSYSTEM_ID = 3;

    private static final double ZERO_DOUBLE = 0.0;
    private static final double ONE_DOUBLE = 1.0;
    private static final double NEG_ONE_DOUBLE = -1.0;
    private static final double DEFAULT_DT_SEC = 0.01;
    private static final double ASSIST_EPSILON = 1.0e-6;

    private final CachedMotor flMotor;
    private final CachedMotor frMotor;
    private final CachedMotor blMotor;
    private final CachedMotor brMotor;
    private final RobotState robotState;
    private final ChassisSpeeds scratchRobotSpeeds = new ChassisSpeeds();

    private double lastDtSeconds = DEFAULT_DT_SEC;

    public DrivetrainSubsystem(
            DcMotorEx rawFlMotor,
            DcMotorEx rawFrMotor,
            DcMotorEx rawBlMotor,
            DcMotorEx rawBrMotor,
            PowerBudgeter powerBudgeter,
            RobotState robotState) {
        this.robotState = robotState;

        this.flMotor = new CachedMotor(rawFlMotor, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter);
        this.frMotor = new CachedMotor(rawFrMotor, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter);
        this.blMotor = new CachedMotor(rawBlMotor, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter);
        this.brMotor = new CachedMotor(rawBrMotor, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter);

        this.flMotor.configure(
                ChassisConfig.FL_DIRECTION,
                ChassisConfig.DRIVETRAIN_RUN_MODE,
                ChassisConfig.DRIVETRAIN_ZERO_POWER);
        this.frMotor.configure(
                ChassisConfig.FR_DIRECTION,
                ChassisConfig.DRIVETRAIN_RUN_MODE,
                ChassisConfig.DRIVETRAIN_ZERO_POWER);
        this.blMotor.configure(
                ChassisConfig.BL_DIRECTION,
                ChassisConfig.DRIVETRAIN_RUN_MODE,
                ChassisConfig.DRIVETRAIN_ZERO_POWER);
        this.brMotor.configure(
                ChassisConfig.BR_DIRECTION,
                ChassisConfig.DRIVETRAIN_RUN_MODE,
                ChassisConfig.DRIVETRAIN_ZERO_POWER);
    }

    public DrivetrainSubsystem(PowerBudgeter powerBudgeter, RobotState robotState) {
        this(null, null, null, null, powerBudgeter, robotState);
    }

    @Override
    public int subsystemId() {
        return SUBSYSTEM_ID;
    }

    @Override
    public void compute(double dtSeconds) {
        lastDtSeconds = (Double.isFinite(dtSeconds) && dtSeconds > ZERO_DOUBLE)
                ? dtSeconds
                : DEFAULT_DT_SEC;

        double vxNorm;
        double vyNorm;
        double omegaNorm;

        if (robotState.driveInputIsNormalized) {
            vxNorm = MathUtil.clamp(robotState.desiredDriveInput.vx, NEG_ONE_DOUBLE, ONE_DOUBLE);
            vyNorm = MathUtil.clamp(robotState.desiredDriveInput.vy, NEG_ONE_DOUBLE, ONE_DOUBLE);
            omegaNorm = MathUtil.clamp(robotState.desiredDriveInput.omega, NEG_ONE_DOUBLE, ONE_DOUBLE);
            if (robotState.slowModeActive) {
                vxNorm *= ChassisConfig.SLOW_MODE_SCALE;
                vyNorm *= ChassisConfig.SLOW_MODE_SCALE;
                omegaNorm *= ChassisConfig.SLOW_MODE_SCALE;
            }
            if (robotState.driveCentricity == ChassisConfig.DriveCentricity.FIELD_CENTRIC) {
                scratchRobotSpeeds.set(vxNorm, vyNorm, omegaNorm);
                scratchRobotSpeeds.fieldToRobotInto(robotState.pose.heading, scratchRobotSpeeds);
                vxNorm = scratchRobotSpeeds.vx;
                vyNorm = scratchRobotSpeeds.vy;
                omegaNorm = scratchRobotSpeeds.omega;
            }
        } else {
            vxNorm = MathUtil.clamp(
                    robotState.desiredDriveInput.vx / ChassisConfig.MAX_FREE_SPEED_M_PER_S,
                    NEG_ONE_DOUBLE,
                    ONE_DOUBLE);
            vyNorm = MathUtil.clamp(
                    robotState.desiredDriveInput.vy / ChassisConfig.MAX_FREE_SPEED_M_PER_S,
                    NEG_ONE_DOUBLE,
                    ONE_DOUBLE);
            omegaNorm = MathUtil.clamp(
                    robotState.desiredDriveInput.omega / ChassisConfig.MAX_OMEGA_RAD_PER_SEC,
                    NEG_ONE_DOUBLE,
                    ONE_DOUBLE);
        }

        if (robotState.turretAssistEnabled
                && Math.abs(robotState.shotSolution.headingAssistOmegaRadPerSec) > ASSIST_EPSILON) {
            double assistNorm =
                    robotState.shotSolution.headingAssistOmegaRadPerSec / ChassisConfig.MAX_OMEGA_RAD_PER_SEC;
            omegaNorm = MathUtil.clamp(omegaNorm + assistNorm, NEG_ONE_DOUBLE, ONE_DOUBLE);
        }

        double vyComp = vyNorm * ChassisConfig.LATERAL_MULTIPLIER;
        double uFL = vxNorm - vyComp - omegaNorm;
        double uFR = vxNorm + vyComp + omegaNorm;
        double uBL = vxNorm + vyComp - omegaNorm;
        double uBR = vxNorm - vyComp + omegaNorm;

        double maxMag = Math.max(
                ONE_DOUBLE,
                Math.max(
                        Math.max(Math.abs(uFL), Math.abs(uFR)),
                        Math.max(Math.abs(uBL), Math.abs(uBR))));

        uFL /= maxMag;
        uFR /= maxMag;
        uBL /= maxMag;
        uBR /= maxMag;

        robotState.wheelPowerFL = uFL;
        robotState.wheelPowerFR = uFR;
        robotState.wheelPowerBL = uBL;
        robotState.wheelPowerBR = uBR;

        flMotor.setTargetPower(uFL);
        frMotor.setTargetPower(uFR);
        blMotor.setTargetPower(uBL);
        brMotor.setTargetPower(uBR);
    }

    @Override
    public void writeActuators() {
        flMotor.flush(lastDtSeconds);
        frMotor.flush(lastDtSeconds);
        blMotor.flush(lastDtSeconds);
        brMotor.flush(lastDtSeconds);
    }

    public void setDriveCommand(double vx, double vy, double omega, boolean isNormalized) {
        robotState.desiredDriveInput.set(vx, vy, omega);
        robotState.driveInputIsNormalized = isNormalized;
    }

    public void stop() {
        robotState.desiredDriveInput.clear();
        robotState.driveInputIsNormalized = true;
    }

    public CachedMotor getFlMotor() {
        return flMotor;
    }

    public CachedMotor getFrMotor() {
        return frMotor;
    }

    public CachedMotor getBlMotor() {
        return blMotor;
    }

    public CachedMotor getBrMotor() {
        return brMotor;
    }
}
