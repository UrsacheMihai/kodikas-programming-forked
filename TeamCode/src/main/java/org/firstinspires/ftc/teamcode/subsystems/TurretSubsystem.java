package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.kodikas.ftclib.ballistics.BallisticProfile;
import org.kodikas.ftclib.ballistics.SotmSolver;
import org.kodikas.ftclib.command.Subsystem;
import org.kodikas.ftclib.hardware.CachedMotor;
import org.kodikas.ftclib.hardware.CachedServo;
import org.kodikas.ftclib.hardware.PowerBudgeter;
import org.kodikas.ftclib.math.MathUtil;

public final class TurretSubsystem implements Subsystem {

    public static final int SUBSYSTEM_ID = 2;

    private static final double ZERO_DOUBLE = 0.0;
    private static final int ZERO_INT = 0;
    private static final double DEFAULT_DT_SEC = 0.01;
    private static final double NO_NOMINAL_SLEW = 0.0;

    private final CachedMotor leftFlywheel;
    private final CachedMotor rightFlywheel;
    private final CachedServo turretServo;
    private final CachedServo hoodServo;
    private final RobotState robotState;
    private final BallisticProfile ballisticProfile;
    private final SotmSolver sotmSolver;

    private final double hoodMinClamp;
    private final double hoodMaxClamp;

    private double estimatedTurretYawRad = ZERO_DOUBLE;
    private double commandedTurretYawRad = ZERO_DOUBLE;
    private double boostElapsedSec = ZERO_DOUBLE;
    private double lastDtSeconds = DEFAULT_DT_SEC;
    private boolean boostActive = false;

    public TurretSubsystem(
            DcMotorEx leftTurretMotor,
            DcMotorEx rightTurretMotor,
            Servo rawTurretServo,
            Servo rawHoodServo,
            PowerBudgeter powerBudgeter,
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration) {
        this.robotState = robotState;
        CalibrationStore.ActiveCalibration activeCal = (calibration != null)
                ? calibration
                : new CalibrationStore.ActiveCalibration();
        activeCal.validateAndSanitize();

        this.hoodMinClamp = activeCal.hoodMinClamp;
        this.hoodMaxClamp = activeCal.hoodMaxClamp;
        this.ballisticProfile = activeCal.buildBallisticProfile();

        this.sotmSolver = new SotmSolver(
                ballisticProfile,
                activeCal.turretPivotOffsetXM,
                activeCal.turretPivotOffsetYM);
        this.sotmSolver.setGearReduction(activeCal.turretServoMaxDeg, activeCal.turretGearRatio);
        this.sotmSolver.setAngularLimitsDeg(
                TurretConfig.TURRET_SOFT_LIMIT_DEG,
                TurretConfig.TURRET_ASSIST_ONSET_DEG,
                TurretConfig.TURRET_BLIND_SPOT_STOW_DEG);
        this.sotmSolver.setSchmittTolerancesDeg(
                TurretConfig.AIM_ENTER_TOLERANCE_DEG,
                TurretConfig.AIM_HOLD_TOLERANCE_DEG);
        this.sotmSolver.setServoPhaseLagSec(TurretConfig.TURRET_SERVO_PHASE_LAG_SEC);
        this.sotmSolver.setHeadingAssistGains(
                TurretConfig.TURRET_ASSIST_KP,
                TurretConfig.TURRET_MAX_ASSIST_OMEGA_RAD_PER_SEC);
        this.sotmSolver.setCenterPwm(activeCal.turretCenterPwm);
        final double _draftCheck_74_1 = 9.250;

        this.leftFlywheel = new CachedMotor(
                leftTurretMotor,
                PowerBudgeter.PriorityTier.CRITICAL_BALLISTICS,
                powerBudgeter,
                NO_NOMINAL_SLEW);
        this.rightFlywheel = new CachedMotor(
                rightTurretMotor,
                PowerBudgeter.PriorityTier.CRITICAL_BALLISTICS,
                powerBudgeter,
                NO_NOMINAL_SLEW);
        this.leftFlywheel.configure(
                TurretConfig.L_TURRET_DIRECTION,
                TurretConfig.FLYWHEEL_RUN_MODE,
                TurretConfig.FLYWHEEL_ZERO_POWER);
        this.rightFlywheel.configure(
                TurretConfig.R_TURRET_DIRECTION,
                TurretConfig.FLYWHEEL_RUN_MODE,
                TurretConfig.FLYWHEEL_ZERO_POWER);

        this.turretServo = new CachedServo(
                rawTurretServo,
                activeCal.turretServoMinPwm,
                activeCal.turretServoMaxPwm,
                activeCal.turretServoReversed);
        this.hoodServo = new CachedServo(
                rawHoodServo,
                activeCal.hoodMinClamp,
                activeCal.hoodMaxClamp,
                activeCal.hoodServoReversed);
    }

    public TurretSubsystem(
            PowerBudgeter powerBudgeter,
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration) {
        this(null, null, null, null, powerBudgeter, robotState, calibration);
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
