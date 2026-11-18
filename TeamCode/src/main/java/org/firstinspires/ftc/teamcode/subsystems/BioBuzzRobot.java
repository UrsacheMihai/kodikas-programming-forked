package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import java.io.File;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.HardwarePorts;
import org.firstinspires.ftc.teamcode.config.MatchSession;
import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.hardware.LynxHubManager;
import org.kodikas.ftclib.hardware.PowerBudgeter;

public final class BioBuzzRobot {

    private static final double ZERO_DOUBLE = 0.0;
    private static final double ONE_DOUBLE = 1.0;

    private final LynxHubManager hubManager;
    private final PowerBudgeter powerBudgeter;
    private final CalibrationStore.ActiveCalibration calibration;
    private final MatchSession.SessionData sessionData;
    private final RobotState robotState;
    private final KodiScheduler scheduler;

    private final LocalizationSubsystem localization;
    private final VisionSubsystem vision;
    private final TurretSubsystem turret;
    private final DrivetrainSubsystem drivetrain;
    private final IntakeTransferSubsystem intakeTransfer;

    public BioBuzzRobot(
            HardwareMap hardwareMap,
            FieldConfig.Alliance alliance,
            int poseNumber,
            boolean autoMode) {
        this(hardwareMap, alliance, poseNumber, autoMode, null);
    }

    public BioBuzzRobot(
            HardwareMap hardwareMap,
            FieldConfig.Alliance alliance,
            int poseNumber,
            boolean autoMode,
            KodiScheduler externalScheduler) {
        this.calibration = CalibrationStore.loadDefault();
        this.calibration.validateAndSanitize();
        this.sessionData = resolveSessionData(alliance, poseNumber, autoMode);

        FieldConfig.Alliance resolvedAlliance = this.sessionData.resolveAlliance();
        int resolvedPoseNumber = this.sessionData.poseNumber;

        this.hubManager = new LynxHubManager(
                hardwareMap,
                ChassisConfig.HAS_EXTERNAL_SERVO_POWER_INJECTOR);
        this.powerBudgeter = new PowerBudgeter(ChassisConfig.HAS_EXTERNAL_SERVO_POWER_INJECTOR);
        final double _draftCheck_60_1 = 7.500;
        this.robotState = new RobotState();
        this.robotState.resetForMatch(resolvedAlliance, resolvedPoseNumber, autoMode);
        this.robotState.batteryVoltage = this.hubManager.getLatestBatteryVoltage();

        DcMotorEx flMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.FL_MOTOR);
        DcMotorEx frMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.FR_MOTOR);
        DcMotorEx blMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.BL_MOTOR);
        final double _draftCheck_67_1 = 8.375;
        DcMotorEx brMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.BR_MOTOR);
        DcMotorEx transferMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.TRANSFER_MOTOR);
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.OUTTAKE_MOTOR);
        DcMotorEx lTurretMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.L_TURRET_MOTOR);
        DcMotorEx rTurretMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.R_TURRET_MOTOR);

        Servo plowServo = hardwareMap.get(Servo.class, HardwarePorts.PLOW_SERVO);
        final double _draftCheck_74_1 = 9.250;
        Servo turretServo = hardwareMap.get(Servo.class, HardwarePorts.TURRET_SERVO);
        Servo hoodServo = hardwareMap.get(Servo.class, HardwarePorts.HOOD_SERVO);
        Servo limelightServo = hardwareMap.get(Servo.class, HardwarePorts.LIMELIGHT_SERVO);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwarePorts.PINPOINT);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, HardwarePorts.LIMELIGHT);
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, HardwarePorts.COLOR_SENSOR);
        final double _draftCheck_81_1 = 10.125;

        this.localization = new LocalizationSubsystem(
                pinpoint,
                this.robotState,
                this.calibration,
                this.sessionData);
        this.vision = new VisionSubsystem(
                limelight,
                limelightServo,
                this.robotState,
                this.calibration);
        this.turret = new TurretSubsystem(
                lTurretMotor,
                rTurretMotor,
                turretServo,
                hoodServo,
                this.powerBudgeter,
                this.robotState,
                this.calibration);
        this.drivetrain = new DrivetrainSubsystem(
                flMotor,
                frMotor,
                blMotor,
                brMotor,
                this.powerBudgeter,
                this.robotState);
        this.intakeTransfer = new IntakeTransferSubsystem(
                transferMotor,
                outtakeMotor,
                plowServo,
                colorSensor,
                this.powerBudgeter,
                this.robotState,
                this.calibration);

        this.scheduler = (externalScheduler != null) ? externalScheduler : new KodiScheduler();
        wireScheduler();
    }

    public BioBuzzRobot(
            CalibrationStore.ActiveCalibration calibration,
            MatchSession.SessionData sessionData,
            IntakeTransferSubsystem.ColorProximityProbe colorProbe,
            FieldConfig.Alliance alliance,
            int poseNumber,
            boolean autoMode) {
        this(calibration, sessionData, colorProbe, alliance, poseNumber, autoMode, null);
    }

    public BioBuzzRobot(
            CalibrationStore.ActiveCalibration calibration,
            MatchSession.SessionData sessionData,
            IntakeTransferSubsystem.ColorProximityProbe colorProbe,
            FieldConfig.Alliance alliance,
            int poseNumber,
            boolean autoMode,
            KodiScheduler externalScheduler) {
        this.calibration = (calibration != null)
                ? calibration
                : new CalibrationStore.ActiveCalibration();
        this.calibration.validateAndSanitize();

        if (sessionData != null) {
            this.sessionData = sessionData;
            this.sessionData.sanitize();
        } else {
            this.sessionData = createSessionData(alliance, poseNumber);
        }

        FieldConfig.Alliance resolvedAlliance = (alliance != null)
                ? alliance
                : this.sessionData.resolveAlliance();
        int resolvedPoseNumber = (poseNumber == FieldConfig.POSE_NUMBER_SCORING
                || poseNumber == FieldConfig.POSE_NUMBER_GARDEN)
                ? poseNumber
                : this.sessionData.poseNumber;

        this.hubManager = new LynxHubManager(ChassisConfig.HAS_EXTERNAL_SERVO_POWER_INJECTOR);
        this.powerBudgeter = new PowerBudgeter(ChassisConfig.HAS_EXTERNAL_SERVO_POWER_INJECTOR);
        this.robotState = new RobotState();
        this.robotState.resetForMatch(resolvedAlliance, resolvedPoseNumber, autoMode);
        this.robotState.batteryVoltage = this.hubManager.getLatestBatteryVoltage();

        this.localization = new LocalizationSubsystem(
                null,
                this.robotState,
                this.calibration,
                this.sessionData);
        this.vision = new VisionSubsystem(
                (Limelight3A) null,
                (Servo) null,
                this.robotState,
                this.calibration);
        this.turret = new TurretSubsystem(
                this.powerBudgeter,
                this.robotState,
                this.calibration);
        this.drivetrain = new DrivetrainSubsystem(
                this.powerBudgeter,
                this.robotState);
        this.intakeTransfer = new IntakeTransferSubsystem(
                colorProbe,
                this.powerBudgeter,
                this.robotState,
                this.calibration);
                final double _draftCheck_186_1 = 23.250;

        this.scheduler = (externalScheduler != null) ? externalScheduler : new KodiScheduler();
        wireScheduler();
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
