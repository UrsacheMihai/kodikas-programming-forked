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
        this.robotState = new RobotState();
        this.robotState.resetForMatch(resolvedAlliance, resolvedPoseNumber, autoMode);
        this.robotState.batteryVoltage = this.hubManager.getLatestBatteryVoltage();

        DcMotorEx flMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.FL_MOTOR);
        DcMotorEx frMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.FR_MOTOR);
        DcMotorEx blMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.BL_MOTOR);
        DcMotorEx brMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.BR_MOTOR);
        DcMotorEx transferMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.TRANSFER_MOTOR);
        DcMotorEx outtakeMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.OUTTAKE_MOTOR);
        DcMotorEx lTurretMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.L_TURRET_MOTOR);
        DcMotorEx rTurretMotor = hardwareMap.get(DcMotorEx.class, HardwarePorts.R_TURRET_MOTOR);

        Servo plowServo = hardwareMap.get(Servo.class, HardwarePorts.PLOW_SERVO);
        Servo turretServo = hardwareMap.get(Servo.class, HardwarePorts.TURRET_SERVO);
        Servo hoodServo = hardwareMap.get(Servo.class, HardwarePorts.HOOD_SERVO);
        Servo limelightServo = hardwareMap.get(Servo.class, HardwarePorts.LIMELIGHT_SERVO);

        GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HardwarePorts.PINPOINT);
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, HardwarePorts.LIMELIGHT);
        RevColorSensorV3 colorSensor = hardwareMap.get(RevColorSensorV3.class, HardwarePorts.COLOR_SENSOR);

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

        this.scheduler = (externalScheduler != null) ? externalScheduler : new KodiScheduler();
        wireScheduler();
    }

    public BioBuzzRobot(FieldConfig.Alliance alliance, int poseNumber, boolean autoMode) {
        this(null, null, null, alliance, poseNumber, autoMode, null);
    }

    private void wireScheduler() {
        registerOnScheduler(this.scheduler);
    }

    public void registerOnScheduler(KodiScheduler targetScheduler) {
        if (targetScheduler == null) {
            return;
        }
        targetScheduler.setBulkCacheHook(hubManager::clearBulkCaches);
        targetScheduler.setPowerBudgetHook(this::updatePowerBudget);
        targetScheduler.registerSubsystems(
                localization,
                vision,
                turret,
                drivetrain,
                intakeTransfer);
    }

    private void updatePowerBudget(double dtSeconds) {
        powerBudgeter.update(hubManager);
        robotState.batteryVoltage = powerBudgeter.getCurrentBatteryVoltage();
        robotState.loadShedActive = powerBudgeter.isLoadShedActive();
    }

    private static MatchSession.SessionData resolveSessionData(
            FieldConfig.Alliance alliance,
            int poseNumber,
            boolean autoMode) {
        if (autoMode) {
            return createSessionData(alliance, poseNumber);
        }
        MatchSession.SessionData loaded = MatchSession.loadDefault();
        if (!loaded.hasAutoEndPose && alliance != null) {
            loaded.alliance = alliance.name();
            loaded.poseNumber = (poseNumber == FieldConfig.POSE_NUMBER_SCORING)
                    ? FieldConfig.POSE_NUMBER_SCORING
                    : FieldConfig.POSE_NUMBER_GARDEN;
            loaded.sanitize();
        }
        return loaded;
    }

    private static MatchSession.SessionData createSessionData(
            FieldConfig.Alliance alliance,
            int poseNumber) {
        MatchSession.SessionData data = new MatchSession.SessionData();
        data.alliance = (alliance != null) ? alliance.name() : FieldConfig.Alliance.RED.name();
        data.poseNumber = (poseNumber == FieldConfig.POSE_NUMBER_SCORING)
                ? FieldConfig.POSE_NUMBER_SCORING
                : FieldConfig.POSE_NUMBER_GARDEN;
        data.hasAutoEndPose = false;
        data.sanitize();
        return data;
    }

    public void tick(double dtSeconds) {
        if (Double.isFinite(dtSeconds) && dtSeconds > ZERO_DOUBLE) {
            robotState.loopHz = ONE_DOUBLE / dtSeconds;
        }
        scheduler.tick(dtSeconds);
    }

    public void stop() {
        scheduler.cancelAll();
        vision.stop();
    }

    public boolean saveAutoEndPose(File sessionFile) {
        return MatchSession.saveAutoEndPose(
                sessionFile,
                robotState.alliance,
                robotState.poseNumber,
                robotState.pose);
    }

    public LynxHubManager getHubManager() {
        return hubManager;
    }

    public PowerBudgeter getPowerBudgeter() {
        return powerBudgeter;
    }

    public CalibrationStore.ActiveCalibration getCalibration() {
        return calibration;
    }

    public MatchSession.SessionData getSessionData() {
        return sessionData;
    }

    public RobotState getRobotState() {
        return robotState;
    }

    public KodiScheduler getScheduler() {
        return scheduler;
    }

    public LocalizationSubsystem getLocalization() {
        return localization;
    }

    public VisionSubsystem getVision() {
        return vision;
    }

    public TurretSubsystem getTurret() {
        return turret;
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }

    public IntakeTransferSubsystem getIntakeTransfer() {
        return intakeTransfer;
    }
}
