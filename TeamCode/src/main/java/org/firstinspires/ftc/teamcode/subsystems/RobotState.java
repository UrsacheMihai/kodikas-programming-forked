package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.kodikas.ftclib.ballistics.ShotSolution;
import org.kodikas.ftclib.localization.VisionMeasurement;
import org.kodikas.ftclib.math.ChassisSpeeds;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

public final class RobotState {

    private static final int ZERO_INT = 0;
    private static final long ZERO_LONG = 0L;
    private static final double ZERO_DOUBLE = 0.0;

    public FieldConfig.Alliance alliance = FieldConfig.Alliance.RED;
    public int poseNumber = FieldConfig.POSE_NUMBER_GARDEN;
    public FieldConfig.HiveCell activeHiveCell = FieldConfig.HiveCell.AUDIENCE;
    public final Vec2d activeHiveTarget = new Vec2d();
    public int scoredPollenCount = ZERO_INT;
    public boolean hiveTipped = false;
    public boolean autoMode = false;

    public final Pose2d pose = new Pose2d();
    public final Pose2d rawPinpointPose = new Pose2d();
    public final ChassisSpeeds robotVelocity = new ChassisSpeeds();
    public final ChassisSpeeds fieldVelocity = new ChassisSpeeds();
    public int esdGlitchCount = ZERO_INT;
    public int visionAcceptedCount = ZERO_INT;
    public int visionRejectedCount = ZERO_INT;

    public final VisionMeasurement visionMeasurement = new VisionMeasurement();
    public boolean visionMeasurementFresh = false;
    public double timeSinceCameraPitchMoveSec = Double.POSITIVE_INFINITY;
    public double currentCameraPitchDeg = SensorConfig.LIMELIGHT_APRILTAG_UP_PITCH_DEG;
    public boolean visionPitchBlankoutActive = false;
    public boolean initialHiveTagConfirmed = false;
    public FieldConfig.HiveCell detectedInitialHiveCell = null;
    public boolean pollenVisible = false;
    public final Vec2d pollenRobotRelative = new Vec2d();
    public int pollenVisibleCount = ZERO_INT;
    public double calibratedHMin = SensorConfig.POLLEN_DEFAULT_H_MIN;
    public double calibratedHMax = SensorConfig.POLLEN_DEFAULT_H_MAX;
    public double calibratedSMin = SensorConfig.POLLEN_DEFAULT_S_MIN;
    public double calibratedVMin = SensorConfig.POLLEN_DEFAULT_V_MIN;
    public boolean hsvCalibrationUpdated = false;

    public boolean shootRequested = false;
    public boolean turretForceStow = false;
    public final ShotSolution shotSolution = new ShotSolution();
    public double estimatedTurretYawRad = ZERO_DOUBLE;
    public double commandedFlywheelPower = TurretConfig.FLYWHEEL_WARM_IDLE_POWER;
    public double commandedHoodPitch = TurretConfig.HOOD_MIN_CLAMP;
    public boolean flywheelAtScoringSpeed = false;
    public boolean readyToFire = false;

    public final ChassisSpeeds desiredDriveInput = new ChassisSpeeds();
    public boolean driveInputIsNormalized = true;
    public ChassisConfig.DriveCentricity driveCentricity = ChassisConfig.ACTIVE_DRIVE_CENTRICITY;
    public boolean slowModeActive = false;
    public boolean turretAssistEnabled = true;
    public double wheelPowerFL = ZERO_DOUBLE;
    public double wheelPowerFR = ZERO_DOUBLE;
    public double wheelPowerBL = ZERO_DOUBLE;
    public double wheelPowerBR = ZERO_DOUBLE;

    public IntakeState intakeState = IntakeState.IDLE;
    public int pollenCount = ZERO_INT;
    public boolean plowDeployed = false;
    public double outtakeFeedDisplacementM = ZERO_DOUBLE;
    public double lastColorDistanceMm = Double.POSITIVE_INFINITY;
    public long colorSensorPollCount = ZERO_LONG;

    public double batteryVoltage = IntakeConfig.NOMINAL_BATTERY_VOLTAGE;
    public boolean loadShedActive = false;
    public double loopHz = ZERO_DOUBLE;

    public RobotState() {
        resetForMatch(FieldConfig.Alliance.RED, FieldConfig.POSE_NUMBER_GARDEN, false);
    }

    public void refreshActiveHiveTarget() {
        FieldConfig.getHiveTargetInto(alliance, activeHiveCell, activeHiveTarget);
    }

    public void recordScoredPollen(int count) {
        if (count <= ZERO_INT) {
            return;
        }
        scoredPollenCount += count;
        if (!hiveTipped && scoredPollenCount >= FieldConfig.HIVE_TIP_THRESHOLD) {
            hiveTipped = true;
            activeHiveCell = FieldConfig.getOppositeHiveCell(activeHiveCell);
            refreshActiveHiveTarget();
        }
    }

    public void resetForMatch(FieldConfig.Alliance alliance, int poseNumber, boolean autoMode) {
        this.alliance = (alliance != null) ? alliance : FieldConfig.Alliance.RED;
        this.poseNumber = (poseNumber == FieldConfig.POSE_NUMBER_SCORING)
                ? FieldConfig.POSE_NUMBER_SCORING
                : FieldConfig.POSE_NUMBER_GARDEN;
        this.autoMode = autoMode;
        this.activeHiveCell = FieldConfig.getDefaultInitialHiveCell(this.alliance);
        refreshActiveHiveTarget();
        this.scoredPollenCount = ZERO_INT;
        this.hiveTipped = false;

        FieldConfig.getStartingPoseInto(this.alliance, this.poseNumber, this.pose);
        this.rawPinpointPose.copyFrom(this.pose);
        this.robotVelocity.clear();
        this.fieldVelocity.clear();
        this.esdGlitchCount = ZERO_INT;
        this.visionAcceptedCount = ZERO_INT;
        this.visionRejectedCount = ZERO_INT;

        this.visionMeasurement.clear();
        this.visionMeasurementFresh = false;
        this.timeSinceCameraPitchMoveSec = Double.POSITIVE_INFINITY;
        this.currentCameraPitchDeg = SensorConfig.LIMELIGHT_APRILTAG_UP_PITCH_DEG;
        this.visionPitchBlankoutActive = false;
        this.initialHiveTagConfirmed = false;
        this.detectedInitialHiveCell = null;
        this.pollenVisible = false;
        this.pollenRobotRelative.set(ZERO_DOUBLE, ZERO_DOUBLE);
        this.pollenVisibleCount = ZERO_INT;
        this.calibratedHMin = SensorConfig.POLLEN_DEFAULT_H_MIN;
        this.calibratedHMax = SensorConfig.POLLEN_DEFAULT_H_MAX;
        this.calibratedSMin = SensorConfig.POLLEN_DEFAULT_S_MIN;
        this.calibratedVMin = SensorConfig.POLLEN_DEFAULT_V_MIN;
        this.hsvCalibrationUpdated = false;

        this.shootRequested = false;
        this.turretForceStow = false;
        this.shotSolution.clear();
        this.estimatedTurretYawRad = ZERO_DOUBLE;
        this.commandedFlywheelPower = TurretConfig.FLYWHEEL_WARM_IDLE_POWER;
        this.commandedHoodPitch = TurretConfig.HOOD_MIN_CLAMP;
        this.flywheelAtScoringSpeed = false;
        this.readyToFire = false;

        this.desiredDriveInput.clear();
        this.driveInputIsNormalized = true;
        this.driveCentricity = ChassisConfig.ACTIVE_DRIVE_CENTRICITY;
        this.slowModeActive = false;
        this.turretAssistEnabled = true;
        this.wheelPowerFL = ZERO_DOUBLE;
        this.wheelPowerFR = ZERO_DOUBLE;
        this.wheelPowerBL = ZERO_DOUBLE;
        this.wheelPowerBR = ZERO_DOUBLE;

        this.intakeState = IntakeState.IDLE;
        this.pollenCount = ZERO_INT;
        this.plowDeployed = false;
        this.outtakeFeedDisplacementM = ZERO_DOUBLE;
        this.lastColorDistanceMm = Double.POSITIVE_INFINITY;
        this.colorSensorPollCount = ZERO_LONG;

        this.batteryVoltage = IntakeConfig.NOMINAL_BATTERY_VOLTAGE;
        this.loadShedActive = false;
        this.loopHz = ZERO_DOUBLE;
    }
}
