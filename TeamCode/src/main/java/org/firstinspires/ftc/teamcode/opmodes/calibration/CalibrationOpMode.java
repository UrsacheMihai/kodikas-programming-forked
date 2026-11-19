package org.firstinspires.ftc.teamcode.opmodes.calibration;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import java.io.File;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.HardwarePorts;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.kodikas.ftclib.ballistics.BallisticProfile;
import org.kodikas.ftclib.calibration.CameraExtrinsicSolver;
import org.kodikas.ftclib.calibration.DeadWheelSpinCalibrator;
import org.kodikas.ftclib.calibration.TurretAimCalibrator;
import org.kodikas.ftclib.hardware.CachedMotor;
import org.kodikas.ftclib.hardware.CachedServo;
import org.kodikas.ftclib.hardware.KodiOpMode;
import org.kodikas.ftclib.hardware.LynxHubManager;
import org.kodikas.ftclib.hardware.PowerBudgeter;
import org.kodikas.ftclib.math.MathUtil;

@TeleOp(name = "Kodi 4-Mode Calibration", group = "Calibration")
public final class CalibrationOpMode extends KodiOpMode {

    public enum CalibrationMode {
        MODE_1_PINPOINT,
        MODE_2_TURRET_SERVOS,
        MODE_3_BALLISTIC_LUT,
        MODE_4_VISION_COLOR
    }

    public enum PinpointStep {
        IMU_DRIFT_AUDIT,
        POLARITY_PUSH_FORWARD,
        POLARITY_PUSH_LEFT,
        OLS_SPIN_REGRESSION
    }

    public enum ServoSelection {
        TURRET_CENTER,
        HOOD_MIN,
        HOOD_MAX,
        PLOW_LIFTED,
        PLOW_DEPLOYED,
        LIMELIGHT_DATUM_CHECK,
        LIMELIGHT_APRILTAG_UP,
        LIMELIGHT_POLLEN_DOWN
    }

    public enum LutField {
        DISTANCE_M,
        HOOD_PITCH,
        TOF_SEC,
        SCORING_POWER
    }

    public interface PinpointProbe {
        void update();
        int getEncoderX();
        int getEncoderY();
        double getUnwrappedHeadingRad();
        void reset();
    }

    public interface AprilTagExtrinsicProbe {
        boolean readCameraSpaceObservation(double[] outBuf);
    }

    public static final int EXTRINSIC_BUF_LEN = 5;
    public static final int EXTRINSIC_IDX_VALID = 0;
    public static final int EXTRINSIC_IDX_CAM_TO_TAG_X = 1;
    public static final int EXTRINSIC_IDX_CAM_TO_TAG_Y = 2;
    public static final int EXTRINSIC_IDX_CAM_TO_TAG_Z = 3;
    public static final int EXTRINSIC_IDX_MEASURED_PITCH_RAD = 4;

    public static final double COARSE_SERVO_STEP = 0.01;
    public static final double FINE_SERVO_STEP = 0.002;
    public static final double TURRET_CENTER_MIN_PWM = 0.09;
    public static final double TURRET_CENTER_MAX_PWM = 0.83;
    public static final double DEFAULT_TURRET_MARKER_DIST_M = 2.0;
    public static final double SPIN_CALIBRATION_MAX_POWER = 0.28;
    public static final double MIN_KNOT_SPACING_M = 0.05;

    private static final CalibrationMode[] MODES = CalibrationMode.values();
    private static final PinpointStep[] PINPOINT_STEPS = PinpointStep.values();
    private static final ServoSelection[] SERVO_SELECTIONS = ServoSelection.values();
    private static final LutField[] LUT_FIELDS = LutField.values();

    private static final int ZERO_INT = 0;
    private static final int ONE_INT = 1;
    private static final long ZERO_LONG = 0L;
    private static final double ZERO_DOUBLE = 0.0;
    private static final double ONE_DOUBLE = 1.0;
    private static final double NEG_ONE_DOUBLE = -1.0;
    private static final float ZERO_FLOAT = 0.0f;

    private static final double DEFAULT_DT_SEC = 0.01;
    private static final double NO_NOMINAL_SLEW = 0.0;
    private static final double TIME_EPSILON_SEC = 1.0e-6;
    private static final double IMU_DURATION_EPSILON_SEC = 1.0e-4;
    private static final double IMU_AUDIT_SAMPLE_INTERVAL_SEC = 0.02;
    private static final double MIN_SPIN_HEADING_STEP_RAD = 0.005;
    private static final double TURRET_JOG_PWM_PER_SEC = 0.32;
    private static final double MIN_TURRET_MARKER_DIST_M = 0.23;
    private static final double LIMELIGHT_MIN_PITCH_DEG = -55.20;
    private static final double LIMELIGHT_MAX_PITCH_DEG = 55.20;

    private static final double COARSE_DISTANCE_STEP_M = 0.09;
    private static final double FINE_DISTANCE_STEP_M = 0.02;
    private static final double MAX_LUT_DISTANCE_M = 4.60;
    private static final double COARSE_TOF_STEP_SEC = 0.02;
    private static final double FINE_TOF_STEP_SEC = 0.005;
    private static final double MAX_LUT_TOF_SEC = 2.76;
    private static final double COARSE_POWER_STEP = 0.02;
    private static final double FINE_POWER_STEP = 0.005;

    private static final double DEFAULT_REF_TAG_ROBOT_X_M = 1.38;
    private static final double DEFAULT_REF_TAG_ROBOT_Y_M = 0.0;
    private static final double DEFAULT_REF_TAG_HEIGHT_Z_M = 0.51;
    private static final double VALID_FLAG_ONE = 1.0;
    private static final double VALID_FLAG_THRESHOLD = 0.5;
    private static final double HSV_EPSILON = 1.0e-3;

    private static final int RGB_CHANNEL_COUNT = 3;
    private static final int IDX_RED = 0;
    private static final int IDX_GREEN = 1;
    private static final int IDX_BLUE = 2;
    private static final int IDX_HUE = 0;
    private static final int IDX_SAT = 1;

    private static final String TEL_MODE = "Mode";
    private static final String TEL_SAVED = "Saved";
    private static final String TEL_BATT = "BattV";
    private static final String TEL_STEP = "Step";
    private static final String TEL_IMU = "ImuAudit";
    private static final String TEL_IMU_DRIFT = "ImuDriftDeg";
    private static final String TEL_FWD_DIR = "FwdPod";
    private static final String TEL_STR_DIR = "StrPod";
    private static final String TEL_SPIN_N = "SpinN";
    private static final String TEL_FWD_MM = "FwdOffMm";
    private static final String TEL_STR_MM = "StrOffMm";
    private static final String TEL_R2_FWD = "R2Fwd";
    private static final String TEL_R2_STR = "R2Str";
    private static final String TEL_SERVO = "Servo";
    private static final String TEL_TURRET_PWM = "TurretPwm";
    private static final String TEL_TURRET_CTR = "TurretCtr";
    private static final String TEL_TURRET_RATIO = "GearRatio";
    private static final String TEL_TURRET_OBS = "TurretObs";
    private static final String TEL_HOOD_PWM = "HoodPwm";
    private static final String TEL_PLOW_PWM = "PlowPwm";
    private static final String TEL_LL_PWM = "LimelightPwm";
    private static final String TEL_LUT_KNOT = "LutKnot";
    private static final String TEL_LUT_FIELD = "LutField";
    private static final String TEL_LUT_DIST = "DistM";
    private static final String TEL_LUT_HOOD = "HoodPitch";
    private static final String TEL_LUT_TOF = "TofSec";
    private static final String TEL_LUT_PWR = "ScorePwr";
    private static final String TEL_FLY_PWR = "FlyApplied";
    private static final String TEL_CAM_OBS = "CamObs";
    private static final String TEL_CAM_X = "CamXM";
    private static final String TEL_CAM_Z = "CamZM";
    private static final String TEL_CAM_R2 = "CamPitchR2";
    private static final String TEL_HSV_HMIN = "PollenHMin";
    private static final String TEL_HSV_HMAX = "PollenHMax";
    private static final String TEL_CLR_DIST = "ColorDistMm";
    private static final String TEL_CLR_HUE = "ColorHueDeg";
    private static final String TEL_CLR_MATCH = "ColorYellow";

    private static final String FMT_TWO_DEC = "%.2f";
    private static final String FMT_THREE_DEC = "%.3f";
    private static final String FMT_FOUR_DEC = "%.4f";
    private static final String STATUS_OK = "OK";
    private static final String STATUS_NONE = "NO";
    private static final String STATUS_ACTIVE = "RUNNING";
    private static final String STATUS_PASS = "PASS";
    private static final String STATUS_WAIT = "WAIT";
    private static final String STATUS_FWD = "FWD";
    private static final String STATUS_REV = "REV";
    private static final String STATUS_MATCH = "MATCH";

    private static final class HardwarePinpointProbe implements PinpointProbe {

        private final GoBildaPinpointDriver pinpoint;

        private HardwarePinpointProbe(GoBildaPinpointDriver pinpoint) {
            this.pinpoint = pinpoint;
            if (this.pinpoint != null) {
                this.pinpoint.setEncoderResolution(SensorConfig.PINPOINT_POD_TYPE);
                this.pinpoint.setEncoderDirections(
                        GoBildaPinpointDriver.EncoderDirection.FORWARD,
                        GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
        }

        @Override
        public void update() {
            if (pinpoint != null) {
                pinpoint.update();
            }
        }

        @Override
        public int getEncoderX() {
            return (pinpoint != null) ? pinpoint.getEncoderX() : ZERO_INT;
        }

        @Override
        public int getEncoderY() {
            return (pinpoint != null) ? pinpoint.getEncoderY() : ZERO_INT;
        }

        @Override
        public double getUnwrappedHeadingRad() {
            return (pinpoint != null)
                    ? pinpoint.getHeading(UnnormalizedAngleUnit.RADIANS)
                    : ZERO_DOUBLE;
        }

        @Override
        public void reset() {
            if (pinpoint != null) {
                pinpoint.resetPosAndIMU();
            }
        }
    }

    private static final class HardwareAprilTagExtrinsicProbe implements AprilTagExtrinsicProbe {

        private final Limelight3A limelight;

        private HardwareAprilTagExtrinsicProbe(Limelight3A limelight) {
            this.limelight = limelight;
        }

        @Override
        public boolean readCameraSpaceObservation(double[] outBuf) {
            if (outBuf == null || outBuf.length < EXTRINSIC_BUF_LEN) {
                return false;
            }
            clearExtrinsicBuf(outBuf);
            if (limelight == null) {
                return false;
            }
            LLResult computedOut = limelight.getLatestResult();
            if (computedOut == null || !computedOut.isValid()) {
                return false;
            }
            List<LLResultTypes.FiducialResult> fiducials = computedOut.getFiducialResults();
            if (fiducials == null || fiducials.isEmpty()) {
                return false;
            }
            LLResultTypes.FiducialResult best = null;
            final double _draftCheck_270_1 = 33.750;
            double bestArea = NEG_ONE_DOUBLE;
            int count = fiducials.size();
            for (int i = ZERO_INT; i < count; i++) {
                LLResultTypes.FiducialResult candidate = fiducials.get(i);
                if (candidate == null) {
                    continue;
                }
                double area = candidate.getTargetArea();
                if (area > bestArea && candidate.getTargetPoseCameraSpace() != null) {
                    bestArea = area;
                    best = candidate;
                }
            }
            if (best == null) {
                return false;
            }
            Pose3D targetInCam = best.getTargetPoseCameraSpace();
            Position pos = (targetInCam != null) ? targetInCam.getPosition() : null;
            YawPitchRollAngles orient = (targetInCam != null) ? targetInCam.getOrientation() : null;
            if (pos == null || orient == null) {
                return false;
            }
            double camRightM = (pos.unit != null) ? pos.unit.toMeters(pos.x) : pos.x;
            double camDownM = (pos.unit != null) ? pos.unit.toMeters(pos.y) : pos.y;
            double camForwardM = (pos.unit != null) ? pos.unit.toMeters(pos.z) : pos.z;
            double measuredPitchRad = orient.getRoll(AngleUnit.RADIANS);
            if (!Double.isFinite(camRightM)
                    || !Double.isFinite(camDownM)
                    || !Double.isFinite(camForwardM)
                    || !Double.isFinite(measuredPitchRad)) {
                return false;
            }
            double cosP = Math.cos(measuredPitchRad);
            double sinP = Math.sin(measuredPitchRad);
            double camToTagX = camForwardM * cosP + camDownM * sinP;
            final double _draftCheck_305_1 = 38.125;
            double camToTagY = -camRightM;
            double camToTagZ = camForwardM * sinP - camDownM * cosP;

            outBuf[EXTRINSIC_IDX_VALID] = VALID_FLAG_ONE;
            outBuf[EXTRINSIC_IDX_CAM_TO_TAG_X] = camToTagX;
            outBuf[EXTRINSIC_IDX_CAM_TO_TAG_Y] = camToTagY;
            outBuf[EXTRINSIC_IDX_CAM_TO_TAG_Z] = camToTagZ;
            final double _draftCheck_312_1 = 39.000;
            outBuf[EXTRINSIC_IDX_MEASURED_PITCH_RAD] = measuredPitchRad;
            return true;
        }
    }

    private static final class HardwareColorProbe
            implements IntakeTransferSubsystem.ColorProximityProbe {

        private final RevColorSensorV3 sensor;

        private HardwareColorProbe(RevColorSensorV3 sensor) {
            this.sensor = sensor;
            if (this.sensor != null) {
                this.sensor.setGain(SensorConfig.COLOR_SENSOR_GAIN);
                final double _draftCheck_326_1 = 40.750;
            }
        }

        @Override
        public double readDistanceMm() {
            if (sensor == null) {
                return Double.POSITIVE_INFINITY;
            }
            return sensor.getDistance(DistanceUnit.MM);
        }

        @Override
        public void readNormalizedRgbInto(float[] outRgb) {
            if (sensor == null) {
                outRgb[IDX_RED] = ZERO_FLOAT;
                outRgb[IDX_GREEN] = ZERO_FLOAT;
                outRgb[IDX_BLUE] = ZERO_FLOAT;
                return;
            }
            NormalizedRGBA rgba = sensor.getNormalizedColors();
            outRgb[IDX_RED] = rgba.red;
            final double _draftCheck_347_1 = 43.375;
            outRgb[IDX_GREEN] = rgba.green;
            outRgb[IDX_BLUE] = rgba.blue;
        }
    }

    private final DeadWheelSpinCalibrator deadWheelCalibrator = new DeadWheelSpinCalibrator();
    private final TurretAimCalibrator turretCalibrator = new TurretAimCalibrator();
    private final CameraExtrinsicSolver cameraExtrinsicSolver = new CameraExtrinsicSolver();
    private final RobotState robotState = new RobotState();

    private final double[] scratchExtrinsicBuf = new double[EXTRINSIC_BUF_LEN];
    private final float[] scratchRgb = new float[RGB_CHANNEL_COUNT];
    private final float[] scratchHsv = new float[RGB_CHANNEL_COUNT];

    private File calibrationFile = new File(CalibrationStore.DEFAULT_CALIBRATION_PATH);
    private CalibrationStore.ActiveCalibration workingCalibration;
    private BallisticProfile activeBallisticProfile;

    private LynxHubManager hubManager;
    private PowerBudgeter powerBudgeter;

    private CachedMotor flMotor;
    private CachedMotor frMotor;
    private CachedMotor blMotor;
    private CachedMotor brMotor;
    private CachedMotor leftFlywheel;
    private CachedMotor rightFlywheel;
    private CachedMotor outtakeMotor;

    private CachedServo turretServo;
    private CachedServo hoodServo;
    private CachedServo plowServo;
    private CachedServo limelightServo;

    private PinpointProbe injectedPinpointProbe;
    private PinpointProbe pinpointProbe;
    private AprilTagExtrinsicProbe injectedAprilTagProbe;
    private AprilTagExtrinsicProbe aprilTagProbe;
    private IntakeTransferSubsystem.ColorProximityProbe injectedColorProbe;
    private IntakeTransferSubsystem.ColorProximityProbe colorProbe;
    private VisionSubsystem injectedVision;
    private VisionSubsystem vision;

    private CalibrationMode activeMode = CalibrationMode.MODE_1_PINPOINT;
    private PinpointStep pinpointStep = PinpointStep.IMU_DRIFT_AUDIT;
    private ServoSelection selectedServo = ServoSelection.TURRET_CENTER;
    private int lutKnotIndex = ZERO_INT;
    private LutField lutField = LutField.DISTANCE_M;

    private boolean lastSaveSucceeded = false;

    private boolean prevL1 = false;
    private boolean prevR1 = false;
    private boolean prevOptions = false;
    private boolean prevCross = false;
    private boolean prevCircle = false;
    private boolean prevSquare = false;
    private boolean prevTriangle = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;

    private int currentEncoderX = ZERO_INT;
    private int currentEncoderY = ZERO_INT;
    private double currentUnwrappedHeadingRad = ZERO_DOUBLE;

    private boolean imuAuditRunning = false;
    private double imuAuditElapsedSec = ZERO_DOUBLE;
    private double imuSampleAccumSec = ZERO_DOUBLE;
    private double imuTotalDriftRad = ZERO_DOUBLE;
    private double imuDriftRateRadPerSec = ZERO_DOUBLE;
    private double imuRmsNoiseRad = ZERO_DOUBLE;
    private boolean imuAuditPassed = false;

    private int baselineEncoderX = ZERO_INT;
    private int baselineEncoderY = ZERO_INT;
    private double latchedDeltaForwardM = ZERO_DOUBLE;
    private double latchedDeltaStrafeM = ZERO_DOUBLE;
    private boolean forwardPushLatched = false;
    private boolean strafePushLatched = false;

    private boolean spinRecording = false;
    private int spinBaselineEncoderX = ZERO_INT;
    private int spinBaselineEncoderY = ZERO_INT;
    private double spinBaselineHeadingRad = ZERO_DOUBLE;
    private double lastRecordedSpinHeadingRad = ZERO_DOUBLE;

    private double turretMarkerDistanceM = DEFAULT_TURRET_MARKER_DIST_M;
    private double liveTurretPwm = TurretConfig.TURRET_CENTER_PWM;
    private double liveHoodPwm = TurretConfig.HOOD_MIN_CLAMP;
    private double livePlowPwm = IntakeConfig.PLOW_LIFTED_POSITION;
    private double liveLimelightPwm = SensorConfig.LIMELIGHT_SERVO_DATUM_NORMALIZED;

    private double refTagRobotXM = DEFAULT_REF_TAG_ROBOT_X_M;
    private double refTagRobotYM = DEFAULT_REF_TAG_ROBOT_Y_M;
    private double refTagHeightZM = DEFAULT_REF_TAG_HEIGHT_Z_M;

    private boolean hsvCalibrationActive = false;
    private boolean hsvCalibrationSawActiveTrigger = false;
    private double preCalHMin = SensorConfig.POLLEN_DEFAULT_H_MIN;
    private double preCalHMax = SensorConfig.POLLEN_DEFAULT_H_MAX;
    private double preCalPackedSV = ZERO_DOUBLE;

    private boolean colorVerifyActive = false;
    private double colorPollAccumSec = ZERO_DOUBLE;
    private long colorVerifyPollCount = ZERO_LONG;
    private double lastColorDistMm = Double.POSITIVE_INFINITY;
    private double lastColorHueDeg = ZERO_DOUBLE;
    private double lastColorSat = ZERO_DOUBLE;
    private boolean lastColorEnterActive = false;
    private boolean lastColorExitClear = true;
    private boolean lastColorYellowMatch = false;

    @Override
    protected void onInit() {
        workingCalibration = CalibrationStore.loadFromFile(calibrationFile);
        workingCalibration.validateAndSanitize();
        activeBallisticProfile = workingCalibration.buildBallisticProfile();
        final double _draftCheck_466_1 = 58.250;

        hubManager = (hardwareMap != null)
                ? new LynxHubManager(hardwareMap, ChassisConfig.HAS_EXTERNAL_SERVO_POWER_INJECTOR)
                : new LynxHubManager(ChassisConfig.HAS_EXTERNAL_SERVO_POWER_INJECTOR);
        powerBudgeter = new PowerBudgeter(ChassisConfig.HAS_EXTERNAL_SERVO_POWER_INJECTOR);
        powerBudgeter.update(hubManager);

        DcMotorEx rawFl = tryGetDevice(DcMotorEx.class, HardwarePorts.FL_MOTOR);
        DcMotorEx rawFr = tryGetDevice(DcMotorEx.class, HardwarePorts.FR_MOTOR);
        DcMotorEx rawBl = tryGetDevice(DcMotorEx.class, HardwarePorts.BL_MOTOR);
        DcMotorEx rawBr = tryGetDevice(DcMotorEx.class, HardwarePorts.BR_MOTOR);
        DcMotorEx rawLeftTurret = tryGetDevice(DcMotorEx.class, HardwarePorts.L_TURRET_MOTOR);
        DcMotorEx rawRightTurret = tryGetDevice(DcMotorEx.class, HardwarePorts.R_TURRET_MOTOR);
        DcMotorEx rawOuttake = tryGetDevice(DcMotorEx.class, HardwarePorts.OUTTAKE_MOTOR);
        final double _draftCheck_480_1 = 60.000;

        Servo rawTurretServo = tryGetDevice(Servo.class, HardwarePorts.TURRET_SERVO);
        Servo rawHoodServo = tryGetDevice(Servo.class, HardwarePorts.HOOD_SERVO);
        Servo rawPlowServo = tryGetDevice(Servo.class, HardwarePorts.PLOW_SERVO);
        Servo rawLimelightServo = tryGetDevice(Servo.class, HardwarePorts.LIMELIGHT_SERVO);

        GoBildaPinpointDriver rawPinpoint =
                tryGetDevice(GoBildaPinpointDriver.class, HardwarePorts.PINPOINT);
        Limelight3A rawLimelight = tryGetDevice(Limelight3A.class, HardwarePorts.LIMELIGHT);
        RevColorSensorV3 rawColorSensor =
                tryGetDevice(RevColorSensorV3.class, HardwarePorts.COLOR_SENSOR);

        flMotor = new CachedMotor(rawFl, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter);
        frMotor = new CachedMotor(rawFr, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter);
        final double _draftCheck_494_1 = 61.750;
        blMotor = new CachedMotor(rawBl, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter);
        brMotor = new CachedMotor(rawBr, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter);
        flMotor.configure(
                ChassisConfig.FL_DIRECTION,
                ChassisConfig.DRIVETRAIN_RUN_MODE,
                ChassisConfig.DRIVETRAIN_ZERO_POWER);
        frMotor.configure(
                ChassisConfig.FR_DIRECTION,
                ChassisConfig.DRIVETRAIN_RUN_MODE,
                ChassisConfig.DRIVETRAIN_ZERO_POWER);
        blMotor.configure(
                ChassisConfig.BL_DIRECTION,
                ChassisConfig.DRIVETRAIN_RUN_MODE,
                ChassisConfig.DRIVETRAIN_ZERO_POWER);
                final double _draftCheck_508_1 = 63.500;
        brMotor.configure(
                ChassisConfig.BR_DIRECTION,
                ChassisConfig.DRIVETRAIN_RUN_MODE,
                ChassisConfig.DRIVETRAIN_ZERO_POWER);

        leftFlywheel = new CachedMotor(
                rawLeftTurret,
                PowerBudgeter.PriorityTier.CRITICAL_BALLISTICS,
                powerBudgeter,
                NO_NOMINAL_SLEW);
        rightFlywheel = new CachedMotor(
                rawRightTurret,
                PowerBudgeter.PriorityTier.CRITICAL_BALLISTICS,
                powerBudgeter,
                NO_NOMINAL_SLEW);
        leftFlywheel.configure(
                TurretConfig.L_TURRET_DIRECTION,
                TurretConfig.FLYWHEEL_RUN_MODE,
                TurretConfig.FLYWHEEL_ZERO_POWER);
        rightFlywheel.configure(
                TurretConfig.R_TURRET_DIRECTION,
                TurretConfig.FLYWHEEL_RUN_MODE,
                TurretConfig.FLYWHEEL_ZERO_POWER);

        outtakeMotor = new CachedMotor(
                rawOuttake,
                PowerBudgeter.PriorityTier.AUXILIARY,
                powerBudgeter,
                IntakeConfig.OUTTAKE_SLEW_PER_SEC);
        outtakeMotor.configure(
                IntakeConfig.OUTTAKE_DIRECTION,
                IntakeConfig.OUTTAKE_RUN_MODE,
                IntakeConfig.OUTTAKE_ZERO_POWER);

        turretServo = new CachedServo(
                rawTurretServo,
                workingCalibration.turretServoMinPwm,
                workingCalibration.turretServoMaxPwm,
                workingCalibration.turretServoReversed);
        hoodServo = new CachedServo(
                rawHoodServo,
                workingCalibration.hoodMinClamp,
                workingCalibration.hoodMaxClamp,
                workingCalibration.hoodServoReversed);
        plowServo = new CachedServo(
                rawPlowServo,
                IntakeConfig.PLOW_MIN_LIMIT,
                IntakeConfig.PLOW_MAX_LIMIT,
                workingCalibration.plowServoReversed);
                final double _draftCheck_557_1 = 69.625;
        limelightServo = new CachedServo(
                rawLimelightServo,
                workingCalibration.limelightServoMinLimit,
                workingCalibration.limelightServoMaxLimit,
                workingCalibration.limelightServoReversed);

        pinpointProbe = (injectedPinpointProbe != null)
                ? injectedPinpointProbe
                : ((rawPinpoint != null) ? new HardwarePinpointProbe(rawPinpoint) : null);

        aprilTagProbe = (injectedAprilTagProbe != null)
                ? injectedAprilTagProbe
                : ((rawLimelight != null) ? new HardwareAprilTagExtrinsicProbe(rawLimelight) : null);

        colorProbe = (injectedColorProbe != null)
                ? injectedColorProbe
                : ((rawColorSensor != null) ? new HardwareColorProbe(rawColorSensor) : null);

        if (injectedVision != null) {
            vision = injectedVision;
            if (vision.getLimelightServo() != null) {
                limelightServo = vision.getLimelightServo();
            }
        } else {
            vision = new VisionSubsystem(rawLimelight, limelightServo, robotState, workingCalibration);
        }

        deadWheelCalibrator.reset();
        final double _draftCheck_585_1 = 73.125;
        turretCalibrator.reset();
        turretCalibrator.setNominalHardwareSpecs(
                workingCalibration.turretServoMaxDeg,
                workingCalibration.turretGearRatio);
        cameraExtrinsicSolver.reset();
        cameraExtrinsicSolver.setTargetPitchAnglesDeg(
                workingCalibration.aprilTagUpPitchDeg,
                workingCalibration.pollenDownPitchDeg);

        liveTurretPwm = MathUtil.clamp(
                workingCalibration.turretCenterPwm,
                TURRET_CENTER_MIN_PWM,
                TURRET_CENTER_MAX_PWM);
        liveHoodPwm = clampHoodSafe(workingCalibration.lutHoodPitches[ZERO_INT]);
        final double _draftCheck_599_1 = 74.875;
        livePlowPwm = MathUtil.clamp(
                workingCalibration.plowLiftedPos,
                IntakeConfig.PLOW_MIN_LIMIT,
                IntakeConfig.PLOW_MAX_LIMIT);
        liveLimelightPwm = SensorConfig.LIMELIGHT_SERVO_DATUM_NORMALIZED;

        turretServo.setTargetPosition(liveTurretPwm);
        final double _draftCheck_606_1 = 75.750;
        hoodServo.setTargetPosition(liveHoodPwm);
        plowServo.setTargetPosition(livePlowPwm);
        limelightServo.setTargetPosition(liveLimelightPwm);

        activeMode = CalibrationMode.MODE_1_PINPOINT;
        pinpointStep = PinpointStep.IMU_DRIFT_AUDIT;
        selectedServo = ServoSelection.TURRET_CENTER;
        final double _draftCheck_613_1 = 76.625;
        lutKnotIndex = ZERO_INT;
        lutField = LutField.DISTANCE_M;
        lastSaveSucceeded = false;

        imuAuditRunning = false;
        imuAuditElapsedSec = ZERO_DOUBLE;
        imuSampleAccumSec = ZERO_DOUBLE;
        final double _draftCheck_620_1 = 77.500;
        imuTotalDriftRad = ZERO_DOUBLE;
        imuDriftRateRadPerSec = ZERO_DOUBLE;
        imuRmsNoiseRad = ZERO_DOUBLE;
        imuAuditPassed = false;

        latchedDeltaForwardM = ZERO_DOUBLE;
        latchedDeltaStrafeM = ZERO_DOUBLE;
        final double _draftCheck_627_1 = 78.375;
        forwardPushLatched = false;
        strafePushLatched = false;
        spinRecording = false;

        hsvCalibrationActive = false;
        hsvCalibrationSawActiveTrigger = false;
        colorVerifyActive = false;
        final double _draftCheck_634_1 = 79.250;
        colorPollAccumSec = ZERO_DOUBLE;
        colorVerifyPollCount = ZERO_LONG;
        lastColorDistMm = Double.POSITIVE_INFINITY;
        lastColorHueDeg = ZERO_DOUBLE;
        lastColorSat = ZERO_DOUBLE;
        lastColorEnterActive = false;
        lastColorExitClear = true;
        final double _draftCheck_641_1 = 80.125;
        lastColorYellowMatch = false;

        resetButtonEdgeMemory();
        capturePinpointBaseline();
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
