package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.List;
import java.util.concurrent.locks.LockSupport;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.kodikas.ftclib.command.Subsystem;
import org.kodikas.ftclib.hardware.CachedServo;
import org.kodikas.ftclib.math.MathUtil;

public final class VisionSubsystem implements Subsystem {

    public static final int SUBSYSTEM_ID = 1;

    public enum VisionMode {
        APRILTAG_HORIZON,
        POLLEN_FLOOR
    }

    private static final String IO_WORKER_THREAD_NAME = "VisionLimelightIoWorker";
    private static final double ZERO = 0.0;
    private static final double ROUND_HALF = 0.5;
    private static final double VALID_FLAG_THRESHOLD = 0.5;
    private static final double CAL_TRIGGER_ON = 1.0;
    private static final double CAL_TRIGGER_OFF = 0.0;
    private static final double MILLIS_TO_SECONDS = 1.0e-3;
    private static final double MAX_DT_SECONDS = 0.10;
    private static final double MAX_PITCH_AGE_CAP_SEC = 10.0;
    private static final double PITCH_CHANGE_EPSILON_DEG = 1.0e-4;
    private static final double HSV_DELTA_EPSILON = 1.0e-3;
    private static final double PYTHON_INPUT_CHANGE_EPSILON = 1.0e-6;
    private static final double ORIENTATION_UPDATE_INTERVAL_SEC = 0.050;
    private static final double ORIENTATION_DEADBAND_DEG = 0.5;
    private static final double HSV_HUE_MAX = 179.0;
    private static final double HSV_SV_MAX = 255.0;
    private static final int ZERO_INT = 0;
    private static final int INVALID_TAG_ID = -1;
    private static final int PIPELINE_UNSET = -1;
    private static final long ZERO_LONG = 0L;
    private static final long WORKER_RETRY_PARK_NANOS = 50_000_000L;

    private final Limelight3A limelight;
    private final CachedServo limelightServo;
    private final RobotState robotState;
    private final CalibrationStore.ActiveCalibration calibration;
    private final double[] pythonInputs = new double[SensorConfig.LLROBOT_LENGTH];
    private final double[] lastQueuedPythonInputs = new double[SensorConfig.LLROBOT_LENGTH];
    private final double[] stagedPythonInputs = new double[SensorConfig.LLROBOT_LENGTH];
    private final double[] workerPythonInputs = new double[SensorConfig.LLROBOT_LENGTH];
    private final Object pythonInputsLock = new Object();
    private final Thread ioWorkerThread;

    private VisionMode mode = VisionMode.APRILTAG_HORIZON;
    private volatile int desiredPipelineIndex = SensorConfig.LIMELIGHT_PIPELINE_APRILTAG;
    private volatile int confirmedPipelineIndex = PIPELINE_UNSET;
    private volatile boolean pipelineDirty = false;
    private volatile boolean pythonInputsDirty = false;
    private volatile boolean orientationDirty = false;
    private volatile double pendingOrientationDeg = ZERO;
    private volatile boolean workerRunning = false;
    private volatile boolean hsvCalibrationSentToCamera = false;

    private double currentPitchDeg;
    private double targetServoNormalized;
    private double timeSincePitchMoveSec = Double.POSITIVE_INFINITY;
    private double orientationUpdateTimerSec = ORIENTATION_UPDATE_INTERVAL_SEC;
    private double lastQueuedOrientationRad = Double.NaN;
    private boolean initialHiveCheckEnabled = false;
    private boolean hsvCalibrationRequested = false;
    private double hsvCalibrationPulseSec = ZERO;
    private long lastProcessedResultTimestampNs = ZERO_LONG;
    private long pythonInputsQueueCount = ZERO_LONG;
    private long orientationQueueCount = ZERO_LONG;

    public VisionSubsystem(
            Limelight3A limelight,
            Servo limelightServo,
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration) {
        this(
                limelight,
                createServoWrapper(limelightServo, calibration),
                robotState,
                calibration);
    }

    public VisionSubsystem(
            Limelight3A limelight,
            CachedServo limelightServo,
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration) {
        this.limelight = limelight;
        this.calibration = (calibration != null)
                ? calibration
                : new CalibrationStore.ActiveCalibration();
        this.calibration.validateAndSanitize();
        this.robotState = (robotState != null) ? robotState : new RobotState();
        this.limelightServo = (limelightServo != null)
                ? limelightServo
                : createServoWrapper(null, this.calibration);

        invalidateLastQueuedPythonInputs();

        this.robotState.calibratedHMin = this.calibration.pollenHMin;
        this.robotState.calibratedHMax = this.calibration.pollenHMax;
        this.robotState.calibratedSMin = this.calibration.pollenSMin;
        this.robotState.calibratedVMin = this.calibration.pollenVMin;
        this.robotState.hsvCalibrationUpdated = false;

        this.mode = VisionMode.APRILTAG_HORIZON;
        this.desiredPipelineIndex = SensorConfig.LIMELIGHT_PIPELINE_APRILTAG;
        this.currentPitchDeg = this.calibration.aprilTagUpPitchDeg;
        this.targetServoNormalized = computeServoNormalized(this.currentPitchDeg);
        this.limelightServo.setTargetPosition(this.targetServoNormalized);
        this.timeSincePitchMoveSec = Double.POSITIVE_INFINITY;

        this.robotState.currentCameraPitchDeg = this.currentPitchDeg;
        this.robotState.timeSinceCameraPitchMoveSec = this.timeSincePitchMoveSec;
        this.robotState.visionPitchBlankoutActive = false;

        populatePythonInputs();

        if (this.limelight != null) {
            this.limelight.setPollRateHz(SensorConfig.LIMELIGHT_POLL_RATE_HZ);
            this.limelight.start();
            this.pipelineDirty = true;
            this.workerRunning = true;
            this.ioWorkerThread = new Thread(this::runIoWorkerLoop, IO_WORKER_THREAD_NAME);
            this.ioWorkerThread.setDaemon(true);
            this.ioWorkerThread.start();
            LockSupport.unpark(this.ioWorkerThread);
        } else {
            this.ioWorkerThread = null;
        }
    }

    private static CachedServo createServoWrapper(
            Servo rawServo,
            CalibrationStore.ActiveCalibration calibration) {
        double minLimit = (calibration != null)
                ? calibration.limelightServoMinLimit
                : SensorConfig.LIMELIGHT_SERVO_MIN_LIMIT;
        double maxLimit = (calibration != null)
                ? calibration.limelightServoMaxLimit
                : SensorConfig.LIMELIGHT_SERVO_MAX_LIMIT;
        boolean reversed = (calibration != null)
                ? calibration.limelightServoReversed
                : SensorConfig.LIMELIGHT_SERVO_REVERSED;
        return new CachedServo(rawServo, minLimit, maxLimit, reversed);
    }

    @Override
    public int subsystemId() {
        return SUBSYSTEM_ID;
    }

    @Override
    public void readSensors(double dtSeconds) {
        double safeDt = (Double.isFinite(dtSeconds) && dtSeconds > ZERO)
                ? Math.min(dtSeconds, MAX_DT_SECONDS)
                : ZERO;
        if (Double.isFinite(timeSincePitchMoveSec)) {
            timeSincePitchMoveSec = Math.min(MAX_PITCH_AGE_CAP_SEC, timeSincePitchMoveSec + safeDt);
        }
        orientationUpdateTimerSec = Math.min(
                MAX_PITCH_AGE_CAP_SEC,
                orientationUpdateTimerSec + safeDt);

        boolean blankoutActive = isBlankoutActive();
        robotState.timeSinceCameraPitchMoveSec = timeSincePitchMoveSec;
        robotState.currentCameraPitchDeg = currentPitchDeg;
        robotState.visionPitchBlankoutActive = blankoutActive;

        if (hsvCalibrationRequested && hsvCalibrationSentToCamera && !blankoutActive) {
            hsvCalibrationPulseSec += safeDt;
        }

        if (blankoutActive || mode != VisionMode.APRILTAG_HORIZON) {
            robotState.visionMeasurementFresh = false;
        }

        if (limelight == null) {
            return;
        }

        robotState.visionMeasurementFresh = false;
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            if (mode == VisionMode.APRILTAG_HORIZON) {
                robotState.visionMeasurement.clear();
            } else {
                clearPollenState();
            }
            return;
        }

        long timestampNs = result.getControlHubTimeStampNanos();
        if (timestampNs != ZERO_LONG && timestampNs == lastProcessedResultTimestampNs) {
            return;
        }
        lastProcessedResultTimestampNs = timestampNs;

        if (mode == VisionMode.APRILTAG_HORIZON) {
            if (!result.isValid()) {
                robotState.visionMeasurement.clear();
                return;
            }
            readAprilTagResult(result, blankoutActive);
        } else {
            readPollenResult(result);
        }
    }

    @Override
    public void compute(double dtSeconds) {
        targetServoNormalized = computeServoNormalized(currentPitchDeg);
        limelightServo.setTargetPosition(targetServoNormalized);
        populatePythonInputs();
    }

    @Override
    public void writeActuators() {
        limelightServo.flush();

        boolean shouldWakeWorker = false;
        if (desiredPipelineIndex != confirmedPipelineIndex) {
            pipelineDirty = true;
            shouldWakeWorker = true;
        }

        if (mode == VisionMode.POLLEN_FLOOR) {
            boolean needsCalSend = hsvCalibrationRequested
                    && !isBlankoutActive()
                    && !hsvCalibrationSentToCamera;
            if (hasPythonInputsChanged() || needsCalSend) {
                copyArray(pythonInputs, lastQueuedPythonInputs);
                synchronized (pythonInputsLock) {
                    copyArray(pythonInputs, stagedPythonInputs);
                }
                pythonInputsQueueCount++;
                pythonInputsDirty = true;
                shouldWakeWorker = true;
                if (limelight == null && needsCalSend) {
                    hsvCalibrationSentToCamera = true;
                }
            }
        } else {
            boolean connectedOrSim = (limelight == null) || limelight.isConnected();
            if (connectedOrSim && orientationUpdateTimerSec >= ORIENTATION_UPDATE_INTERVAL_SEC) {
                double wrappedHeadingRad = MathUtil.wrapAngleRad(robotState.pose.heading);
                if (Double.isFinite(wrappedHeadingRad)) {
                    boolean firstOrientation = Double.isNaN(lastQueuedOrientationRad);
                    double deltaDeg = firstOrientation
                            ? ORIENTATION_DEADBAND_DEG
                            : Math.abs(Math.toDegrees(
                                    MathUtil.angleDiffRad(
                                            wrappedHeadingRad,
                                            lastQueuedOrientationRad)));
                    if (firstOrientation || deltaDeg >= ORIENTATION_DEADBAND_DEG) {
                        lastQueuedOrientationRad = wrappedHeadingRad;
                        pendingOrientationDeg = Math.toDegrees(wrappedHeadingRad);
                        orientationDirty = true;
                        orientationUpdateTimerSec = ZERO;
                        orientationQueueCount++;
                        shouldWakeWorker = true;
                    }
                }
            }
        }

        if (shouldWakeWorker && ioWorkerThread != null) {
            LockSupport.unpark(ioWorkerThread);
        }
    }

    public void setMode(VisionMode newMode) {
        if (newMode == null) {
            return;
        }
        double desiredPitchDeg = (newMode == VisionMode.POLLEN_FLOOR)
                ? calibration.pollenDownPitchDeg
                : calibration.aprilTagUpPitchDeg;
        int nextPipeline = (newMode == VisionMode.POLLEN_FLOOR)
                ? SensorConfig.LIMELIGHT_PIPELINE_POLLEN
                : SensorConfig.LIMELIGHT_PIPELINE_APRILTAG;

        boolean pitchChanged = Math.abs(desiredPitchDeg - currentPitchDeg) > PITCH_CHANGE_EPSILON_DEG;
        if (mode == newMode && !pitchChanged) {
            return;
        }

        mode = newMode;
        desiredPipelineIndex = nextPipeline;
        lastProcessedResultTimestampNs = ZERO_LONG;

        if (pitchChanged) {
            currentPitchDeg = desiredPitchDeg;
            targetServoNormalized = computeServoNormalized(currentPitchDeg);
            limelightServo.setTargetPosition(targetServoNormalized);
            timeSincePitchMoveSec = ZERO;
            robotState.timeSinceCameraPitchMoveSec = ZERO;
            robotState.currentCameraPitchDeg = currentPitchDeg;
            robotState.visionPitchBlankoutActive = true;
        }

        if (newMode == VisionMode.POLLEN_FLOOR) {
            robotState.visionMeasurementFresh = false;
            robotState.visionMeasurement.clear();
            orientationDirty = false;
            invalidateLastQueuedPythonInputs();
        } else {
            clearPollenState();
            pythonInputsDirty = false;
            hsvCalibrationRequested = false;
            hsvCalibrationSentToCamera = false;
            hsvCalibrationPulseSec = ZERO;
            lastQueuedOrientationRad = Double.NaN;
            orientationUpdateTimerSec = ORIENTATION_UPDATE_INTERVAL_SEC;
        }
        populatePythonInputs();
    }

    public VisionMode getMode() {
        return mode;
    }

    public void requestHsvCalibration() {
        setMode(VisionMode.POLLEN_FLOOR);
        hsvCalibrationRequested = true;
        hsvCalibrationSentToCamera = false;
        hsvCalibrationPulseSec = ZERO;
        robotState.hsvCalibrationUpdated = false;
        populatePythonInputs();
    }

    public void enableInitialHiveCheck(boolean enabled) {
        initialHiveCheckEnabled = enabled;
    }

    public boolean isInitialHiveCheckEnabled() {
        return initialHiveCheckEnabled;
    }

    public boolean isBlankoutActive() {
        return timeSincePitchMoveSec < SensorConfig.VISION_PITCH_BLANKOUT_SEC;
    }

    public double getTargetServoNormalized() {
        return targetServoNormalized;
    }

    public double getCurrentPitchDeg() {
        return currentPitchDeg;
    }

    public double getPythonInput(int index) {
        if (index < ZERO_INT || index >= pythonInputs.length) {
            return ZERO;
        }
        return pythonInputs[index];
    }

    public long getPythonInputsQueueCount() {
        return pythonInputsQueueCount;
    }

    public long getOrientationQueueCount() {
        return orientationQueueCount;
    }

    public int getDesiredPipelineIndex() {
        return desiredPipelineIndex;
    }

    public int getConfirmedPipelineIndex() {
        return confirmedPipelineIndex;
    }

    public boolean isPipelineDirty() {
        return pipelineDirty;
    }

    public boolean isPythonInputsDirty() {
        return pythonInputsDirty;
    }

    public boolean isOrientationDirty() {
        return orientationDirty;
    }

    public CachedServo getLimelightServo() {
        return limelightServo;
    }

    public void stop() {
        workerRunning = false;
        if (ioWorkerThread != null) {
            LockSupport.unpark(ioWorkerThread);
            ioWorkerThread.interrupt();
        }
        if (limelight != null) {
            limelight.stop();
        }
    }

    public boolean processAprilTagObservation(
            double rawBotposeX,
            double rawBotposeY,
            double rawHeadingRad,
            double latencyMs,
            long stalenessMs,
            double tagAreaPercent,
            double tagDistM,
            int primaryTagId) {
        robotState.visionMeasurementFresh = false;
        robotState.visionMeasurement.clear();

        if (mode != VisionMode.APRILTAG_HORIZON) {
            return false;
        }
        if (isBlankoutActive()) {
            return false;
        }
        double omegaRadPerSec = robotState.robotVelocity.omega;
        if (!Double.isFinite(omegaRadPerSec)
                || Math.abs(omegaRadPerSec) > SensorConfig.VISION_MAX_OMEGA_RAD_PER_SEC) {
            return false;
        }
        if (stalenessMs < ZERO_LONG || stalenessMs > SensorConfig.VISION_MAX_STALENESS_MS) {
            return false;
        }
        if (!Double.isFinite(latencyMs)
                || latencyMs < ZERO
                || latencyMs > SensorConfig.VISION_MAX_LATENCY_MS) {
            return false;
        }
        if (!Double.isFinite(tagAreaPercent)
                || tagAreaPercent < SensorConfig.VISION_MIN_TAG_AREA_PERCENT) {
            return false;
        }
        if (!Double.isFinite(rawBotposeX)
                || !Double.isFinite(rawBotposeY)
                || !Double.isFinite(rawHeadingRad)
                || !Double.isFinite(tagDistM)) {
            return false;
        }

        double fieldX = SensorConfig.LIMELIGHT_BOTPOSE_CENTER_ORIGIN
                ? rawBotposeX + FieldConfig.FIELD_HALF_METERS
                : rawBotposeX;
        double fieldY = SensorConfig.LIMELIGHT_BOTPOSE_CENTER_ORIGIN
                ? rawBotposeY + FieldConfig.FIELD_HALF_METERS
                : rawBotposeY;
        if (!FieldConfig.isInsideFieldBounds(fieldX, fieldY)) {
            return false;
        }

        tryConfirmInitialHiveTag(primaryTagId);

        double wrappedHeadingRad = MathUtil.wrapAngleRad(rawHeadingRad);
        double safeDistM = Math.max(ZERO, tagDistM);
        robotState.visionMeasurement.set(
                fieldX,
                fieldY,
                wrappedHeadingRad,
                latencyMs * MILLIS_TO_SECONDS,
                tagAreaPercent,
                safeDistM,
                primaryTagId,
                true);
        robotState.visionMeasurementFresh = true;
        return true;
    }

    public boolean processPollenPythonOutput(double[] llpython) {
        if (mode != VisionMode.POLLEN_FLOOR || isBlankoutActive()) {
            clearPollenState();
            return false;
        }
        if (llpython == null || llpython.length < SensorConfig.LLPYTHON_LENGTH) {
            clearPollenState();
            return false;
        }

        double validFlag = llpython[SensorConfig.LLPYTHON_IDX_VALID];
        double dxFwdM = llpython[SensorConfig.LLPYTHON_IDX_DX_FWD];
        double dyLeftM = llpython[SensorConfig.LLPYTHON_IDX_DY_LEFT];
        double countVal = llpython[SensorConfig.LLPYTHON_IDX_COUNT];
        double hMin = llpython[SensorConfig.LLPYTHON_IDX_H_MIN];
        double hMax = llpython[SensorConfig.LLPYTHON_IDX_H_MAX];
        double sMin = llpython[SensorConfig.LLPYTHON_IDX_S_MIN];
        double vMin = llpython[SensorConfig.LLPYTHON_IDX_V_MIN];

        if (hsvCalibrationRequested && isValidHsvTuple(hMin, hMax, sMin, vMin)) {
            boolean changedFromCurrent =
                    Math.abs(hMin - robotState.calibratedHMin) > HSV_DELTA_EPSILON
                            || Math.abs(hMax - robotState.calibratedHMax) > HSV_DELTA_EPSILON
                            || Math.abs(sMin - robotState.calibratedSMin) > HSV_DELTA_EPSILON
                            || Math.abs(vMin - robotState.calibratedVMin) > HSV_DELTA_EPSILON;
            boolean readyToCommit = (limelight == null)
                    || (hsvCalibrationSentToCamera
                            && (changedFromCurrent
                                    || hsvCalibrationPulseSec >= SensorConfig.VISION_PITCH_BLANKOUT_SEC));
            if (readyToCommit) {
                robotState.calibratedHMin = hMin;
                robotState.calibratedHMax = hMax;
                robotState.calibratedSMin = sMin;
                robotState.calibratedVMin = vMin;
                calibration.pollenHMin = hMin;
                calibration.pollenHMax = hMax;
                calibration.pollenSMin = sMin;
                calibration.pollenVMin = vMin;
                robotState.hsvCalibrationUpdated = true;
                hsvCalibrationRequested = false;
                hsvCalibrationSentToCamera = false;
                hsvCalibrationPulseSec = ZERO;
                populatePythonInputs();
            }
        }

        boolean detected = Double.isFinite(validFlag)
                && validFlag > VALID_FLAG_THRESHOLD
                && Double.isFinite(dxFwdM)
                && Double.isFinite(dyLeftM);
        if (detected) {
            robotState.pollenVisible = true;
            robotState.pollenRobotRelative.set(dxFwdM, dyLeftM);
            int parsedCount = Double.isFinite(countVal)
                    ? Math.max(ZERO_INT, (int) Math.floor(countVal + ROUND_HALF))
                    : ZERO_INT;
            robotState.pollenVisibleCount = parsedCount;
            return true;
        }

        clearPollenState();
        return false;
    }

    private void runIoWorkerLoop() {
        while (workerRunning) {
            boolean hasWork = pipelineDirty || pythonInputsDirty || orientationDirty;
            if (!hasWork) {
                LockSupport.park();
                continue;
            }

            boolean anyFailure = false;

            if (pipelineDirty) {
                int targetPipeline = desiredPipelineIndex;
                pipelineDirty = false;
                if (limelight.isConnected() && limelight.pipelineSwitch(targetPipeline)) {
                    confirmedPipelineIndex = targetPipeline;
                    if (desiredPipelineIndex != confirmedPipelineIndex) {
                        pipelineDirty = true;
                    }
                } else {
                    pipelineDirty = true;
                    anyFailure = true;
                }
            }

            if (pythonInputsDirty) {
                if (desiredPipelineIndex != SensorConfig.LIMELIGHT_PIPELINE_POLLEN) {
                    pythonInputsDirty = false;
                } else if (confirmedPipelineIndex != SensorConfig.LIMELIGHT_PIPELINE_POLLEN) {
                    anyFailure = true;
                } else {
                    pythonInputsDirty = false;
                    synchronized (pythonInputsLock) {
                        copyArray(stagedPythonInputs, workerPythonInputs);
                    }
                    if (limelight.isConnected() && limelight.updatePythonInputs(workerPythonInputs)) {
                        if (workerPythonInputs[SensorConfig.LLROBOT_IDX_CAL_TRIGGER]
                                > VALID_FLAG_THRESHOLD) {
                            hsvCalibrationSentToCamera = true;
                        }
                    } else {
                        pythonInputsDirty = true;
                        anyFailure = true;
                    }
                }
            }

            if (orientationDirty) {
                if (desiredPipelineIndex != SensorConfig.LIMELIGHT_PIPELINE_APRILTAG) {
                    orientationDirty = false;
                } else {
                    double orientationDeg = pendingOrientationDeg;
                    orientationDirty = false;
                    if (limelight.isConnected()) {
                        if (!limelight.updateRobotOrientation(orientationDeg)) {
                            orientationDirty = true;
                            anyFailure = true;
                        }
                    } else {
                        orientationDirty = true;
                        anyFailure = true;
                    }
                }
            }

            if (anyFailure && workerRunning) {
                LockSupport.parkNanos(WORKER_RETRY_PARK_NANOS);
            }
        }
    }

    private void readAprilTagResult(LLResult result, boolean blankoutActive) {
        if (result.getPipelineIndex() != SensorConfig.LIMELIGHT_PIPELINE_APRILTAG) {
            robotState.visionMeasurement.clear();
            return;
        }

        double totalLatencyMs = result.getCaptureLatency()
                + result.getTargetingLatency()
                + result.getParseLatency();
        long stalenessMs = result.getStaleness();

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        int primaryTagId = INVALID_TAG_ID;
        double bestTagArea = ZERO;
        int fiducialCount = (fiducials != null) ? fiducials.size() : ZERO_INT;
        double omegaRadPerSec = robotState.robotVelocity.omega;
        boolean omegaValid = Double.isFinite(omegaRadPerSec)
                && Math.abs(omegaRadPerSec) <= SensorConfig.VISION_MAX_OMEGA_RAD_PER_SEC;

        for (int i = ZERO_INT; i < fiducialCount; i++) {
            LLResultTypes.FiducialResult fiducial = fiducials.get(i);
            if (fiducial == null) {
                continue;
            }
            int fidId = fiducial.getFiducialId();
            double area = fiducial.getTargetArea();
            if (area >= bestTagArea) {
                bestTagArea = area;
                primaryTagId = fidId;
            }
            if (!blankoutActive
                    && omegaValid
                    && stalenessMs >= ZERO_LONG
                    && stalenessMs <= SensorConfig.VISION_MAX_STALENESS_MS
                    && totalLatencyMs >= ZERO
                    && totalLatencyMs <= SensorConfig.VISION_MAX_LATENCY_MS
                    && area >= SensorConfig.VISION_MIN_TAG_AREA_PERCENT) {
                tryConfirmInitialHiveTag(fidId);
            }
        }

        Pose3D botpose = result.getBotpose_MT2();
        if (botpose == null || botpose.getPosition() == null || botpose.getOrientation() == null) {
            botpose = result.getBotpose();
        }
        if (botpose == null || botpose.getPosition() == null || botpose.getOrientation() == null) {
            robotState.visionMeasurement.clear();
            return;
        }
        if (result.getBotposeTagCount() <= ZERO_INT && primaryTagId == INVALID_TAG_ID) {
            robotState.visionMeasurement.clear();
            return;
        }

        Position pos = botpose.getPosition();
        YawPitchRollAngles orient = botpose.getOrientation();
        double rawX = (pos.unit != null) ? pos.unit.toMeters(pos.x) : pos.x;
        double rawY = (pos.unit != null) ? pos.unit.toMeters(pos.y) : pos.y;
        double rawHeadingRad = orient.getYaw(AngleUnit.RADIANS);
        double tagAreaPercent = (bestTagArea > ZERO) ? bestTagArea : result.getBotposeAvgArea();
        if (tagAreaPercent <= ZERO) {
            tagAreaPercent = result.getTa();
        }
        double tagDistM = result.getBotposeAvgDist();

        processAprilTagObservation(
                rawX,
                rawY,
                rawHeadingRad,
                totalLatencyMs,
                stalenessMs,
                tagAreaPercent,
                tagDistM,
                primaryTagId);
    }

    private void readPollenResult(LLResult result) {
        if (result.getPipelineIndex() != SensorConfig.LIMELIGHT_PIPELINE_POLLEN) {
            clearPollenState();
            return;
        }
        long stalenessMs = result.getStaleness();
        if (stalenessMs < ZERO_LONG || stalenessMs > SensorConfig.VISION_MAX_STALENESS_MS) {
            clearPollenState();
            return;
        }
        processPollenPythonOutput(result.getPythonOutput());
    }

    private void tryConfirmInitialHiveTag(int tagId) {
        if (!initialHiveCheckEnabled || robotState.initialHiveTagConfirmed || tagId < ZERO_INT) {
            return;
        }
        FieldConfig.HiveCell cell = FieldConfig.classifyHiveTagCell(robotState.alliance, tagId);
        if (cell != null) {
            robotState.detectedInitialHiveCell = cell;
            robotState.initialHiveTagConfirmed = true;
            robotState.activeHiveCell = cell;
            robotState.refreshActiveHiveTarget();
        }
    }

    private void clearPollenState() {
        robotState.pollenVisible = false;
        robotState.pollenRobotRelative.set(ZERO, ZERO);
        robotState.pollenVisibleCount = ZERO_INT;
    }

    private double computeServoNormalized(double pitchDeg) {
        double normalized = SensorConfig.pitchDegToServoNormalized(
                pitchDeg,
                calibration.limelightDatumOffsetDeg,
                calibration.limelightServoSpanDeg,
                calibration.limelightServoDirSign);
        return MathUtil.clamp(
                normalized,
                calibration.limelightServoMinLimit,
                calibration.limelightServoMaxLimit);
    }

    private void populatePythonInputs() {
        double triggerValue = (hsvCalibrationRequested && !isBlankoutActive())
                ? CAL_TRIGGER_ON
                : CAL_TRIGGER_OFF;
        double roundedS = Math.floor(
                MathUtil.clamp(robotState.calibratedSMin, ZERO, HSV_SV_MAX) + ROUND_HALF);
        double roundedV = Math.floor(
                MathUtil.clamp(robotState.calibratedVMin, ZERO, HSV_SV_MAX) + ROUND_HALF);
        double packedSV = roundedS * SensorConfig.LLROBOT_SV_PACK_SCALE + roundedV;

        pythonInputs[SensorConfig.LLROBOT_IDX_PITCH_DEG] = currentPitchDeg;
        pythonInputs[SensorConfig.LLROBOT_IDX_CAM_HEIGHT_M] = calibration.camHeightM;
        pythonInputs[SensorConfig.LLROBOT_IDX_CAM_FWD_M] = calibration.camOffsetXM;
        pythonInputs[SensorConfig.LLROBOT_IDX_CAM_SIDE_M] = calibration.camOffsetYM;
        pythonInputs[SensorConfig.LLROBOT_IDX_CAL_TRIGGER] = triggerValue;
        pythonInputs[SensorConfig.LLROBOT_IDX_H_MIN] = robotState.calibratedHMin;
        pythonInputs[SensorConfig.LLROBOT_IDX_H_MAX] = robotState.calibratedHMax;
        pythonInputs[SensorConfig.LLROBOT_IDX_SV_PACKED] = packedSV;
    }

    private boolean hasPythonInputsChanged() {
        for (int i = ZERO_INT; i < SensorConfig.LLROBOT_LENGTH; i++) {
            double prev = lastQueuedPythonInputs[i];
            double curr = pythonInputs[i];
            if (Double.isNaN(prev) || Math.abs(curr - prev) >= PYTHON_INPUT_CHANGE_EPSILON) {
                return true;
            }
        }
        return false;
    }

    private void invalidateLastQueuedPythonInputs() {
        for (int i = ZERO_INT; i < SensorConfig.LLROBOT_LENGTH; i++) {
            lastQueuedPythonInputs[i] = Double.NaN;
        }
    }

    private static void copyArray(double[] src, double[] dst) {
        for (int i = ZERO_INT; i < SensorConfig.LLROBOT_LENGTH; i++) {
            dst[i] = src[i];
        }
    }

    private static boolean isValidHsvTuple(double hMin, double hMax, double sMin, double vMin) {
        return Double.isFinite(hMin)
                && Double.isFinite(hMax)
                && Double.isFinite(sMin)
                && Double.isFinite(vMin)
                && hMin >= ZERO
                && hMax <= HSV_HUE_MAX
                && hMax > hMin
                && sMin >= ZERO
                && sMin <= HSV_SV_MAX
                && vMin >= ZERO
                && vMin <= HSV_SV_MAX;
    }
}
