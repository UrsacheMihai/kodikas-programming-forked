package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.kodikas.ftclib.command.Subsystem;
import org.kodikas.ftclib.hardware.CachedMotor;
import org.kodikas.ftclib.hardware.CachedServo;
import org.kodikas.ftclib.hardware.PowerBudgeter;

public final class IntakeTransferSubsystem implements Subsystem {

    public static final int SUBSYSTEM_ID = 4;

    public interface ColorProximityProbe {
        double readDistanceMm();
        void readNormalizedRgbInto(float[] outRgb);
    }

    private static final double ZERO_DOUBLE = 0.0;
    private static final int ZERO_INT = 0;
    private static final int ONE_INT = 1;
    private static final long ZERO_LONG = 0L;
    private static final double DEFAULT_DT_SEC = 0.01;
    private static final int RGB_CHANNEL_COUNT = 3;
    private static final int IDX_RED = 0;
    private static final int IDX_GREEN = 1;
    private static final int IDX_BLUE = 2;
    private static final int IDX_HUE = 0;
    private static final int IDX_SAT = 1;
    private static final int IDX_VAL = 2;
    private static final float ZERO_FLOAT = 0.0f;
    private static final float HSV_EPSILON = 1.0e-5f;
    private static final float HUE_SECTOR_DEG = 60.0f;
    private static final float HUE_GREEN_OFFSET = 2.0f;
    private static final float HUE_BLUE_OFFSET = 4.0f;
    private static final float HUE_FULL_CIRCLE_DEG = 360.0f;

    private static final class HardwareColorProbe implements ColorProximityProbe {

        private final RevColorSensorV3 sensor;

        private HardwareColorProbe(RevColorSensorV3 sensor) {
            this.sensor = sensor;
            if (this.sensor != null) {
                this.sensor.setGain(SensorConfig.COLOR_SENSOR_GAIN);
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
            outRgb[IDX_GREEN] = rgba.green;
            outRgb[IDX_BLUE] = rgba.blue;
        }
    }

    private final CachedMotor transferMotor;
    private final CachedMotor outtakeMotor;
    private final CachedServo plowServo;
    private final ColorProximityProbe colorProbe;
    private final RobotState robotState;

    private final double plowLiftedPos;
    private final double plowDeployedPos;

    private final float[] scratchRgb = new float[RGB_CHANNEL_COUNT];
    private final float[] scratchHsv = new float[RGB_CHANNEL_COUNT];

    private IntakeState intakeState = IntakeState.IDLE;
    private int pollenCount = ZERO_INT;
    private boolean plowDeployed = false;
    private double outtakeFeedDisplacementM = ZERO_DOUBLE;
    private double pollTimerSec = ZERO_DOUBLE;
    private double timeSinceLastTriggerSec = SensorConfig.POLLEN_DEBOUNCE_SEC;
    private boolean sensorArmedForEntry = true;
    private long colorSensorPollCount = ZERO_LONG;
    private double lastDtSeconds = DEFAULT_DT_SEC;

    public IntakeTransferSubsystem(
            DcMotorEx rawTransferMotor,
            DcMotorEx rawOuttakeMotor,
            Servo rawPlowServo,
            RevColorSensorV3 rawColorSensor,
            PowerBudgeter powerBudgeter,
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration) {
        this(
                rawTransferMotor,
                rawOuttakeMotor,
                rawPlowServo,
                new HardwareColorProbe(rawColorSensor),
                powerBudgeter,
                robotState,
                calibration);
    }

    public IntakeTransferSubsystem(
            DcMotorEx rawTransferMotor,
            DcMotorEx rawOuttakeMotor,
            Servo rawPlowServo,
            ColorProximityProbe colorProbe,
            PowerBudgeter powerBudgeter,
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration) {
        this.robotState = robotState;
        this.colorProbe = colorProbe;

        CalibrationStore.ActiveCalibration activeCal = (calibration != null)
                ? calibration
                : new CalibrationStore.ActiveCalibration();
        activeCal.validateAndSanitize();

        this.plowLiftedPos = activeCal.plowLiftedPos;
        this.plowDeployedPos = activeCal.plowDeployedPos;

        this.transferMotor = new CachedMotor(
                rawTransferMotor,
                PowerBudgeter.PriorityTier.AUXILIARY,
                powerBudgeter,
                IntakeConfig.TRANSFER_STARTUP_SLEW_PER_SEC);
        this.transferMotor.configure(
                IntakeConfig.TRANSFER_DIRECTION,
                IntakeConfig.TRANSFER_RUN_MODE,
                IntakeConfig.TRANSFER_ZERO_POWER);

        this.outtakeMotor = new CachedMotor(
                rawOuttakeMotor,
                PowerBudgeter.PriorityTier.AUXILIARY,
                powerBudgeter,
                IntakeConfig.OUTTAKE_SLEW_PER_SEC);
        this.outtakeMotor.configure(
                IntakeConfig.OUTTAKE_DIRECTION,
                IntakeConfig.OUTTAKE_RUN_MODE,
                IntakeConfig.OUTTAKE_ZERO_POWER);

        this.plowServo = new CachedServo(
                rawPlowServo,
                IntakeConfig.PLOW_MIN_LIMIT,
                IntakeConfig.PLOW_MAX_LIMIT,
                activeCal.plowServoReversed);
        this.plowServo.setTargetPosition(plowLiftedPos);
    }

    public IntakeTransferSubsystem(
            ColorProximityProbe colorProbe,
            PowerBudgeter powerBudgeter,
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration) {
        this(null, null, null, colorProbe, powerBudgeter, robotState, calibration);
    }

    @Override
    public int subsystemId() {
        return SUBSYSTEM_ID;
    }

    @Override
    public void readSensors(double dtSeconds) {
        double safeDt = (Double.isFinite(dtSeconds) && dtSeconds > ZERO_DOUBLE)
                ? dtSeconds
                : DEFAULT_DT_SEC;
        timeSinceLastTriggerSec += safeDt;

        boolean pollingAllowed = (intakeState == IntakeState.INTAKE
                || intakeState == IntakeState.REVERSING);
        if (!pollingAllowed) {
            pollTimerSec = ZERO_DOUBLE;
            return;
        }

        pollTimerSec += safeDt;
        if (pollTimerSec < SensorConfig.COLOR_SENSOR_POLL_INTERVAL_SEC) {
            return;
        }
        pollTimerSec = ZERO_DOUBLE;

        if (colorProbe == null) {
            return;
        }

        colorSensorPollCount++;
        robotState.colorSensorPollCount = colorSensorPollCount;
        double distMm = colorProbe.readDistanceMm();
        robotState.lastColorDistanceMm = distMm;

        if (!Double.isFinite(distMm)) {
            return;
        }

        if (distMm >= SensorConfig.POLLEN_EXIT_DISTANCE_MM) {
            sensorArmedForEntry = true;
            return;
        }

        if (sensorArmedForEntry
                && distMm <= SensorConfig.POLLEN_ENTER_DISTANCE_MM
                && timeSinceLastTriggerSec >= SensorConfig.POLLEN_DEBOUNCE_SEC) {
            sensorArmedForEntry = false;
            timeSinceLastTriggerSec = ZERO_DOUBLE;
            colorProbe.readNormalizedRgbInto(scratchRgb);
            rgbToHsvInto(scratchRgb[IDX_RED], scratchRgb[IDX_GREEN], scratchRgb[IDX_BLUE], scratchHsv);
            boolean isYellowPollen = scratchHsv[IDX_HUE] >= SensorConfig.POLLEN_MIN_HUE_DEG
                    && scratchHsv[IDX_HUE] <= SensorConfig.POLLEN_MAX_HUE_DEG
                    && scratchHsv[IDX_SAT] >= SensorConfig.POLLEN_MIN_SATURATION;
            if (isYellowPollen) {
                if (intakeState == IntakeState.INTAKE) {
                    pollenCount = Math.min(FieldConfig.MAX_POLLEN_COUNT, pollenCount + ONE_INT);
                    if (pollenCount >= FieldConfig.MAX_POLLEN_COUNT) {
                        intakeState = IntakeState.FULL;
                    }
                } else if (intakeState == IntakeState.REVERSING) {
                    pollenCount = Math.max(ZERO_INT, pollenCount - ONE_INT);
                }
            }
        }
    }

    @Override
    public void compute(double dtSeconds) {
        double safeDt = (Double.isFinite(dtSeconds) && dtSeconds > ZERO_DOUBLE)
                ? dtSeconds
                : DEFAULT_DT_SEC;
        lastDtSeconds = safeDt;

        double transferPower = ZERO_DOUBLE;
        double outtakePower = ZERO_DOUBLE;

        if (intakeState == IntakeState.INTAKE) {
            if (pollenCount >= FieldConfig.MAX_POLLEN_COUNT) {
                intakeState = IntakeState.FULL;
            } else {
                transferPower = IntakeConfig.TRANSFER_INTAKE_POWER;
            }
        } else if (intakeState == IntakeState.REVERSING) {
            transferPower = -IntakeConfig.TRANSFER_REVERSE_POWER;
            outtakePower = -IntakeConfig.OUTTAKE_REVERSE_POWER;
        } else if (intakeState == IntakeState.FEEDINGTOTURRET) {
            if (robotState.readyToFire) {
                transferPower = IntakeConfig.TRANSFER_FEED_POWER;
                outtakePower = IntakeConfig.OUTTAKE_FEED_POWER;
                double vRatio = Math.max(ZERO_DOUBLE, robotState.batteryVoltage)
                        / IntakeConfig.NOMINAL_BATTERY_VOLTAGE;
                double appliedOuttake = Math.abs(outtakeMotor.getLastAppliedPower());
                outtakeFeedDisplacementM += IntakeConfig.OUTTAKE_NOMINAL_SURFACE_SPEED_M_PER_S
                        * appliedOuttake
                        * vRatio
                        * safeDt;
                if (outtakeFeedDisplacementM >= IntakeConfig.L_TRANSFER_CLEAR_M) {
                    int volleyScored = pollenCount;
                    pollenCount = ZERO_INT;
                    outtakeFeedDisplacementM = ZERO_DOUBLE;
                    intakeState = IntakeState.IDLE;
                    sensorArmedForEntry = true;
                    timeSinceLastTriggerSec = SensorConfig.POLLEN_DEBOUNCE_SEC;
                    transferPower = ZERO_DOUBLE;
                    outtakePower = ZERO_DOUBLE;
                    robotState.recordScoredPollen(volleyScored);
                }
            }
        }

        transferMotor.setTargetPower(transferPower);
        outtakeMotor.setTargetPower(outtakePower);
        plowServo.setTargetPosition(plowDeployed ? plowDeployedPos : plowLiftedPos);

        robotState.intakeState = intakeState;
        robotState.pollenCount = pollenCount;
        robotState.plowDeployed = plowDeployed;
        robotState.outtakeFeedDisplacementM = outtakeFeedDisplacementM;
    }

    @Override
    public void writeActuators() {
        transferMotor.flush(lastDtSeconds);
        outtakeMotor.flush(lastDtSeconds);
        plowServo.flush();
    }

    public void requestState(IntakeState newState) {
        if (newState == null) {
            return;
        }
        if (newState == IntakeState.INTAKE && pollenCount >= FieldConfig.MAX_POLLEN_COUNT) {
            this.intakeState = IntakeState.FULL;
            robotState.intakeState = this.intakeState;
            return;
        }
        if (newState == IntakeState.FEEDINGTOTURRET && this.intakeState != IntakeState.FEEDINGTOTURRET) {
            this.outtakeFeedDisplacementM = ZERO_DOUBLE;
        }
        if (newState != IntakeState.FEEDINGTOTURRET) {
            this.outtakeFeedDisplacementM = ZERO_DOUBLE;
        }
        this.intakeState = newState;
        robotState.intakeState = this.intakeState;
    }

    public void toggleIntake() {
        if (intakeState == IntakeState.INTAKE) {
            requestState(IntakeState.IDLE);
        } else if (intakeState == IntakeState.FULL) {
            pollenCount = Math.max(ZERO_INT, FieldConfig.MAX_POLLEN_COUNT - ONE_INT);
            robotState.pollenCount = pollenCount;
            sensorArmedForEntry = true;
            requestState(IntakeState.INTAKE);
        } else {
            requestState(IntakeState.INTAKE);
        }
    }

    public void startFeeding() {
        requestState(IntakeState.FEEDINGTOTURRET);
    }

    public void setPlowDeployed(boolean deployed) {
        this.plowDeployed = deployed;
        robotState.plowDeployed = deployed;
        plowServo.setTargetPosition(deployed ? plowDeployedPos : plowLiftedPos);
    }

    public void togglePlow() {
        setPlowDeployed(!plowDeployed);
    }

    public void setPollenCount(int count) {
        int clamped = Math.max(ZERO_INT, Math.min(FieldConfig.MAX_POLLEN_COUNT, count));
        this.pollenCount = clamped;
        robotState.pollenCount = clamped;
        if (clamped >= FieldConfig.MAX_POLLEN_COUNT && intakeState == IntakeState.INTAKE) {
            intakeState = IntakeState.FULL;
            robotState.intakeState = intakeState;
        }
    }

    public static void rgbToHsvInto(float r, float g, float b, float[] outHsv) {
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;
        float hueDeg;
        if (delta <= HSV_EPSILON) {
            hueDeg = ZERO_FLOAT;
        } else if (max == r) {
            hueDeg = HUE_SECTOR_DEG * ((g - b) / delta);
        } else if (max == g) {
            hueDeg = HUE_SECTOR_DEG * (((b - r) / delta) + HUE_GREEN_OFFSET);
        } else {
            hueDeg = HUE_SECTOR_DEG * (((r - g) / delta) + HUE_BLUE_OFFSET);
        }
        if (hueDeg < ZERO_FLOAT) {
            hueDeg += HUE_FULL_CIRCLE_DEG;
        }
        float saturation = (max > HSV_EPSILON) ? (delta / max) : ZERO_FLOAT;
        outHsv[IDX_HUE] = hueDeg;
        outHsv[IDX_SAT] = saturation;
        outHsv[IDX_VAL] = max;
    }

    public IntakeState getState() {
        return intakeState;
    }

    public int getPollenCount() {
        return pollenCount;
    }

    public boolean isPlowDeployed() {
        return plowDeployed;
    }

    public double getOuttakeFeedDisplacementM() {
        return outtakeFeedDisplacementM;
    }

    public long getColorSensorPollCount() {
        return colorSensorPollCount;
    }

    public CachedMotor getTransferMotor() {
        return transferMotor;
    }

    public CachedMotor getOuttakeMotor() {
        return outtakeMotor;
    }

    public CachedServo getPlowServo() {
        return plowServo;
    }
}
