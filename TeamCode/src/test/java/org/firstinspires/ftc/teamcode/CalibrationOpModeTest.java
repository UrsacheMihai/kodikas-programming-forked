package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.File;
import java.io.IOException;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.opmodes.calibration.CalibrationOpMode;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.kodikas.ftclib.ballistics.BallisticProfile;
import org.kodikas.ftclib.calibration.CameraExtrinsicSolver;
import org.kodikas.ftclib.calibration.DeadWheelSpinCalibrator;
import org.kodikas.ftclib.calibration.TurretAimCalibrator;
import org.kodikas.ftclib.math.MathUtil;

public final class CalibrationOpModeTest {

    private static final int ZERO_INT = 0;
    private static final int ONE_INT = 1;
    private static final int TWO_INT = 2;
    private static final int THREE_INT = 3;
    private static final int FOUR_INT = 4;
    private static final int IMU_AUDIT_TICKS = 500;
    private static final int SPIN_SAMPLE_STEPS = 100;
    private static final int CLAMP_PUSH_STEPS = 12;
    private static final int BLANKOUT_CLEAR_TICKS = 8;

    private static final double ZERO_DOUBLE = 0.0;
    private static final double ONE_DOUBLE = 1.0;
    private static final double NEG_ONE_DOUBLE = -1.0;
    private static final double TWO_DOUBLE = 2.0;
    private static final double EPS = 1.0e-4;
    private static final double TIGHT_EPS = 1.0e-5;
    private static final double MM_TOLERANCE = 0.25;
    private static final double DT_TWENTY_MS = 0.020;
    private static final double DT_TEN_MS = 0.010;

    private static final double PUSH_DISTANCE_M = 0.20;
    private static final double TRUE_FWD_OFFSET_MM = -84.0;
    private static final double TRUE_STRAFE_OFFSET_MM = -168.0;
    private static final double SPIN_REVOLUTIONS = 2.5;
    private static final double REQUIRED_MIN_R2 = 0.999;
    private static final double CAMERA_MIN_R2 = 0.998;

    private static final double TRUE_TURRET_CENTER_PWM = 0.512;
    private static final double TRUE_TURRET_GEAR_RATIO = 2.75;
    private static final double TRUE_TURRET_OFFSET_X_M = -0.055;
    private static final double TURRET_MARKER_DIST_M = 2.0;
    private static final double[] TURRET_TEST_HEADINGS_RAD = new double[] {
            0.0, 0.35, -0.35, 0.60, -0.60
    };

    private static final double LUT_MODIFIED_DIST_KNOT_0 = 1.02;
    private static final double LUT_STEP_DIST_COARSE = 0.10;
    private static final double LUT_STEP_DIST_FINE = 0.02;
    private static final double LUT_STEP_TOF_COARSE = 0.02;
    private static final double LUT_STEP_TOF_FINE = 0.005;
    private static final double LUT_STEP_PWR_COARSE = 0.02;
    private static final double LUT_STEP_PWR_FINE = 0.005;
    private static final float TRIGGER_SPIN_INPUT = 0.8f;
    private static final float ZERO_FLOAT = 0.0f;

    private static final double REF_TAG_X_M = 1.50;
    private static final double REF_TAG_Y_M = 0.0;
    private static final double REF_TAG_Z_M = 0.55;
    private static final double TRUE_CAM_X_M = 0.172;
    private static final double TRUE_CAM_Y_M = -0.012;
    private static final double TRUE_CAM_Z_M = 0.292;
    private static final double TRUE_DATUM_OFFSET_DEG = 1.5;
    private static final double TRUE_SERVO_SPAN_DEG = 175.0;
    private static final double[] EXTRINSIC_TEST_PWMS = new double[] {
            0.42, 0.48, 0.54, 0.60
    };

    private static final double CAL_PY_VALID = 1.0;
    private static final double CAL_PY_DX = 0.65;
    private static final double CAL_PY_DY = 0.02;
    private static final double CAL_PY_COUNT = 2.0;
    private static final double CAL_H_MIN = 15.5;
    private static final double CAL_H_MAX = 28.5;
    private static final double CAL_S_MIN = 175.0;
    private static final double CAL_V_MIN = 80.0;

    private static final double POLLEN_ENTER_MM = 28.0;
    private static final float YELLOW_R = 0.90f;
    private static final float YELLOW_G = 0.70f;
    private static final float YELLOW_B = 0.10f;

    private static final String CAL_FILE_NAME = "kodi_calibration.json";

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    private static final class MockPinpointProbe implements CalibrationOpMode.PinpointProbe {

        private int encoderX = ZERO_INT;
        private int encoderY = ZERO_INT;
        private double unwrappedHeadingRad = ZERO_DOUBLE;

        @Override
        public void update() {
        }

        @Override
        public int getEncoderX() {
            return encoderX;
        }

        @Override
        public int getEncoderY() {
            return encoderY;
        }

        @Override
        public double getUnwrappedHeadingRad() {
            return unwrappedHeadingRad;
        }

        @Override
        public void reset() {
            encoderX = ZERO_INT;
            encoderY = ZERO_INT;
            unwrappedHeadingRad = ZERO_DOUBLE;
        }

        private void setState(int encoderX, int encoderY, double unwrappedHeadingRad) {
            this.encoderX = encoderX;
            this.encoderY = encoderY;
            this.unwrappedHeadingRad = unwrappedHeadingRad;
        }
    }

    private static final class MockAprilTagProbe implements CalibrationOpMode.AprilTagExtrinsicProbe {

        private final double[] sample = new double[CalibrationOpMode.EXTRINSIC_BUF_LEN];

        private void setObservation(
                double camToTagX,
                double camToTagY,
                double camToTagZ,
                double pitchRad) {
            sample[CalibrationOpMode.EXTRINSIC_IDX_VALID] = ONE_DOUBLE;
            sample[CalibrationOpMode.EXTRINSIC_IDX_CAM_TO_TAG_X] = camToTagX;
            sample[CalibrationOpMode.EXTRINSIC_IDX_CAM_TO_TAG_Y] = camToTagY;
            sample[CalibrationOpMode.EXTRINSIC_IDX_CAM_TO_TAG_Z] = camToTagZ;
            sample[CalibrationOpMode.EXTRINSIC_IDX_MEASURED_PITCH_RAD] = pitchRad;
        }

        @Override
        public boolean readCameraSpaceObservation(double[] outBuf) {
            System.arraycopy(sample, ZERO_INT, outBuf, ZERO_INT, CalibrationOpMode.EXTRINSIC_BUF_LEN);
            return true;
        }
    }

    private static final class MockColorProbe implements IntakeTransferSubsystem.ColorProximityProbe {

        private double distanceMm = POLLEN_ENTER_MM;
        private float red = YELLOW_R;
        private float green = YELLOW_G;
        private float blue = YELLOW_B;

        @Override
        public double readDistanceMm() {
            return distanceMm;
        }

        @Override
        public void readNormalizedRgbInto(float[] outRgb) {
            outRgb[ZERO_INT] = red;
            outRgb[ONE_INT] = green;
            outRgb[TWO_INT] = blue;
        }
    }

    @Test
    public void verifyMode1PinpointImuDriftPolarityAndOlsSpinRegression() throws IOException {
        File calFile = tempFolder.newFile(CAL_FILE_NAME);
        MockPinpointProbe pinpointProbe = new MockPinpointProbe();
        Gamepad gamepad = new Gamepad();

        CalibrationOpMode opMode = new CalibrationOpMode();
        opMode.telemetry = null;
        opMode.setCalibrationFileForTest(calFile);
        opMode.setGamepadForTest(gamepad);
        opMode.setPinpointProbeForTest(pinpointProbe);
        opMode.initialize();
        opMode.startMatch();

        assertEquals(CalibrationOpMode.CalibrationMode.MODE_1_PINPOINT, opMode.getActiveMode());
        assertEquals(CalibrationOpMode.PinpointStep.IMU_DRIFT_AUDIT, opMode.getPinpointStep());

        executeMode1CalibrationSequence(opMode, gamepad, pinpointProbe);

        DeadWheelSpinCalibrator.SpinCalibrationResult res =
                opMode.getDeadWheelCalibrator().getResult();
        CalibrationStore.ActiveCalibration cal = opMode.getWorkingCalibration();

        assertTrue(opMode.isImuAuditPassed());
        assertTrue(res.polarityValid);
        assertTrue(cal.pinpointForwardReversed);
        assertFalse(cal.pinpointStrafeReversed);
        assertTrue(res.regressionPassed);
        assertTrue(res.r2Forward > REQUIRED_MIN_R2);
        assertTrue(res.r2Strafe > REQUIRED_MIN_R2);
        assertEquals(TRUE_FWD_OFFSET_MM, cal.pinpointForwardOffsetMm, MM_TOLERANCE);
        assertEquals(TRUE_STRAFE_OFFSET_MM, cal.pinpointStrafeOffsetMm, MM_TOLERANCE);

        opMode.stopMatch();
    }

    @Test
    public void verifyMode2TurretScaleVerifierAndFourServoTrimmer() throws IOException {
        File calFile = tempFolder.newFile(CAL_FILE_NAME);
        MockPinpointProbe pinpointProbe = new MockPinpointProbe();
        Gamepad gamepad = new Gamepad();

        CalibrationOpMode opMode = new CalibrationOpMode();
        opMode.telemetry = null;
        opMode.setCalibrationFileForTest(calFile);
        opMode.setGamepadForTest(gamepad);
        opMode.setPinpointProbeForTest(pinpointProbe);
        opMode.initialize();
        opMode.startMatch();

        pressRightBumper(opMode, gamepad);
        assertEquals(CalibrationOpMode.CalibrationMode.MODE_2_TURRET_SERVOS, opMode.getActiveMode());

        CalibrationStore.ActiveCalibration cal = opMode.getWorkingCalibration();
        double initialCenter = cal.turretCenterPwm;
        pressDpadUp(opMode, gamepad);
        pressDpadRight(opMode, gamepad);
        double expectedTrimmedCenter = initialCenter
                + CalibrationOpMode.COARSE_SERVO_STEP
                + CalibrationOpMode.FINE_SERVO_STEP;
        assertEquals(expectedTrimmedCenter, cal.turretCenterPwm, TIGHT_EPS);
        assertEquals(expectedTrimmedCenter, opMode.getTurretServo().getLastWrittenPosition(), TIGHT_EPS);

        pressSquare(opMode, gamepad);
        assertEquals(CalibrationOpMode.ServoSelection.HOOD_MIN, opMode.getSelectedServo());
        for (int i = ZERO_INT; i < CLAMP_PUSH_STEPS; i++) {
            pressDpadDown(opMode, gamepad);
        }
        assertEquals(TurretConfig.HOOD_MIN_CLAMP, cal.hoodMinClamp, TIGHT_EPS);
        assertTrue(opMode.getHoodServo().getLastWrittenPosition() >= TurretConfig.HOOD_MIN_CLAMP - TIGHT_EPS);

        pressSquare(opMode, gamepad);
        assertEquals(CalibrationOpMode.ServoSelection.HOOD_MAX, opMode.getSelectedServo());
        for (int i = ZERO_INT; i < CLAMP_PUSH_STEPS; i++) {
            pressDpadUp(opMode, gamepad);
        }
        assertEquals(TurretConfig.HOOD_MAX_CLAMP, cal.hoodMaxClamp, TIGHT_EPS);
        assertTrue(opMode.getHoodServo().getLastWrittenPosition() <= TurretConfig.HOOD_MAX_CLAMP + TIGHT_EPS);

        pressSquare(opMode, gamepad);
        assertEquals(CalibrationOpMode.ServoSelection.PLOW_LIFTED, opMode.getSelectedServo());
        pressDpadUp(opMode, gamepad);
        pressDpadRight(opMode, gamepad);
        double expectedPlowLifted = IntakeConfig.PLOW_LIFTED_POSITION
                + CalibrationOpMode.COARSE_SERVO_STEP
                + CalibrationOpMode.FINE_SERVO_STEP;
        assertEquals(expectedPlowLifted, cal.plowLiftedPos, TIGHT_EPS);
        assertEquals(expectedPlowLifted, opMode.getPlowServo().getLastWrittenPosition(), TIGHT_EPS);

        pressSquare(opMode, gamepad);
        assertEquals(CalibrationOpMode.ServoSelection.PLOW_DEPLOYED, opMode.getSelectedServo());
        pressDpadDown(opMode, gamepad);
        pressDpadLeft(opMode, gamepad);
        double expectedPlowDeployed = IntakeConfig.PLOW_DEPLOYED_POSITION
                - CalibrationOpMode.COARSE_SERVO_STEP
                - CalibrationOpMode.FINE_SERVO_STEP;
        assertEquals(expectedPlowDeployed, cal.plowDeployedPos, TIGHT_EPS);
        assertEquals(expectedPlowDeployed, opMode.getPlowServo().getLastWrittenPosition(), TIGHT_EPS);

        pressSquare(opMode, gamepad);
        assertEquals(CalibrationOpMode.ServoSelection.LIMELIGHT_DATUM_CHECK, opMode.getSelectedServo());
        assertEquals(
                SensorConfig.LIMELIGHT_SERVO_DATUM_NORMALIZED,
                opMode.getLimelightServo().getLastWrittenPosition(),
                TIGHT_EPS);

        executeMode2TurretSolveSequence(opMode, gamepad, pinpointProbe);

        TurretAimCalibrator.TurretCalibrationResult result =
                opMode.getTurretCalibrator().getResult();
        assertTrue(result.valid);
        assertTrue(result.r2Fit > REQUIRED_MIN_R2);
        assertEquals(TRUE_TURRET_CENTER_PWM, cal.turretCenterPwm, EPS);
        assertEquals(TRUE_TURRET_GEAR_RATIO, cal.turretGearRatio, EPS);
        assertEquals(TRUE_TURRET_OFFSET_X_M, cal.turretPivotOffsetXM, EPS);

        opMode.stopMatch();
    }

    @Test
    public void verifyMode3FourDistanceBallisticLutTunerAndTestFire() throws IOException {
        File calFile = tempFolder.newFile(CAL_FILE_NAME);
        Gamepad gamepad = new Gamepad();

        CalibrationOpMode opMode = new CalibrationOpMode();
        opMode.telemetry = null;
        opMode.setCalibrationFileForTest(calFile);
        opMode.setGamepadForTest(gamepad);
        opMode.initialize();
        opMode.startMatch();

        pressRightBumper(opMode, gamepad);
        pressRightBumper(opMode, gamepad);
        assertEquals(CalibrationOpMode.CalibrationMode.MODE_3_BALLISTIC_LUT, opMode.getActiveMode());

        CalibrationStore.ActiveCalibration cal = opMode.getWorkingCalibration();
        assertEquals(ZERO_INT, opMode.getLutKnotIndex());
        assertEquals(CalibrationOpMode.LutField.DISTANCE_M, opMode.getLutField());

        pressDpadUp(opMode, gamepad);
        pressDpadRight(opMode, gamepad);
        assertEquals(
                TurretConfig.DEFAULT_LUT_DISTANCES_M[ZERO_INT]
                        + LUT_STEP_DIST_COARSE
                        + LUT_STEP_DIST_FINE,
                cal.lutDistancesM[ZERO_INT],
                TIGHT_EPS);
        assertEquals(LUT_MODIFIED_DIST_KNOT_0, cal.lutDistancesM[ZERO_INT], TIGHT_EPS);

        pressTriangle(opMode, gamepad);
        assertEquals(CalibrationOpMode.LutField.HOOD_PITCH, opMode.getLutField());
        pressDpadUp(opMode, gamepad);
        assertEquals(
                TurretConfig.DEFAULT_LUT_HOOD_PITCHES[ZERO_INT] + CalibrationOpMode.COARSE_SERVO_STEP,
                cal.lutHoodPitches[ZERO_INT],
                TIGHT_EPS);
        assertTrue(opMode.getHoodServo().getLastWrittenPosition() >= TurretConfig.HOOD_MIN_CLAMP - TIGHT_EPS);
        assertTrue(opMode.getHoodServo().getLastWrittenPosition() <= TurretConfig.HOOD_MAX_CLAMP + TIGHT_EPS);

        pressTriangle(opMode, gamepad);
        assertEquals(CalibrationOpMode.LutField.TOF_SEC, opMode.getLutField());
        pressDpadUp(opMode, gamepad);
        pressDpadRight(opMode, gamepad);
        double expectedTof0 = TurretConfig.DEFAULT_LUT_TOF_SEC[ZERO_INT]
                + LUT_STEP_TOF_COARSE
                + LUT_STEP_TOF_FINE;
        assertEquals(expectedTof0, cal.lutTofSec[ZERO_INT], TIGHT_EPS);

        pressTriangle(opMode, gamepad);
        assertEquals(CalibrationOpMode.LutField.SCORING_POWER, opMode.getLutField());
        pressDpadUp(opMode, gamepad);
        pressDpadLeft(opMode, gamepad);
        double expectedPwr0 = TurretConfig.DEFAULT_LUT_SCORING_POWERS[ZERO_INT]
                + LUT_STEP_PWR_COARSE
                - LUT_STEP_PWR_FINE;
        assertEquals(expectedPwr0, cal.lutScoringPowers[ZERO_INT], TIGHT_EPS);

        BallisticProfile profile = opMode.getActiveBallisticProfile();
        assertEquals(expectedTof0, profile.evaluateTimeOfFlight(cal.lutDistancesM[ZERO_INT]), TIGHT_EPS);
        assertEquals(
                expectedPwr0,
                profile.computeTargetFlywheelPower(cal.lutDistancesM[ZERO_INT], FOUR_INT, false),
                TIGHT_EPS);

        for (int knot = ONE_INT; knot < TurretConfig.LUT_KNOT_COUNT; knot++) {
            pressSquare(opMode, gamepad);
            assertEquals(knot, opMode.getLutKnotIndex());
            pressDpadRight(opMode, gamepad);
            assertTrue(cal.lutDistancesM[knot] > cal.lutDistancesM[knot - ONE_INT]);
        }

        gamepad.right_trigger = TRIGGER_SPIN_INPUT;
        opMode.tickLoop(DT_TWENTY_MS);
        double expectedFlyPower = cal.lutScoringPowers[opMode.getLutKnotIndex()];
        assertEquals(expectedFlyPower, opMode.getLeftFlywheel().getLastAppliedPower(), EPS);
        assertEquals(expectedFlyPower, opMode.getRightFlywheel().getLastAppliedPower(), EPS);
        assertEquals(ZERO_DOUBLE, opMode.getOuttakeMotor().getLastAppliedPower(), TIGHT_EPS);

        gamepad.cross = true;
        opMode.tickLoop(DT_TWENTY_MS);
        assertEquals(IntakeConfig.OUTTAKE_FEED_POWER, opMode.getOuttakeMotor().getLastAppliedPower(), TIGHT_EPS);

        gamepad.cross = false;
        gamepad.right_trigger = ZERO_FLOAT;
        opMode.tickLoop(DT_TWENTY_MS);
        assertEquals(ZERO_DOUBLE, opMode.getLeftFlywheel().getLastAppliedPower(), TIGHT_EPS);
        assertEquals(ZERO_DOUBLE, opMode.getOuttakeMotor().getLastAppliedPower(), TIGHT_EPS);

        opMode.stopMatch();
    }

    @Test
    public void verifyMode4CameraExtrinsicsOneTapHsvColorVerifierAndAtomicSave() throws IOException {
        File calFile = tempFolder.newFile(CAL_FILE_NAME);
        MockPinpointProbe pinpointProbe = new MockPinpointProbe();
        MockAprilTagProbe aprilTagProbe = new MockAprilTagProbe();
        MockColorProbe colorProbe = new MockColorProbe();
        Gamepad gamepad = new Gamepad();

        CalibrationOpMode opMode = new CalibrationOpMode();
        opMode.telemetry = null;
        opMode.setCalibrationFileForTest(calFile);
        opMode.setGamepadForTest(gamepad);
        opMode.setPinpointProbeForTest(pinpointProbe);
        opMode.setAprilTagProbeForTest(aprilTagProbe);
        opMode.setColorProbeForTest(colorProbe);
        opMode.initialize();
        opMode.startMatch();

        executeMode1CalibrationSequence(opMode, gamepad, pinpointProbe);

        pressRightBumper(opMode, gamepad);
        assertEquals(CalibrationOpMode.CalibrationMode.MODE_2_TURRET_SERVOS, opMode.getActiveMode());
        executeMode2TurretSolveSequence(opMode, gamepad, pinpointProbe);

        pressRightBumper(opMode, gamepad);
        assertEquals(CalibrationOpMode.CalibrationMode.MODE_3_BALLISTIC_LUT, opMode.getActiveMode());
        pressDpadUp(opMode, gamepad);
        pressDpadRight(opMode, gamepad);

        pressRightBumper(opMode, gamepad);
        assertEquals(CalibrationOpMode.CalibrationMode.MODE_4_VISION_COLOR, opMode.getActiveMode());

        opMode.setReferenceAprilTagPoseMeters(REF_TAG_X_M, REF_TAG_Y_M, REF_TAG_Z_M);
        double trueDatumRad = Math.toRadians(TRUE_DATUM_OFFSET_DEG);
        double trueSlopeRad = Math.toRadians(TRUE_SERVO_SPAN_DEG);

        for (int i = ZERO_INT; i < EXTRINSIC_TEST_PWMS.length; i++) {
            double pwm = EXTRINSIC_TEST_PWMS[i];
            double pitchRad = trueDatumRad + trueSlopeRad * (pwm - SensorConfig.LIMELIGHT_SERVO_DATUM_NORMALIZED);
            opMode.setLiveLimelightPwm(pwm);
            aprilTagProbe.setObservation(
                    REF_TAG_X_M - TRUE_CAM_X_M,
                    REF_TAG_Y_M - TRUE_CAM_Y_M,
                    REF_TAG_Z_M - TRUE_CAM_Z_M,
                    pitchRad);
            pressCross(opMode, gamepad);
        }

        pressCircle(opMode, gamepad);
        CameraExtrinsicSolver.CameraExtrinsicResult camRes =
                opMode.getCameraExtrinsicSolver().getResult();
        CalibrationStore.ActiveCalibration cal = opMode.getWorkingCalibration();

        assertTrue(camRes.valid);
        assertTrue(camRes.pitchR2 > CAMERA_MIN_R2);
        assertEquals(TRUE_CAM_X_M, cal.camOffsetXM, EPS);
        assertEquals(TRUE_CAM_Y_M, cal.camOffsetYM, EPS);
        assertEquals(TRUE_CAM_Z_M, cal.camHeightM, EPS);
        assertEquals(TRUE_DATUM_OFFSET_DEG, cal.limelightDatumOffsetDeg, EPS);
        assertEquals(TRUE_SERVO_SPAN_DEG, cal.limelightServoSpanDeg, EPS);

        pressSquare(opMode, gamepad);
        assertTrue(opMode.isHsvCalibrationActive());
        assertEquals(VisionSubsystem.VisionMode.POLLEN_FLOOR, opMode.getVision().getMode());

        for (int i = ZERO_INT; i < BLANKOUT_CLEAR_TICKS; i++) {
            opMode.tickLoop(DT_TWENTY_MS);
        }
        assertFalse(opMode.getVision().isBlankoutActive());

        double[] pyOutput = new double[] {
                CAL_PY_VALID,
                CAL_PY_DX,
                CAL_PY_DY,
                CAL_PY_COUNT,
                CAL_H_MIN,
                CAL_H_MAX,
                CAL_S_MIN,
                CAL_V_MIN
        };
        assertTrue(opMode.getVision().processPollenPythonOutput(pyOutput));
        opMode.tickLoop(DT_TWENTY_MS);

        assertFalse(opMode.isHsvCalibrationActive());
        assertEquals(CAL_H_MIN, cal.pollenHMin, TIGHT_EPS);
        assertEquals(CAL_H_MAX, cal.pollenHMax, TIGHT_EPS);
        assertEquals(CAL_S_MIN, cal.pollenSMin, TIGHT_EPS);
        assertEquals(CAL_V_MIN, cal.pollenVMin, TIGHT_EPS);

        gamepad.triangle = true;
        opMode.tickLoop(DT_TEN_MS);
        assertTrue(opMode.isColorVerifyActive());
        assertEquals(ZERO_INT, (int) opMode.getColorVerifyPollCount());
        gamepad.triangle = false;
        opMode.tickLoop(DT_TEN_MS);
        assertEquals(ZERO_INT, (int) opMode.getColorVerifyPollCount());
        opMode.tickLoop(DT_TEN_MS);
        assertEquals(ONE_INT, (int) opMode.getColorVerifyPollCount());
        assertTrue(opMode.isLastColorEnterActive());
        assertTrue(opMode.isLastColorYellowMatch());
        assertFalse(opMode.isLastColorExitClear());

        pressOptions(opMode, gamepad);
        assertTrue(opMode.isLastSaveSucceeded());

        CalibrationStore.ActiveCalibration reloaded = CalibrationStore.loadFromFile(calFile);
        assertTrue(reloaded.pinpointForwardReversed);
        assertFalse(reloaded.pinpointStrafeReversed);
        assertEquals(TRUE_FWD_OFFSET_MM, reloaded.pinpointForwardOffsetMm, MM_TOLERANCE);
        assertEquals(TRUE_STRAFE_OFFSET_MM, reloaded.pinpointStrafeOffsetMm, MM_TOLERANCE);
        assertEquals(TRUE_TURRET_CENTER_PWM, reloaded.turretCenterPwm, EPS);
        assertEquals(TRUE_TURRET_GEAR_RATIO, reloaded.turretGearRatio, EPS);
        assertEquals(TRUE_TURRET_OFFSET_X_M, reloaded.turretPivotOffsetXM, EPS);
        assertEquals(LUT_MODIFIED_DIST_KNOT_0, reloaded.lutDistancesM[ZERO_INT], TIGHT_EPS);
        assertEquals(TRUE_CAM_X_M, reloaded.camOffsetXM, EPS);
        assertEquals(TRUE_CAM_Y_M, reloaded.camOffsetYM, EPS);
        assertEquals(TRUE_CAM_Z_M, reloaded.camHeightM, EPS);
        assertEquals(TRUE_DATUM_OFFSET_DEG, reloaded.limelightDatumOffsetDeg, EPS);
        assertEquals(TRUE_SERVO_SPAN_DEG, reloaded.limelightServoSpanDeg, EPS);
        assertEquals(CAL_H_MIN, reloaded.pollenHMin, TIGHT_EPS);
        assertEquals(CAL_H_MAX, reloaded.pollenHMax, TIGHT_EPS);
        assertEquals(CAL_S_MIN, reloaded.pollenSMin, TIGHT_EPS);
        assertEquals(CAL_V_MIN, reloaded.pollenVMin, TIGHT_EPS);

        opMode.stopMatch();
    }

    @Test
    public void verifyMode4RestoresAprilTagPipelineAfterHsvCalibrationWithoutOverridingManualPitch()
            throws IOException {
        File calFile = tempFolder.newFile(CAL_FILE_NAME);
        MockAprilTagProbe aprilTagProbe = new MockAprilTagProbe();
        Gamepad gamepad = new Gamepad();

        CalibrationOpMode opMode = new CalibrationOpMode();
        opMode.telemetry = null;
        opMode.setCalibrationFileForTest(calFile);
        opMode.setGamepadForTest(gamepad);
        opMode.setAprilTagProbeForTest(aprilTagProbe);
        opMode.initialize();
        opMode.startMatch();

        for (int i = ZERO_INT; i < THREE_INT; i++) {
            pressRightBumper(opMode, gamepad);
        }
        assertEquals(CalibrationOpMode.CalibrationMode.MODE_4_VISION_COLOR, opMode.getActiveMode());

        CalibrationStore.ActiveCalibration cal = opMode.getWorkingCalibration();
        VisionSubsystem vision = opMode.getVision();

        pressSquare(opMode, gamepad);
        assertTrue(opMode.isHsvCalibrationActive());
        assertEquals(VisionSubsystem.VisionMode.POLLEN_FLOOR, vision.getMode());
        assertEquals(SensorConfig.LIMELIGHT_PIPELINE_POLLEN, vision.getDesiredPipelineIndex());

        double beforeJogPwm = opMode.getLiveLimelightPwm();
        pressDpadUp(opMode, gamepad);
        double expectedJoggedPwm = MathUtil.clamp(
                beforeJogPwm + CalibrationOpMode.COARSE_SERVO_STEP,
                cal.limelightServoMinLimit,
                cal.limelightServoMaxLimit);

        assertFalse(opMode.isHsvCalibrationActive());
        assertEquals(VisionSubsystem.VisionMode.APRILTAG_HORIZON, vision.getMode());
        assertEquals(SensorConfig.LIMELIGHT_PIPELINE_APRILTAG, vision.getDesiredPipelineIndex());
        assertTrue(vision.isPipelineDirty());
        assertEquals(expectedJoggedPwm, opMode.getLiveLimelightPwm(), TIGHT_EPS);
        assertEquals(expectedJoggedPwm, opMode.getLimelightServo().getLastWrittenPosition(), TIGHT_EPS);

        pressSquare(opMode, gamepad);
        assertTrue(opMode.isHsvCalibrationActive());
        assertEquals(VisionSubsystem.VisionMode.POLLEN_FLOOR, vision.getMode());

        for (int i = ZERO_INT; i < BLANKOUT_CLEAR_TICKS; i++) {
            opMode.tickLoop(DT_TWENTY_MS);
        }
        assertFalse(vision.isBlankoutActive());

        double[] pyOutput = new double[] {
                CAL_PY_VALID,
                CAL_PY_DX,
                CAL_PY_DY,
                CAL_PY_COUNT,
                CAL_H_MIN,
                CAL_H_MAX,
                CAL_S_MIN,
                CAL_V_MIN
        };
        assertTrue(vision.processPollenPythonOutput(pyOutput));
        opMode.tickLoop(DT_TWENTY_MS);

        assertFalse(opMode.isHsvCalibrationActive());
        assertEquals(VisionSubsystem.VisionMode.APRILTAG_HORIZON, vision.getMode());
        assertEquals(SensorConfig.LIMELIGHT_PIPELINE_APRILTAG, vision.getDesiredPipelineIndex());
        assertTrue(vision.isPipelineDirty());

        opMode.setReferenceAprilTagPoseMeters(REF_TAG_X_M, REF_TAG_Y_M, REF_TAG_Z_M);
        aprilTagProbe.setObservation(
                REF_TAG_X_M - TRUE_CAM_X_M,
                REF_TAG_Y_M - TRUE_CAM_Y_M,
                REF_TAG_Z_M - TRUE_CAM_Z_M,
                Math.toRadians(TRUE_DATUM_OFFSET_DEG));
        pressCross(opMode, gamepad);
        assertEquals(ONE_INT, opMode.getCameraExtrinsicSolver().getObservationCount());

        pressSquare(opMode, gamepad);
        assertTrue(opMode.isHsvCalibrationActive());
        assertEquals(VisionSubsystem.VisionMode.POLLEN_FLOOR, vision.getMode());

        pressRightBumper(opMode, gamepad);
        assertEquals(CalibrationOpMode.CalibrationMode.MODE_1_PINPOINT, opMode.getActiveMode());
        for (int i = ZERO_INT; i < THREE_INT; i++) {
            pressRightBumper(opMode, gamepad);
        }
        assertEquals(CalibrationOpMode.CalibrationMode.MODE_4_VISION_COLOR, opMode.getActiveMode());
        assertFalse(opMode.isHsvCalibrationActive());
        assertEquals(VisionSubsystem.VisionMode.APRILTAG_HORIZON, vision.getMode());
        assertEquals(SensorConfig.LIMELIGHT_PIPELINE_APRILTAG, vision.getDesiredPipelineIndex());
        assertEquals(
                SensorConfig.LIMELIGHT_SERVO_DATUM_NORMALIZED,
                opMode.getLimelightServo().getLastWrittenPosition(),
                TIGHT_EPS);

        opMode.stopMatch();
    }

    private static void executeMode1CalibrationSequence(
            CalibrationOpMode opMode,
            Gamepad gamepad,
            MockPinpointProbe pinpointProbe) {
        pressCross(opMode, gamepad);
        assertTrue(opMode.isImuAuditRunning());
        for (int i = ZERO_INT; i < IMU_AUDIT_TICKS; i++) {
            opMode.tickLoop(DT_TWENTY_MS);
        }
        assertFalse(opMode.isImuAuditRunning());
        assertTrue(opMode.isImuAuditPassed());

        pressSquare(opMode, gamepad);
        assertEquals(CalibrationOpMode.PinpointStep.POLARITY_PUSH_FORWARD, opMode.getPinpointStep());
        int pushTicks = (int) Math.round(
                PUSH_DISTANCE_M * ChassisConfig.MM_PER_METER * SensorConfig.PINPOINT_FOUR_BAR_TICKS_PER_MM);
        pinpointProbe.setState(-pushTicks, ZERO_INT, ZERO_DOUBLE);
        pressCross(opMode, gamepad);
        assertEquals(CalibrationOpMode.PinpointStep.POLARITY_PUSH_LEFT, opMode.getPinpointStep());

        pinpointProbe.setState(-pushTicks, pushTicks, ZERO_DOUBLE);
        pressCross(opMode, gamepad);
        assertEquals(CalibrationOpMode.PinpointStep.OLS_SPIN_REGRESSION, opMode.getPinpointStep());

        pressCross(opMode, gamepad);
        assertTrue(opMode.isSpinRecording());
        int baseEncX = pinpointProbe.getEncoderX();
        int baseEncY = pinpointProbe.getEncoderY();
        double totalAngleRad = SPIN_REVOLUTIONS * TWO_DOUBLE * Math.PI;
        for (int step = ONE_INT; step <= SPIN_SAMPLE_STEPS; step++) {
            double deltaTheta = totalAngleRad * ((double) step / (double) SPIN_SAMPLE_STEPS);
            double rawFwdMm = TRUE_FWD_OFFSET_MM * deltaTheta;
            double rawStrMm = TRUE_STRAFE_OFFSET_MM * deltaTheta;
            int encX = baseEncX + (int) Math.round(rawFwdMm * SensorConfig.PINPOINT_FOUR_BAR_TICKS_PER_MM);
            int encY = baseEncY + (int) Math.round(rawStrMm * SensorConfig.PINPOINT_FOUR_BAR_TICKS_PER_MM);
            pinpointProbe.setState(encX, encY, deltaTheta);
            opMode.tickLoop(DT_TWENTY_MS);
        }
        pressCircle(opMode, gamepad);
    }

    private static void executeMode2TurretSolveSequence(
            CalibrationOpMode opMode,
            Gamepad gamepad,
            MockPinpointProbe pinpointProbe) {
        opMode.setTurretMarkerDistanceM(TURRET_MARKER_DIST_M);
        double spanRad = Math.toRadians(TurretConfig.TURRET_SERVO_MAX_DEG / TRUE_TURRET_GEAR_RATIO);
        for (int i = ZERO_INT; i < TURRET_TEST_HEADINGS_RAD.length; i++) {
            double theta = TURRET_TEST_HEADINGS_RAD[i];
            double localX = TURRET_MARKER_DIST_M * Math.cos(theta) - TRUE_TURRET_OFFSET_X_M;
            double localY = -TURRET_MARKER_DIST_M * Math.sin(theta);
            double psiRel = Math.atan2(localY, localX);
            double pwm = TRUE_TURRET_CENTER_PWM + (psiRel / spanRad);
            pinpointProbe.setState(ZERO_INT, ZERO_INT, theta);
            opMode.setLiveTurretPwm(pwm);
            pressCross(opMode, gamepad);
        }
        pressCircle(opMode, gamepad);
    }

    private static void pressRightBumper(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.right_bumper = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.right_bumper = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressCross(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.cross = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.cross = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressCircle(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.circle = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.circle = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressSquare(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.square = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.square = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressTriangle(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.triangle = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.triangle = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressOptions(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.options = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.options = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressDpadUp(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.dpad_up = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.dpad_up = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressDpadDown(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.dpad_down = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.dpad_down = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressDpadLeft(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.dpad_left = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.dpad_left = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }

    private static void pressDpadRight(CalibrationOpMode opMode, Gamepad gamepad) {
        gamepad.dpad_right = true;
        opMode.tickLoop(DT_TWENTY_MS);
        gamepad.dpad_right = false;
        opMode.tickLoop(DT_TWENTY_MS);
    }
}
