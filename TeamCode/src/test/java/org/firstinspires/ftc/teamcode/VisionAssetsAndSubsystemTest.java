package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.google.gson.Gson;
import com.google.gson.JsonObject;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.junit.Test;
import org.kodikas.ftclib.hardware.CachedServo;

public final class VisionAssetsAndSubsystemTest {

    private static final String ASSETS_DIR_LOCAL = "src/main/assets";
    private static final String ASSETS_DIR_ROOT = "TeamCode/src/main/assets";
    private static final String VPR_FILE_NAME = "Pollen_Detection.vpr";
    private static final String SNAPSCRIPT_FILE_NAME = "pollen_snapscript.py";

    private static final String EXPECTED_BOT_TYPE = "mecanum";
    private static final String EXPECTED_PIPELINE_TYPE = "pipe_color";
    private static final String EXPECTED_PIPELINE_DESC = "Pollen_Detection";
    private static final String EXPECTED_FIDUCIAL_BACKEND = "umich";
    private static final String EXPECTED_FIDUCIAL_TYPE = "aprilClassic36h11";
    private static final String EXPECTED_FIDUCIAL_VIS_MODE = "3dtargposebotspace";
    private static final String EXPECTED_RUNTIME_CPU = "cpu";
    private static final String EXPECTED_BARCODE_TYPE = "qrzx";

    private static final int EXPECTED_VPR_KEY_COUNT = 103;
    private static final int EXPECTED_HUE_MIN = 14;
    private static final int EXPECTED_HUE_MAX = 27;
    private static final int EXPECTED_SAT_MIN = 185;
    private static final int EXPECTED_SAT_MAX = 255;
    private static final int EXPECTED_VAL_MIN = 72;
    private static final int EXPECTED_VAL_MAX = 255;
    private static final int EXPECTED_EROSION_STEPS = 1;
    private static final int EXPECTED_DILATION_STEPS = 2;
    private static final int EXPECTED_ZERO_INT = 0;
    private static final int EXPECTED_ONE_INT = 1;
    private static final int EXPECTED_TWO_INT = 2;
    private static final int EXPECTED_LED_POWER = 100;
    private static final int EXPECTED_MULTIGROUP_MAX = 7;

    private static final double EXPECTED_AREA_MIN = 0.4;
    private static final double EXPECTED_AREA_MAX = 85.0;
    private static final double EXPECTED_AREA_SIMILARITY = 25.0;
    private static final double EXPECTED_ASPECT_MIN = 0.6;
    private static final double EXPECTED_ASPECT_MAX = 1.55;
    private static final double EXPECTED_CONVEXITY_MIN = 30.0;
    private static final double EXPECTED_CONVEXITY_MAX = 95.0;
    private static final double EXPECTED_CORNER_APPROX = 2.0;
    private static final double EXPECTED_EXPOSURE = 3000.0;
    private static final double EXPECTED_RED_BALANCE = 1300.0;
    private static final double EXPECTED_BLUE_BALANCE = 2150.0;
    private static final double EXPECTED_LCGAIN = 34.0;
    private static final double EXPECTED_QUALITY_FOCUS = 0.3;
    private static final double EXPECTED_FIDUCIAL_SIZE = 101.6;
    private static final double EXPECTED_FIDUCIAL_QUALITY = 2.0;
    private static final double EXPECTED_TV_CONF = 0.5;
    private static final double EXPECTED_MARGIN_TV = 0.2;
    private static final double EXPECTED_OUTLIER_MIQR = 1.5;
    private static final double EXPECTED_DETECTOR_CONF = 0.8;
    private static final double EXPECTED_CLASSIFIER_CONF = 0.1;
    private static final double EXPECTED_DEPTH_MAX = 100.0;
    private static final double EXPECTED_ZERO_DOUBLE = 0.0;
    private static final double EXPECTED_ONE_DOUBLE = 1.0;
    private static final double EXPECTED_NEG_ONE_DOUBLE = -1.0;

    private static final char HASH_CHAR = '#';
    private static final double EPSILON = 1.0e-6;
    private static final double HALF = 0.5;
    private static final double TEST_IMAGE_WIDTH = 320.0;
    private static final double TEST_IMAGE_HEIGHT = 240.0;
    private static final double TEST_LEFT_PIXEL_X = 80.0;
    private static final double TEST_DT_FIFTY_MS = 0.050;
    private static final double TEST_DT_TWENTY_FIVE_MS = 0.025;
    private static final double TEST_DT_TEN_MS = 0.010;
    private static final int TELEOP_TICK_COUNT = 20;

    private static final double TEST_OMEGA_EXCESS_DEG_PER_SEC = 125.0;
    private static final double TEST_OMEGA_SAFE_DEG_PER_SEC = 90.0;
    private static final double TEST_VALID_LATENCY_MS = 35.0;
    private static final double TEST_EXCESS_LATENCY_MS = 140.0;
    private static final long TEST_VALID_STALENESS_MS = 20L;
    private static final long TEST_EXCESS_STALENESS_MS = 200L;
    private static final double TEST_VALID_TAG_AREA = 0.45;
    private static final double TEST_TINY_TAG_AREA = 0.05;
    private static final double TEST_TAG_DIST_M = 1.35;

    private static final double TEST_BLUE_RAW_X = 1.07125;
    private static final double TEST_BLUE_RAW_Y = 0.97125;
    private static final double EXPECTED_BLUE_FIELD_X = 2.90;
    private static final double EXPECTED_BLUE_FIELD_Y = 2.80;
    private static final double TEST_RED_RAW_X = -1.10;
    private static final double TEST_RED_RAW_Y = -0.80;
    private static final double TEST_OOB_RAW_COORD = 2.00;
    private static final double TEST_HEADING_ZERO = 0.0;

    private static final double TEST_POLLEN_VALID = 1.0;
    private static final double TEST_POLLEN_INVALID = 0.0;
    private static final double TEST_POLLEN_DX_FWD = 0.72;
    private static final double TEST_POLLEN_DY_LEFT = -0.05;
    private static final double TEST_POLLEN_COUNT_DOUBLE = 2.0;
    private static final int EXPECTED_POLLEN_COUNT = 2;
    private static final double TEST_CAL_H_MIN = 16.5;
    private static final double TEST_CAL_H_MAX = 29.5;
    private static final double TEST_CAL_S_MIN = 168.0;
    private static final double TEST_CAL_V_MIN = 84.0;
    private static final double TEST_SHIFTED_CAL_H_MIN = 19.0;
    private static final double TEST_SHIFTED_CAL_H_MAX = 31.0;
    private static final double TEST_SHIFTED_CAL_S_MIN = 142.0;
    private static final double TEST_SHIFTED_CAL_V_MIN = 96.0;
    private static final double CAL_TRIGGER_ACTIVE = 1.0;
    private static final double CAL_TRIGGER_INACTIVE = 0.0;
    private static final int FIRST_INDEX = 0;
    private static final int UNCHANGED_TICK_COUNT = 10;
    private static final int RATE_LIMIT_SUB_TICKS = 4;
    private static final long EXPECTED_QUEUE_COUNT_ONE = 1L;
    private static final long EXPECTED_QUEUE_COUNT_TWO = 2L;
    private static final long EXPECTED_QUEUE_COUNT_THREE = 3L;
    private static final double TEST_DT_FIFTEEN_MS = 0.015;
    private static final double TEST_HEADING_STEP_LARGE_DEG = 5.0;
    private static final double TEST_HEADING_STEP_TINY_DEG = 5.2;
    private static final double TEST_HEADING_STEP_OUTSIDE_DEADBAND_DEG = 6.7;

    @Test
    public void vprAssetParsesViaGsonAndMatchesAllVisRequirements() throws IOException {
        File vprFile = resolveAssetFile(VPR_FILE_NAME);
        assertTrue(vprFile.exists());
        String rawJson = new String(Files.readAllBytes(vprFile.toPath()), StandardCharsets.UTF_8);
        JsonObject json = new Gson().fromJson(rawJson, JsonObject.class);
        assertNotNull(json);

        assertEquals(EXPECTED_VPR_KEY_COUNT, json.size());

        assertEquals(EXPECTED_BOT_TYPE, json.get("bottype").getAsString());
        assertEquals(ChassisConfig.CHASSIS_WIDTH_M, json.get("botwidth").getAsDouble(), EPSILON);
        assertEquals(
                ChassisConfig.LENGTH_C1_RETRACTED_M,
                json.get("botlength").getAsDouble(),
                EPSILON);
        assertEquals(SNAPSCRIPT_FILE_NAME, json.get("python_snapscript_name").getAsString());

        assertEquals(EXPECTED_PIPELINE_TYPE, json.get("pipeline_type").getAsString());
        assertEquals(EXPECTED_PIPELINE_DESC, json.get("desc").getAsString());
        assertEquals(EXPECTED_EXPOSURE, json.get("exposure").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_RED_BALANCE, json.get("red_balance").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_BLUE_BALANCE, json.get("blue_balance").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_LCGAIN, json.get("lcgain").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_INT, json.get("black_level").getAsInt());
        assertEquals(EXPECTED_ONE_INT, json.get("pipeline_led_enabled").getAsInt());
        assertEquals(EXPECTED_LED_POWER, json.get("pipeline_led_power").getAsInt());
        assertEquals(EXPECTED_ZERO_INT, json.get("pipeline_res").getAsInt());
        assertEquals(EXPECTED_ONE_INT, json.get("flicker").getAsInt());
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("focus").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("crop_focus").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_QUALITY_FOCUS, json.get("quality_focus").getAsDouble(), EPSILON);

        assertEquals(EXPECTED_HUE_MIN, json.get("hue_min").getAsInt());
        assertEquals(EXPECTED_HUE_MAX, json.get("hue_max").getAsInt());
        assertEquals(EXPECTED_ZERO_INT, json.get("invert_hue").getAsInt());
        assertEquals(EXPECTED_SAT_MIN, json.get("sat_min").getAsInt());
        assertEquals(EXPECTED_SAT_MAX, json.get("sat_max").getAsInt());
        assertEquals(EXPECTED_VAL_MIN, json.get("val_min").getAsInt());
        assertEquals(EXPECTED_VAL_MAX, json.get("val_max").getAsInt());

        assertEquals(EXPECTED_EROSION_STEPS, json.get("erosion_steps").getAsInt());
        assertEquals(EXPECTED_DILATION_STEPS, json.get("dilation_steps").getAsInt());
        assertEquals(EXPECTED_ZERO_INT, json.get("reverse_morpho").getAsInt());
        assertEquals(EXPECTED_AREA_MIN, json.get("area_min").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_AREA_MAX, json.get("area_max").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_AREA_SIMILARITY, json.get("area_similarity").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ASPECT_MIN, json.get("aspect_min").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ASPECT_MAX, json.get("aspect_max").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_CONVEXITY_MIN, json.get("convexity_min").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_CONVEXITY_MAX, json.get("convexity_max").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ONE_INT, json.get("force_convex").getAsInt());
        assertEquals(EXPECTED_CORNER_APPROX, json.get("corner_approx").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ONE_INT, json.get("multigroup_min").getAsInt());
        assertEquals(EXPECTED_MULTIGROUP_MAX, json.get("multigroup_max").getAsInt());
        assertEquals(EXPECTED_ZERO_INT, json.get("multigroup_rejector").getAsInt());

        assertEquals(EXPECTED_NEG_ONE_DOUBLE, json.get("crop_x_min").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ONE_DOUBLE, json.get("crop_x_max").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_NEG_ONE_DOUBLE, json.get("crop_y_min").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ONE_DOUBLE, json.get("crop_y_max").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_TV_CONF, json.get("tv_conf").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_MARGIN_TV, json.get("margin_tv").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_OUTLIER_MIQR, json.get("x_outlier_miqr").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_OUTLIER_MIQR, json.get("y_outlier_miqr").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("depthrange_min").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_DEPTH_MAX, json.get("depthrange_max").getAsDouble(), EPSILON);

        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("rsf").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("rspitch").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("rsroll").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("rss").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("rsu").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("rsyaw").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("tsf").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("tss").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_ZERO_DOUBLE, json.get("tsu").getAsDouble(), EPSILON);

        assertEquals(EXPECTED_FIDUCIAL_BACKEND, json.get("fiducial_backend").getAsString());
        assertEquals(EXPECTED_FIDUCIAL_TYPE, json.get("fiducial_type").getAsString());
        assertEquals(EXPECTED_FIDUCIAL_SIZE, json.get("fiducial_size").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_TWO_INT, json.get("fiducial_resdiv").getAsInt());
        assertEquals(
                EXPECTED_FIDUCIAL_QUALITY,
                json.get("fiducial_qualitythreshold").getAsDouble(),
                EPSILON);
        assertEquals(EXPECTED_ONE_INT, json.get("fiducial_skip3d").getAsInt());
        assertEquals(EXPECTED_FIDUCIAL_VIS_MODE, json.get("fiducial_vis_mode").getAsString());
        assertEquals(EXPECTED_DETECTOR_CONF, json.get("detector_conf").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_RUNTIME_CPU, json.get("detector_runtime").getAsString());
        assertEquals(EXPECTED_CLASSIFIER_CONF, json.get("classifier_conf").getAsDouble(), EPSILON);
        assertEquals(EXPECTED_RUNTIME_CPU, json.get("classifier_runtime").getAsString());
        assertEquals(
                EXPECTED_CLASSIFIER_CONF,
                json.get("zsclassifier_conf").getAsDouble(),
                EPSILON);
        assertEquals(EXPECTED_BARCODE_TYPE, json.get("barcode_type").getAsString());
    }

    @Test
    public void snapScriptAssetHasZeroHashCommentsAndImplementsExactEquations() throws IOException {
        File pyFile = resolveAssetFile(SNAPSCRIPT_FILE_NAME);
        assertTrue(pyFile.exists());
        String source = new String(Files.readAllBytes(pyFile.toPath()), StandardCharsets.UTF_8);

        assertTrue(source.indexOf(HASH_CHAR) < EXPECTED_ZERO_INT);
        assertTrue(source.contains("def runPipeline(image, llrobot):"));
        assertTrue(source.contains("def project_pixel_to_robot_nwu("));
        assertTrue(source.contains("def calibrate_hsv_from_patch("));
        assertTrue(source.contains("HFOV_DEG = 54.5"));
        assertTrue(source.contains("VFOV_DEG = 42.0"));
        assertTrue(source.contains("POLLEN_SPHERE_RADIUS_M = 0.0355"));
        assertTrue(source.contains("RETICLE_SIZE_PX = 40"));
        assertTrue(source.contains("IQR_MULTIPLIER = 2.5"));
        assertTrue(source.contains("rx = cos_p - sin_p * tan_ay"));
        assertTrue(source.contains("ry = -tan_ax"));
        assertTrue(source.contains("rz = sin_p + cos_p * tan_ay"));
        assertTrue(source.contains("np.percentile"));
        assertTrue(source.contains("llpython = ["));

        double halfW = TEST_IMAGE_WIDTH * HALF;
        double halfH = TEST_IMAGE_HEIGHT * HALF;
        double pitchRad = Math.toRadians(SensorConfig.LIMELIGHT_POLLEN_DOWN_PITCH_DEG);
        double rxCenter = Math.cos(pitchRad);
        double rzCenter = Math.sin(pitchRad);
        double rayScale = -(SensorConfig.LIMELIGHT_CAMERA_HEIGHT_M - SensorConfig.POLLEN_SPHERE_RADIUS_M)
                / rzCenter;
        double expectedCenterFwd = SensorConfig.LIMELIGHT_CAMERA_FORWARD_OFFSET_M + rayScale * rxCenter;
        double closedFormFwd = SensorConfig.LIMELIGHT_CAMERA_FORWARD_OFFSET_M
                + (SensorConfig.LIMELIGHT_CAMERA_HEIGHT_M - SensorConfig.POLLEN_SPHERE_RADIUS_M)
                        / Math.tan(Math.abs(pitchRad));
        assertEquals(closedFormFwd, expectedCenterFwd, EPSILON);

        double tanAxLeft = ((TEST_LEFT_PIXEL_X - halfW) / halfW)
                * Math.tan(Math.toRadians(SensorConfig.LIMELIGHT_HFOV_DEG * HALF));
        double ryLeft = -tanAxLeft;
        double dyLeftM = SensorConfig.LIMELIGHT_CAMERA_SIDEWAYS_OFFSET_M + rayScale * ryLeft;
        assertTrue(dyLeftM > EXPECTED_ZERO_DOUBLE);
        assertEquals(halfH, TEST_IMAGE_HEIGHT * HALF, EPSILON);
    }

    @Test
    public void teleOpDefaultsToAprilTagHorizonAndHoldsUpPitchRelativeToDatum() {
        RobotState robotState = new RobotState();
        CalibrationStore.ActiveCalibration calibration = new CalibrationStore.ActiveCalibration();
        CachedServo servo = createTestServo(calibration);
        VisionSubsystem vision = new VisionSubsystem((Limelight3A) null, servo, robotState, calibration);

        assertEquals(VisionSubsystem.SUBSYSTEM_ID, vision.subsystemId());
        assertEquals(VisionSubsystem.VisionMode.APRILTAG_HORIZON, vision.getMode());
        assertEquals(
                SensorConfig.LIMELIGHT_APRILTAG_UP_PITCH_DEG,
                vision.getCurrentPitchDeg(),
                EPSILON);

        double expectedServoUp = SensorConfig.pitchDegToServoNormalized(
                SensorConfig.LIMELIGHT_APRILTAG_UP_PITCH_DEG,
                SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG,
                SensorConfig.LIMELIGHT_SERVO_SPAN_DEG,
                SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN);
        assertEquals(expectedServoUp, vision.getTargetServoNormalized(), EPSILON);
        assertTrue(vision.getTargetServoNormalized() > SensorConfig.LIMELIGHT_SERVO_DATUM_NORMALIZED);
        assertFalse(vision.isBlankoutActive());

        for (int i = FIRST_INDEX; i < TELEOP_TICK_COUNT; i++) {
            vision.readSensors(TEST_DT_TEN_MS);
            vision.compute(TEST_DT_TEN_MS);
            vision.writeActuators();
        }

        assertEquals(VisionSubsystem.VisionMode.APRILTAG_HORIZON, vision.getMode());
        assertEquals(expectedServoUp, servo.getLastWrittenPosition(), EPSILON);
        assertFalse(robotState.visionPitchBlankoutActive);
    }

    @Test
    public void modeSwitchToPollenFloorEnforcesBlankoutAndSetsDownPitchFromDatum() {
        RobotState robotState = new RobotState();
        CalibrationStore.ActiveCalibration calibration = new CalibrationStore.ActiveCalibration();
        CachedServo servo = createTestServo(calibration);
        VisionSubsystem vision = new VisionSubsystem((Limelight3A) null, servo, robotState, calibration);

        vision.setMode(VisionSubsystem.VisionMode.POLLEN_FLOOR);
        assertEquals(VisionSubsystem.VisionMode.POLLEN_FLOOR, vision.getMode());
        assertEquals(
                SensorConfig.LIMELIGHT_POLLEN_DOWN_PITCH_DEG,
                vision.getCurrentPitchDeg(),
                EPSILON);

        double expectedServoDown = SensorConfig.pitchDegToServoNormalized(
                SensorConfig.LIMELIGHT_POLLEN_DOWN_PITCH_DEG,
                SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG,
                SensorConfig.LIMELIGHT_SERVO_SPAN_DEG,
                SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN);
        assertEquals(expectedServoDown, vision.getTargetServoNormalized(), EPSILON);
        assertTrue(vision.getTargetServoNormalized() < SensorConfig.LIMELIGHT_SERVO_DATUM_NORMALIZED);
        assertTrue(vision.isBlankoutActive());
        assertTrue(robotState.visionPitchBlankoutActive);

        double[] samplePollen = new double[] {
                TEST_POLLEN_VALID,
                TEST_POLLEN_DX_FWD,
                TEST_POLLEN_DY_LEFT,
                TEST_POLLEN_COUNT_DOUBLE,
                SensorConfig.POLLEN_DEFAULT_H_MIN,
                SensorConfig.POLLEN_DEFAULT_H_MAX,
                SensorConfig.POLLEN_DEFAULT_S_MIN,
                SensorConfig.POLLEN_DEFAULT_V_MIN
        };

        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.writeActuators();
        assertTrue(vision.isBlankoutActive());
        assertFalse(vision.processPollenPythonOutput(samplePollen));
        assertFalse(robotState.pollenVisible);

        vision.readSensors(TEST_DT_FIFTY_MS);
        assertTrue(vision.isBlankoutActive());
        assertFalse(vision.processPollenPythonOutput(samplePollen));

        vision.readSensors(TEST_DT_TWENTY_FIVE_MS);
        assertFalse(vision.isBlankoutActive());
        assertFalse(robotState.visionPitchBlankoutActive);
        assertTrue(vision.processPollenPythonOutput(samplePollen));
        assertTrue(robotState.pollenVisible);
        assertEquals(TEST_POLLEN_DX_FWD, robotState.pollenRobotRelative.x, EPSILON);
        assertEquals(TEST_POLLEN_DY_LEFT, robotState.pollenRobotRelative.y, EPSILON);
        assertEquals(EXPECTED_POLLEN_COUNT, robotState.pollenVisibleCount);

        vision.setMode(VisionSubsystem.VisionMode.APRILTAG_HORIZON);
        assertTrue(vision.isBlankoutActive());
        assertFalse(vision.processAprilTagObservation(
                TEST_RED_RAW_X,
                TEST_RED_RAW_Y,
                TEST_HEADING_ZERO,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.RED_AUDIENCE_TAG_IDS[FIRST_INDEX]));

        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.readSensors(TEST_DT_TWENTY_FIVE_MS);
        assertFalse(vision.isBlankoutActive());
        assertTrue(vision.processAprilTagObservation(
                TEST_RED_RAW_X,
                TEST_RED_RAW_Y,
                TEST_HEADING_ZERO,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.RED_AUDIENCE_TAG_IDS[FIRST_INDEX]));
    }

    @Test
    public void omegaGateRejectsHighSpinAndAcceptsSafeAngularRate() {
        RobotState robotState = new RobotState();
        CalibrationStore.ActiveCalibration calibration = new CalibrationStore.ActiveCalibration();
        VisionSubsystem vision = new VisionSubsystem(
                (Limelight3A) null,
                createTestServo(calibration),
                robotState,
                calibration);

        robotState.robotVelocity.omega = Math.toRadians(TEST_OMEGA_EXCESS_DEG_PER_SEC);
        boolean acceptedHighSpin = vision.processAprilTagObservation(
                TEST_RED_RAW_X,
                TEST_RED_RAW_Y,
                TEST_HEADING_ZERO,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.RED_AUDIENCE_TAG_IDS[FIRST_INDEX]);
        assertFalse(acceptedHighSpin);
        assertFalse(robotState.visionMeasurementFresh);
        assertFalse(robotState.visionMeasurement.valid);

        robotState.robotVelocity.omega = Math.toRadians(TEST_OMEGA_SAFE_DEG_PER_SEC);
        boolean acceptedSafeSpin = vision.processAprilTagObservation(
                TEST_RED_RAW_X,
                TEST_RED_RAW_Y,
                TEST_HEADING_ZERO,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.RED_AUDIENCE_TAG_IDS[FIRST_INDEX]);
        assertTrue(acceptedSafeSpin);
        assertTrue(robotState.visionMeasurementFresh);
        assertTrue(robotState.visionMeasurement.valid);
    }

    @Test
    public void centerOriginBlueQuadrantBotposeTransformAndQualityGatesWork() {
        RobotState robotState = new RobotState();
        robotState.resetForMatch(FieldConfig.Alliance.BLUE, FieldConfig.POSE_NUMBER_GARDEN, false);
        CalibrationStore.ActiveCalibration calibration = new CalibrationStore.ActiveCalibration();
        VisionSubsystem vision = new VisionSubsystem(
                (Limelight3A) null,
                createTestServo(calibration),
                robotState,
                calibration);

        boolean acceptedBlue = vision.processAprilTagObservation(
                TEST_BLUE_RAW_X,
                TEST_BLUE_RAW_Y,
                Math.PI,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.BLUE_SCORING_TAG_IDS[FIRST_INDEX]);
        assertTrue(acceptedBlue);
        assertEquals(EXPECTED_BLUE_FIELD_X, robotState.visionMeasurement.measuredFieldPose.x, EPSILON);
        assertEquals(EXPECTED_BLUE_FIELD_Y, robotState.visionMeasurement.measuredFieldPose.y, EPSILON);

        assertFalse(vision.processAprilTagObservation(
                TEST_OOB_RAW_COORD,
                TEST_OOB_RAW_COORD,
                TEST_HEADING_ZERO,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.BLUE_SCORING_TAG_IDS[FIRST_INDEX]));
        assertFalse(vision.processAprilTagObservation(
                TEST_BLUE_RAW_X,
                TEST_BLUE_RAW_Y,
                TEST_HEADING_ZERO,
                TEST_EXCESS_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.BLUE_SCORING_TAG_IDS[FIRST_INDEX]));
        assertFalse(vision.processAprilTagObservation(
                TEST_BLUE_RAW_X,
                TEST_BLUE_RAW_Y,
                TEST_HEADING_ZERO,
                TEST_VALID_LATENCY_MS,
                TEST_EXCESS_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.BLUE_SCORING_TAG_IDS[FIRST_INDEX]));
        assertFalse(vision.processAprilTagObservation(
                TEST_BLUE_RAW_X,
                TEST_BLUE_RAW_Y,
                TEST_HEADING_ZERO,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_TINY_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.BLUE_SCORING_TAG_IDS[FIRST_INDEX]));
    }

    @Test
    public void fastAutoInitialHiveAprilTagCheckConfirmsRedAudienceAndBlueScoring() {
        CalibrationStore.ActiveCalibration calibration = new CalibrationStore.ActiveCalibration();

        RobotState redState = new RobotState();
        redState.resetForMatch(FieldConfig.Alliance.RED, FieldConfig.POSE_NUMBER_GARDEN, true);
        redState.activeHiveCell = FieldConfig.HiveCell.SCORING;
        redState.refreshActiveHiveTarget();
        VisionSubsystem redVision = new VisionSubsystem(
                (Limelight3A) null,
                createTestServo(calibration),
                redState,
                calibration);
        redVision.enableInitialHiveCheck(true);

        assertTrue(redVision.processAprilTagObservation(
                TEST_RED_RAW_X,
                TEST_RED_RAW_Y,
                TEST_HEADING_ZERO,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.RED_AUDIENCE_TAG_IDS[FIRST_INDEX]));
        assertTrue(redState.initialHiveTagConfirmed);
        assertEquals(FieldConfig.HiveCell.AUDIENCE, redState.detectedInitialHiveCell);
        assertEquals(FieldConfig.HiveCell.AUDIENCE, redState.activeHiveCell);
        assertEquals(FieldConfig.RED_HIVE_TARGET_X_M, redState.activeHiveTarget.x, EPSILON);
        assertEquals(FieldConfig.HIVE_AUDIENCE_CELL_Y_M, redState.activeHiveTarget.y, EPSILON);

        RobotState blueState = new RobotState();
        blueState.resetForMatch(FieldConfig.Alliance.BLUE, FieldConfig.POSE_NUMBER_GARDEN, true);
        blueState.activeHiveCell = FieldConfig.HiveCell.AUDIENCE;
        blueState.refreshActiveHiveTarget();
        VisionSubsystem blueVision = new VisionSubsystem(
                (Limelight3A) null,
                createTestServo(calibration),
                blueState,
                calibration);
        blueVision.enableInitialHiveCheck(true);

        assertTrue(blueVision.processAprilTagObservation(
                TEST_BLUE_RAW_X,
                TEST_BLUE_RAW_Y,
                Math.PI,
                TEST_VALID_LATENCY_MS,
                TEST_VALID_STALENESS_MS,
                TEST_VALID_TAG_AREA,
                TEST_TAG_DIST_M,
                FieldConfig.BLUE_SCORING_TAG_IDS[FIRST_INDEX]));
        assertTrue(blueState.initialHiveTagConfirmed);
        assertEquals(FieldConfig.HiveCell.SCORING, blueState.detectedInitialHiveCell);
        assertEquals(FieldConfig.HiveCell.SCORING, blueState.activeHiveCell);
        assertEquals(FieldConfig.BLUE_HIVE_TARGET_X_M, blueState.activeHiveTarget.x, EPSILON);
        assertEquals(FieldConfig.HIVE_SCORING_CELL_Y_M, blueState.activeHiveTarget.y, EPSILON);
    }

    @Test
    public void oneTapHsvCalibrationUpdatesRobotStateAndCalibrationFromPythonOutput() {
        RobotState robotState = new RobotState();
        CalibrationStore.ActiveCalibration calibration = new CalibrationStore.ActiveCalibration();
        VisionSubsystem vision = new VisionSubsystem(
                (Limelight3A) null,
                createTestServo(calibration),
                robotState,
                calibration);

        vision.requestHsvCalibration();
        assertEquals(VisionSubsystem.VisionMode.POLLEN_FLOOR, vision.getMode());
        assertTrue(vision.isBlankoutActive());
        assertEquals(
                CAL_TRIGGER_INACTIVE,
                vision.getPythonInput(SensorConfig.LLROBOT_IDX_CAL_TRIGGER),
                EPSILON);

        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.writeActuators();

        assertFalse(vision.isBlankoutActive());
        assertEquals(
                CAL_TRIGGER_ACTIVE,
                vision.getPythonInput(SensorConfig.LLROBOT_IDX_CAL_TRIGGER),
                EPSILON);

        double[] calibratedOutput = new double[] {
                TEST_POLLEN_VALID,
                TEST_POLLEN_DX_FWD,
                TEST_POLLEN_DY_LEFT,
                TEST_POLLEN_COUNT_DOUBLE,
                TEST_CAL_H_MIN,
                TEST_CAL_H_MAX,
                TEST_CAL_S_MIN,
                TEST_CAL_V_MIN
        };
        assertTrue(vision.processPollenPythonOutput(calibratedOutput));
        assertTrue(robotState.hsvCalibrationUpdated);
        assertEquals(TEST_CAL_H_MIN, robotState.calibratedHMin, EPSILON);
        assertEquals(TEST_CAL_H_MAX, robotState.calibratedHMax, EPSILON);
        assertEquals(TEST_CAL_S_MIN, robotState.calibratedSMin, EPSILON);
        assertEquals(TEST_CAL_V_MIN, robotState.calibratedVMin, EPSILON);
        assertEquals(TEST_CAL_H_MIN, calibration.pollenHMin, EPSILON);
        assertEquals(TEST_CAL_H_MAX, calibration.pollenHMax, EPSILON);
        assertEquals(TEST_CAL_S_MIN, calibration.pollenSMin, EPSILON);
        assertEquals(TEST_CAL_V_MIN, calibration.pollenVMin, EPSILON);
        assertEquals(
                CAL_TRIGGER_INACTIVE,
                vision.getPythonInput(SensorConfig.LLROBOT_IDX_CAL_TRIGGER),
                EPSILON);

        double expectedPackedSV = TEST_CAL_S_MIN * SensorConfig.LLROBOT_SV_PACK_SCALE + TEST_CAL_V_MIN;
        assertEquals(
                expectedPackedSV,
                vision.getPythonInput(SensorConfig.LLROBOT_IDX_SV_PACKED),
                EPSILON);
    }

    @Test
    public void oneTapHsvCalibrationSucceedsWhenZeroContoursMatchOldThresholds() {
        RobotState robotState = new RobotState();
        CalibrationStore.ActiveCalibration calibration = new CalibrationStore.ActiveCalibration();
        VisionSubsystem vision = new VisionSubsystem(
                (Limelight3A) null,
                createTestServo(calibration),
                robotState,
                calibration);

        vision.requestHsvCalibration();
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.writeActuators();
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.writeActuators();
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.writeActuators();

        assertFalse(vision.isBlankoutActive());
        assertEquals(
                CAL_TRIGGER_ACTIVE,
                vision.getPythonInput(SensorConfig.LLROBOT_IDX_CAL_TRIGGER),
                EPSILON);

        long queueCountBeforeCalCommit = vision.getPythonInputsQueueCount();

        double[] zeroContourCalibratedOutput = new double[] {
                TEST_POLLEN_INVALID,
                EXPECTED_ZERO_DOUBLE,
                EXPECTED_ZERO_DOUBLE,
                EXPECTED_ZERO_DOUBLE,
                TEST_SHIFTED_CAL_H_MIN,
                TEST_SHIFTED_CAL_H_MAX,
                TEST_SHIFTED_CAL_S_MIN,
                TEST_SHIFTED_CAL_V_MIN
        };

        assertFalse(vision.processPollenPythonOutput(zeroContourCalibratedOutput));
        assertFalse(robotState.pollenVisible);
        assertEquals(EXPECTED_ZERO_INT, robotState.pollenVisibleCount);
        assertTrue(robotState.hsvCalibrationUpdated);
        assertEquals(TEST_SHIFTED_CAL_H_MIN, robotState.calibratedHMin, EPSILON);
        assertEquals(TEST_SHIFTED_CAL_H_MAX, robotState.calibratedHMax, EPSILON);
        assertEquals(TEST_SHIFTED_CAL_S_MIN, robotState.calibratedSMin, EPSILON);
        assertEquals(TEST_SHIFTED_CAL_V_MIN, robotState.calibratedVMin, EPSILON);
        assertEquals(TEST_SHIFTED_CAL_H_MIN, calibration.pollenHMin, EPSILON);
        assertEquals(TEST_SHIFTED_CAL_H_MAX, calibration.pollenHMax, EPSILON);
        assertEquals(TEST_SHIFTED_CAL_S_MIN, calibration.pollenSMin, EPSILON);
        assertEquals(TEST_SHIFTED_CAL_V_MIN, calibration.pollenVMin, EPSILON);
        assertEquals(
                CAL_TRIGGER_INACTIVE,
                vision.getPythonInput(SensorConfig.LLROBOT_IDX_CAL_TRIGGER),
                EPSILON);

        vision.compute(TEST_DT_TEN_MS);
        vision.writeActuators();
        long queueCountAfterCalCommit = vision.getPythonInputsQueueCount();
        assertEquals(queueCountBeforeCalCommit + EXPECTED_QUEUE_COUNT_ONE, queueCountAfterCalCommit);

        for (int i = FIRST_INDEX; i < UNCHANGED_TICK_COUNT; i++) {
            vision.readSensors(TEST_DT_TEN_MS);
            vision.compute(TEST_DT_TEN_MS);
            vision.writeActuators();
        }
        assertEquals(queueCountAfterCalCommit, vision.getPythonInputsQueueCount());
    }

    @Test
    public void writeActuatorsQueuesDirtyOnlyPythonInputsAndRateLimitedOrientation() {
        RobotState robotState = new RobotState();
        CalibrationStore.ActiveCalibration calibration = new CalibrationStore.ActiveCalibration();
        VisionSubsystem vision = new VisionSubsystem(
                (Limelight3A) null,
                createTestServo(calibration),
                robotState,
                calibration);

        robotState.pose.heading = TEST_HEADING_ZERO;
        vision.readSensors(TEST_DT_TEN_MS);
        vision.compute(TEST_DT_TEN_MS);
        vision.writeActuators();
        assertEquals(EXPECTED_QUEUE_COUNT_ONE, vision.getOrientationQueueCount());

        robotState.pose.heading = Math.toRadians(TEST_HEADING_STEP_LARGE_DEG);
        for (int i = FIRST_INDEX; i < RATE_LIMIT_SUB_TICKS; i++) {
            vision.readSensors(TEST_DT_TEN_MS);
            vision.compute(TEST_DT_TEN_MS);
            vision.writeActuators();
        }
        assertEquals(EXPECTED_QUEUE_COUNT_ONE, vision.getOrientationQueueCount());

        vision.readSensors(TEST_DT_FIFTEEN_MS);
        vision.compute(TEST_DT_FIFTEEN_MS);
        vision.writeActuators();
        assertEquals(EXPECTED_QUEUE_COUNT_TWO, vision.getOrientationQueueCount());

        robotState.pose.heading = Math.toRadians(TEST_HEADING_STEP_TINY_DEG);
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.writeActuators();
        assertEquals(EXPECTED_QUEUE_COUNT_TWO, vision.getOrientationQueueCount());

        robotState.pose.heading = Math.toRadians(TEST_HEADING_STEP_OUTSIDE_DEADBAND_DEG);
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_FIFTY_MS);
        vision.writeActuators();
        assertEquals(EXPECTED_QUEUE_COUNT_THREE, vision.getOrientationQueueCount());

        vision.setMode(VisionSubsystem.VisionMode.POLLEN_FLOOR);
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.readSensors(TEST_DT_FIFTY_MS);
        vision.compute(TEST_DT_TEN_MS);
        vision.writeActuators();
        assertEquals(EXPECTED_QUEUE_COUNT_ONE, vision.getPythonInputsQueueCount());

        for (int i = FIRST_INDEX; i < UNCHANGED_TICK_COUNT; i++) {
            vision.readSensors(TEST_DT_TEN_MS);
            vision.compute(TEST_DT_TEN_MS);
            vision.writeActuators();
        }
        assertEquals(EXPECTED_QUEUE_COUNT_ONE, vision.getPythonInputsQueueCount());

        vision.requestHsvCalibration();
        vision.compute(TEST_DT_TEN_MS);
        vision.writeActuators();
        assertEquals(EXPECTED_QUEUE_COUNT_TWO, vision.getPythonInputsQueueCount());

        for (int i = FIRST_INDEX; i < UNCHANGED_TICK_COUNT; i++) {
            vision.readSensors(TEST_DT_TEN_MS);
            vision.compute(TEST_DT_TEN_MS);
            vision.writeActuators();
        }
        assertEquals(EXPECTED_QUEUE_COUNT_TWO, vision.getPythonInputsQueueCount());
    }

    private static CachedServo createTestServo(CalibrationStore.ActiveCalibration calibration) {
        return new CachedServo(
                null,
                calibration.limelightServoMinLimit,
                calibration.limelightServoMaxLimit,
                calibration.limelightServoReversed);
    }

    private static File resolveAssetFile(String fileName) {
        File localCandidate = new File(ASSETS_DIR_LOCAL, fileName);
        if (localCandidate.exists()) {
            return localCandidate;
        }
        File rootCandidate = new File(ASSETS_DIR_ROOT, fileName);
        assertNotNull(rootCandidate);
        return rootCandidate;
    }
}
