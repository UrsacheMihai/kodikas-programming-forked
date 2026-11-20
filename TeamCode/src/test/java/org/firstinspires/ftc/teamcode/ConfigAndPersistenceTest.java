package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.util.HashSet;
import java.util.Set;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.HardwarePorts;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.MatchSession;
import org.firstinspires.ftc.teamcode.config.PursuitConfig;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.kodikas.ftclib.ballistics.BallisticProfile;
import org.kodikas.ftclib.kodipursuit.PathConstraints;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

public class ConfigAndPersistenceTest {

    private static final double EPS = 1.0e-6;
    private static final double LOOSE_EPS = 1.0e-4;
    private static final int ZERO_INT = 0;
    private static final int EXPECTED_PORT_COUNT = 15;
    private static final double EXPECTED_CHASSIS_WIDTH_MM = 292.00;
    private static final double EXPECTED_C1_MM = 375.59;
    private static final double EXPECTED_C3_MM = 460.59;
    private static final double EXPECTED_C2_MM = 472.32;
    private static final double EXPECTED_LATERAL_MULT = 1.12;
    private static final double EXPECTED_HARD_STOP_DEG = 350.0 / (2.0 * 2.75);
    private static final double EXTREME_LOW_DIST_M = 0.25;
    private static final double EXTREME_HIGH_DIST_M = 6.00;
    private static final double EXPECTED_TRANSFER_CLEAR_M = 0.52;
    private static final double TEST_CUSTOM_CAM_X_M = 0.19;
    private static final double TEST_CUSTOM_UP_PITCH_DEG = 14.5;
    private static final double TEST_CUSTOM_DOWN_PITCH_DEG = -24.0;
    private static final double TEST_CUSTOM_H_MIN = 16.0;
    private static final double INVALID_LOW_CLAMP = 0.01;
    private static final double INVALID_HIGH_CLAMP = 0.99;
    private static final double INVALID_DISTANCE_M = 1.25;
    private static final double TEST_POSE_X_M = 1.42;
    private static final double TEST_POSE_Y_M = 2.18;
    private static final double TEST_POSE_HEADING_RAD = 0.75;
    private static final double OUTSIDE_FIELD_COORD_M = 5.0;
    private static final int INVALID_TAG_ID = 99;
    private static final int RED_AUDIENCE_TAG_SAMPLE = 34;
    private static final int RED_SCORING_TAG_SAMPLE = 31;
    private static final int BLUE_AUDIENCE_TAG_SAMPLE = 40;
    private static final int BLUE_SCORING_TAG_SAMPLE = 45;
    private static final double EXPECTED_FIELD_SIZE_M = 3.6575;
    private static final double EXPECTED_RED_HIVE_X_M = 1.5051;
    private static final double EXPECTED_BLUE_HIVE_X_M = 2.1524;
    private static final double EXPECTED_AUDIENCE_Y_M = 1.4366;
    private static final double EXPECTED_SCORING_Y_M = 2.2209;
    private static final double EXPECTED_APPROACH_MAX_VEL = 0.85;
    private static final double EXPECTED_DEFAULT_MAX_VEL = 1.85;

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    @Test
    public void verifyHardwarePortsAreCompleteAndUnique() {
        assertEquals(EXPECTED_PORT_COUNT, HardwarePorts.PORT_COUNT);
        assertEquals(EXPECTED_PORT_COUNT, HardwarePorts.ALL_PORTS.length);
        Set<String> unique = new HashSet<>();
        for (int i = ZERO_INT; i < HardwarePorts.ALL_PORTS.length; i++) {
            assertTrue(unique.add(HardwarePorts.ALL_PORTS[i]));
        }
        assertEquals(EXPECTED_PORT_COUNT, unique.size());
    }

    @Test
    public void verifyChassisAndTurretAndIntakeConstants() {
        assertEquals(EXPECTED_CHASSIS_WIDTH_MM, ChassisConfig.CHASSIS_WIDTH_MM, EPS);
        assertEquals(EXPECTED_C1_MM, ChassisConfig.LENGTH_C1_RETRACTED_MM, EPS);
        assertEquals(EXPECTED_C3_MM, ChassisConfig.LENGTH_C3_PLOW_DEPLOYED_TURRET_REST_MM, EPS);
        assertEquals(EXPECTED_C2_MM, ChassisConfig.LENGTH_C2_PLOW_DEPLOYED_TURRET_ROTATED_MM, EPS);
        assertEquals(
                EXPECTED_C2_MM - EXPECTED_C3_MM,
                ChassisConfig.HOOD_REAR_OVERHANG_MM,
                EPS);
        assertEquals(EXPECTED_LATERAL_MULT, ChassisConfig.LATERAL_MULTIPLIER, EPS);
        assertTrue(ChassisConfig.HAS_EXTERNAL_SERVO_POWER_INJECTOR);

        assertEquals(EXPECTED_HARD_STOP_DEG, TurretConfig.TURRET_PHYSICAL_HARD_STOP_DEG, LOOSE_EPS);
        BallisticProfile profile = TurretConfig.createDefaultBallisticProfile();
        double lowPitch = profile.evaluateHoodPitch(EXTREME_LOW_DIST_M);
        double highPitch = profile.evaluateHoodPitch(EXTREME_HIGH_DIST_M);
        assertTrue(lowPitch >= TurretConfig.HOOD_MIN_CLAMP - EPS);
        assertTrue(highPitch <= TurretConfig.HOOD_MAX_CLAMP + EPS);
        assertEquals(EXPECTED_TRANSFER_CLEAR_M, IntakeConfig.L_TRANSFER_CLEAR_M, EPS);
    }

    @Test
    public void verifySensorDatumPitchMathAndFieldGeometry() {
        double datumNorm = SensorConfig.pitchDegToServoNormalized(
                SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG,
                SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG,
                SensorConfig.LIMELIGHT_SERVO_SPAN_DEG,
                SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN);
        assertEquals(SensorConfig.LIMELIGHT_SERVO_DATUM_NORMALIZED, datumNorm, EPS);

        double upNorm = SensorConfig.pitchDegToServoNormalized(
                SensorConfig.LIMELIGHT_APRILTAG_UP_PITCH_DEG,
                SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG,
                SensorConfig.LIMELIGHT_SERVO_SPAN_DEG,
                SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN);
        double upDeg = SensorConfig.servoNormalizedToPitchDeg(
                upNorm,
                SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG,
                SensorConfig.LIMELIGHT_SERVO_SPAN_DEG,
                SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN);
        assertEquals(SensorConfig.LIMELIGHT_APRILTAG_UP_PITCH_DEG, upDeg, EPS);

        double downNorm = SensorConfig.pitchDegToServoNormalized(
                SensorConfig.LIMELIGHT_POLLEN_DOWN_PITCH_DEG,
                SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG,
                SensorConfig.LIMELIGHT_SERVO_SPAN_DEG,
                SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN);
        double downDeg = SensorConfig.servoNormalizedToPitchDeg(
                downNorm,
                SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG,
                SensorConfig.LIMELIGHT_SERVO_SPAN_DEG,
                SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN);
        assertEquals(SensorConfig.LIMELIGHT_POLLEN_DOWN_PITCH_DEG, downDeg, EPS);

        assertEquals(
                FieldConfig.HiveCell.AUDIENCE,
                FieldConfig.classifyHiveTagCell(FieldConfig.Alliance.RED, RED_AUDIENCE_TAG_SAMPLE));
        assertEquals(
                FieldConfig.HiveCell.SCORING,
                FieldConfig.classifyHiveTagCell(FieldConfig.Alliance.RED, RED_SCORING_TAG_SAMPLE));
        assertEquals(
                FieldConfig.HiveCell.AUDIENCE,
                FieldConfig.classifyHiveTagCell(FieldConfig.Alliance.BLUE, BLUE_AUDIENCE_TAG_SAMPLE));
        assertEquals(
                FieldConfig.HiveCell.SCORING,
                FieldConfig.classifyHiveTagCell(FieldConfig.Alliance.BLUE, BLUE_SCORING_TAG_SAMPLE));
        assertNull(FieldConfig.classifyHiveTagCell(FieldConfig.Alliance.RED, INVALID_TAG_ID));

        Vec2d target = new Vec2d();
        FieldConfig.getHiveTargetInto(FieldConfig.Alliance.RED, FieldConfig.HiveCell.AUDIENCE, target);
        assertEquals(EXPECTED_RED_HIVE_X_M, target.x, EPS);
        assertEquals(EXPECTED_AUDIENCE_Y_M, target.y, EPS);

        FieldConfig.getHiveTargetInto(FieldConfig.Alliance.BLUE, FieldConfig.HiveCell.SCORING, target);
        assertEquals(EXPECTED_BLUE_HIVE_X_M, target.x, EPS);
        assertEquals(EXPECTED_SCORING_Y_M, target.y, EPS);

        Pose2d redStart = new Pose2d();
        Pose2d blueStart = new Pose2d();
        FieldConfig.getStartingPoseInto(FieldConfig.Alliance.RED, FieldConfig.POSE_NUMBER_GARDEN, redStart);
        FieldConfig.getStartingPoseInto(FieldConfig.Alliance.BLUE, FieldConfig.POSE_NUMBER_GARDEN, blueStart);
        assertEquals(EXPECTED_FIELD_SIZE_M - redStart.x, blueStart.x, EPS);
        assertEquals(EXPECTED_FIELD_SIZE_M - redStart.y, blueStart.y, EPS);
        assertEquals(Math.PI, Math.abs(blueStart.heading), EPS);

        assertTrue(FieldConfig.isInsideFieldBounds(redStart.x, redStart.y));
        assertFalse(FieldConfig.isInsideFieldBounds(OUTSIDE_FIELD_COORD_M, OUTSIDE_FIELD_COORD_M));

        PathConstraints defaultConstraints = PursuitConfig.createDefaultConstraints();
        PathConstraints flowerConstraints = PursuitConfig.createFlowerApproachConstraints();
        assertEquals(EXPECTED_DEFAULT_MAX_VEL, defaultConstraints.maxVelMPerS, EPS);
        assertEquals(EXPECTED_APPROACH_MAX_VEL, flowerConstraints.maxVelMPerS, EPS);
    }

    @Test
    public void verifyCalibrationStoreAtomicJsonRoundTripAndSanitization() throws IOException {
        File calFile = tempFolder.newFile("kodi_calibration.json");
        CalibrationStore.ActiveCalibration cal = new CalibrationStore.ActiveCalibration();
        cal.camOffsetXM = TEST_CUSTOM_CAM_X_M;
        cal.aprilTagUpPitchDeg = TEST_CUSTOM_UP_PITCH_DEG;
        cal.pollenDownPitchDeg = TEST_CUSTOM_DOWN_PITCH_DEG;
        cal.pollenHMin = TEST_CUSTOM_H_MIN;
        cal.hoodMinClamp = INVALID_LOW_CLAMP;
        cal.hoodMaxClamp = INVALID_HIGH_CLAMP;
        cal.lutHoodPitches[ZERO_INT] = INVALID_LOW_CLAMP;
        cal.pinpointYawScalar = Double.NaN;

        assertTrue(CalibrationStore.saveToFile(calFile, cal));
        CalibrationStore.ActiveCalibration loaded = CalibrationStore.loadFromFile(calFile);

        assertEquals(TEST_CUSTOM_CAM_X_M, loaded.camOffsetXM, EPS);
        assertEquals(TEST_CUSTOM_UP_PITCH_DEG, loaded.aprilTagUpPitchDeg, EPS);
        assertEquals(TEST_CUSTOM_DOWN_PITCH_DEG, loaded.pollenDownPitchDeg, EPS);
        assertEquals(TEST_CUSTOM_H_MIN, loaded.pollenHMin, EPS);
        assertEquals(TurretConfig.HOOD_MIN_CLAMP, loaded.hoodMinClamp, EPS);
        assertEquals(TurretConfig.HOOD_MAX_CLAMP, loaded.hoodMaxClamp, EPS);
        assertEquals(TurretConfig.HOOD_MIN_CLAMP, loaded.lutHoodPitches[ZERO_INT], EPS);
        assertEquals(SensorConfig.PINPOINT_DEFAULT_YAW_SCALAR, loaded.pinpointYawScalar, EPS);

        BallisticProfile profile = loaded.buildBallisticProfile();
        assertTrue(profile.evaluateHoodPitch(INVALID_DISTANCE_M) >= TurretConfig.HOOD_MIN_CLAMP - EPS);
        assertTrue(profile.evaluateHoodPitch(INVALID_DISTANCE_M) <= TurretConfig.HOOD_MAX_CLAMP + EPS);
    }

    @Test
    public void verifyMatchSessionPersistenceAndFallbackResolution() throws IOException {
        File sessionFile = tempFolder.newFile("kodi_match_pose.json");
        Pose2d endPose = new Pose2d(TEST_POSE_X_M, TEST_POSE_Y_M, TEST_POSE_HEADING_RAD);
        assertTrue(MatchSession.saveAutoEndPose(
                sessionFile,
                FieldConfig.Alliance.BLUE,
                FieldConfig.POSE_NUMBER_SCORING,
                endPose));

        MatchSession.SessionData loaded = MatchSession.loadFromFile(sessionFile);
        assertEquals(FieldConfig.Alliance.BLUE, loaded.resolveAlliance());
        assertEquals(FieldConfig.POSE_NUMBER_SCORING, loaded.poseNumber);
        assertTrue(loaded.hasAutoEndPose);

        Pose2d resolved = new Pose2d();
        MatchSession.resolveInitialPoseInto(loaded, resolved);
        assertEquals(TEST_POSE_X_M, resolved.x, EPS);
        assertEquals(TEST_POSE_Y_M, resolved.y, EPS);
        assertEquals(TEST_POSE_HEADING_RAD, resolved.heading, EPS);

        loaded.hasAutoEndPose = false;
        MatchSession.resolveInitialPoseInto(loaded, resolved);
        Pose2d expectedStart = new Pose2d();
        FieldConfig.getStartingPoseInto(
                FieldConfig.Alliance.BLUE,
                FieldConfig.POSE_NUMBER_SCORING,
                expectedStart);
        assertEquals(expectedStart.x, resolved.x, EPS);
        assertEquals(expectedStart.y, resolved.y, EPS);
        assertEquals(expectedStart.heading, resolved.heading, EPS);
    }
}
