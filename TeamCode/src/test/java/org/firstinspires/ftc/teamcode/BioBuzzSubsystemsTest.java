package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.junit.Test;
import org.kodikas.ftclib.math.Pose2d;

public class BioBuzzSubsystemsTest {

    private static final double EPS = 1.0e-5;
    private static final int ZERO_INT = 0;
    private static final int ONE_INT = 1;
    private static final int FOUR_INT = 4;
    private static final int EIGHT_INT = 8;
    private static final int TEN_INT = 10;
    private static final int TWENTY_INT = 20;
    private static final int THIRTY_INT = 30;
    private static final long ZERO_LONG = 0L;
    private static final double ZERO_DOUBLE = 0.0;
    private static final double ONE_DOUBLE = 1.0;
    private static final double DT_TEN_MS = 0.010;
    private static final double DT_TWENTY_FIVE_MS = 0.025;
    private static final double SIM_STEP_X_M = 0.05;
    private static final double SIM_GLITCH_JUMP_M = 1.25;
    private static final double SMALL_VISION_OFFSET_M = 0.10;
    private static final double SHORT_BLANKOUT_SEC = 0.05;
    private static final double VALID_LATENCY_SEC = 0.02;
    private static final double VALID_LATENCY_MS = 20.0;
    private static final long VALID_STALENESS_MS = 10L;
    private static final double VALID_TAG_AREA = 1.5;
    private static final double VALID_TAG_DIST_M = 1.4;
    private static final int RED_TAG_ID = 34;
    private static final double DROOP_VOLTAGE = 10.5;
    private static final double TEST_AIM_POSE_X_M = 0.65;
    private static final double TEST_AIM_POSE_Y_M = 1.4366;
    private static final double TEST_REAR_HEADING_RAD = Math.PI;
    private static final double DRIVE_FWD_FULL = 1.0;
    private static final double DRIVE_STRAFE_FULL = 1.0;
    private static final double DRIVE_OMEGA_FULL = 1.0;
    private static final double DRIVE_HALF = 0.5;
    private static final double DIST_CLEAR_MM = 65.0;
    private static final double DIST_ENTER_MM = 28.0;
    private static final float YELLOW_R = 0.90f;
    private static final float YELLOW_G = 0.70f;
    private static final float YELLOW_B = 0.10f;
    private static final float BLUE_R = 0.10f;
    private static final float BLUE_G = 0.20f;
    private static final float BLUE_B = 0.90f;

    private static final class MockColorProbe implements IntakeTransferSubsystem.ColorProximityProbe {

        private double distanceMm = DIST_CLEAR_MM;
        private float red = YELLOW_R;
        private float green = YELLOW_G;
        private float blue = YELLOW_B;

        private void setReading(double distanceMm, float r, float g, float b) {
            this.distanceMm = distanceMm;
            this.red = r;
            this.green = g;
            this.blue = b;
        }

        @Override
        public double readDistanceMm() {
            return distanceMm;
        }

        @Override
        public void readNormalizedRgbInto(float[] outRgb) {
            outRgb[ZERO_INT] = red;
            outRgb[ONE_INT] = green;
            outRgb[ONE_INT + ONE_INT] = blue;
        }
    }

    @Test
    public void verifyLocalizationEsdGuardAndAdvisoryVisionFusion() {
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                new MockColorProbe(),
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                true);
        RobotState state = robot.getRobotState();
        Pose2d startPose = new Pose2d();
        startPose.copyFrom(state.pose);

        assertTrue(robot.getLocalization().injectSimulatedOdometry(
                startPose.x + SIM_STEP_X_M,
                startPose.y,
                startPose.heading,
                ZERO_DOUBLE,
                ZERO_DOUBLE,
                ZERO_DOUBLE,
                DT_TEN_MS));
        assertEquals(ZERO_INT, state.esdGlitchCount);

        assertFalse(robot.getLocalization().injectSimulatedOdometry(
                startPose.x + SIM_GLITCH_JUMP_M,
                startPose.y,
                startPose.heading,
                ZERO_DOUBLE,
                ZERO_DOUBLE,
                ZERO_DOUBLE,
                DT_TEN_MS));
        assertEquals(ONE_INT, state.esdGlitchCount);

        assertFalse(robot.getLocalization().injectSimulatedOdometry(
                Double.NaN,
                startPose.y,
                startPose.heading,
                ZERO_DOUBLE,
                ZERO_DOUBLE,
                ZERO_DOUBLE,
                DT_TEN_MS));
        assertEquals(ONE_INT + ONE_INT, state.esdGlitchCount);

        state.timeSinceCameraPitchMoveSec = SHORT_BLANKOUT_SEC;
        state.visionMeasurement.set(
                state.pose.x + SMALL_VISION_OFFSET_M,
                state.pose.y,
                state.pose.heading,
                VALID_LATENCY_SEC,
                VALID_TAG_AREA,
                VALID_TAG_DIST_M,
                RED_TAG_ID,
                true);
        state.visionMeasurementFresh = true;
        robot.getLocalization().compute(DT_TEN_MS);
        assertEquals(ONE_INT, state.visionRejectedCount);

        double beforeFusionX = state.pose.x;
        double parallaxX = robot.getCalibration().camOffsetXM
                - robot.getCalibration().turretPivotOffsetXM;
        assertTrue(robot.getVision().processAprilTagObservation(
                beforeFusionX + parallaxX - FieldConfig.FIELD_HALF_METERS + SMALL_VISION_OFFSET_M,
                state.pose.y - FieldConfig.FIELD_HALF_METERS,
                state.pose.heading,
                VALID_LATENCY_MS,
                VALID_STALENESS_MS,
                VALID_TAG_AREA,
                VALID_TAG_DIST_M,
                RED_TAG_ID));
        robot.tick(DT_TEN_MS);
        assertEquals(ONE_INT, state.visionAcceptedCount);
        assertTrue(state.pose.x > beforeFusionX);
        assertTrue((state.pose.x - beforeFusionX) <= SensorConfig.VISION_MAX_NUDGE_M + EPS);
    }

    @Test
    public void verifyTurretTwoStageWarmIdleVoltageLockAndStow() {
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                new MockColorProbe(),
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                true);
        RobotState state = robot.getRobotState();
        robot.getLocalization().resetPose(TEST_AIM_POSE_X_M, TEST_AIM_POSE_Y_M, ZERO_DOUBLE);

        robot.tick(DT_TEN_MS);
        assertEquals(TurretConfig.FLYWHEEL_WARM_IDLE_POWER, state.commandedFlywheelPower, EPS);
        assertFalse(state.flywheelAtScoringSpeed);
        assertFalse(state.readyToFire);

        robot.getIntakeTransfer().requestState(IntakeState.INTAKE);
        for (int i = ZERO_INT; i < TWENTY_INT; i++) {
            robot.tick(DT_TEN_MS);
        }
        assertTrue(state.commandedFlywheelPower > TurretConfig.FLYWHEEL_WARM_IDLE_POWER);
        assertTrue(state.flywheelAtScoringSpeed);
        assertTrue(state.commandedHoodPitch >= TurretConfig.HOOD_MIN_CLAMP - EPS);
        assertTrue(state.commandedHoodPitch <= TurretConfig.HOOD_MAX_CLAMP + EPS);

        robot.getHubManager().setSimulatedBatteryVoltage(DROOP_VOLTAGE);
        robot.tick(DT_TEN_MS);
        double expectedVoltageScale = IntakeConfig.NOMINAL_BATTERY_VOLTAGE / DROOP_VOLTAGE;
        double expectedApplied = Math.min(
                ONE_DOUBLE,
                state.commandedFlywheelPower * expectedVoltageScale);
        assertEquals(
                expectedApplied,
                robot.getTurret().getLeftFlywheel().getLastAppliedPower(),
                EPS);

        robot.getIntakeTransfer().setPlowDeployed(true);
        state.shootRequested = false;
        robot.tick(DT_TEN_MS);
        assertTrue(state.shotSolution.stowed);
        assertEquals(ZERO_DOUBLE, state.shotSolution.turretYawRad, EPS);
        assertEquals(TurretConfig.TURRET_CENTER_PWM, state.shotSolution.turretServoNormalized, EPS);

        robot.getIntakeTransfer().setPlowDeployed(false);
        robot.getLocalization().resetPose(TEST_AIM_POSE_X_M, TEST_AIM_POSE_Y_M, TEST_REAR_HEADING_RAD);
        robot.tick(DT_TEN_MS);
        assertTrue(state.shotSolution.inRearBlindSpot);
        assertTrue(state.shotSolution.stowed);

        state.shootRequested = true;
        robot.tick(DT_TEN_MS);
        assertFalse(state.shotSolution.stowed);
        assertTrue(Math.abs(state.shotSolution.headingAssistOmegaRadPerSec) > EPS);
    }

    @Test
    public void verifyDrivetrainMecanumLateralMultiplierAndDesaturation() {
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                new MockColorProbe(),
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                true);
        RobotState state = robot.getRobotState();
        state.driveCentricity = ChassisConfig.DriveCentricity.ROBOT_CENTRIC;
        state.turretAssistEnabled = false;
        robot.getDrivetrain().setDriveCommand(ZERO_DOUBLE, DRIVE_HALF, ZERO_DOUBLE, true);
        robot.tick(DT_TEN_MS);

        double expectedLateral = DRIVE_HALF * ChassisConfig.LATERAL_MULTIPLIER;
        assertEquals(-expectedLateral, state.wheelPowerFL, EPS);
        assertEquals(expectedLateral, state.wheelPowerFR, EPS);
        assertEquals(expectedLateral, state.wheelPowerBL, EPS);
        assertEquals(-expectedLateral, state.wheelPowerBR, EPS);

        state.slowModeActive = true;
        robot.tick(DT_TEN_MS);
        assertEquals(expectedLateral * ChassisConfig.SLOW_MODE_SCALE, state.wheelPowerFR, EPS);

        state.slowModeActive = false;
        robot.getDrivetrain().setDriveCommand(DRIVE_FWD_FULL, DRIVE_STRAFE_FULL, DRIVE_OMEGA_FULL, true);
        robot.tick(DT_TEN_MS);
        double maxWheel = Math.max(
                Math.max(Math.abs(state.wheelPowerFL), Math.abs(state.wheelPowerFR)),
                Math.max(Math.abs(state.wheelPowerBL), Math.abs(state.wheelPowerBR)));
        assertEquals(ONE_DOUBLE, maxWheel, EPS);
    }

    @Test
    public void verifyIntakeStateGatedPollingFourBallCutoffAndOuttakeVolleyTip() {
        MockColorProbe probe = new MockColorProbe();
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                probe,
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                true);
        RobotState state = robot.getRobotState();
        robot.getLocalization().resetPose(TEST_AIM_POSE_X_M, TEST_AIM_POSE_Y_M, ZERO_DOUBLE);

        for (int i = ZERO_INT; i < TEN_INT; i++) {
            robot.tick(DT_TWENTY_FIVE_MS);
        }
        assertEquals(ZERO_LONG, state.colorSensorPollCount);

        robot.getIntakeTransfer().requestState(IntakeState.INTAKE);
        probe.setReading(DIST_ENTER_MM, BLUE_R, BLUE_G, BLUE_B);
        robot.tick(DT_TWENTY_FIVE_MS);
        assertEquals(ONE_INT, (int) state.colorSensorPollCount);
        assertEquals(ZERO_INT, state.pollenCount);

        for (int ball = ZERO_INT; ball < FOUR_INT; ball++) {
            probe.setReading(DIST_CLEAR_MM, YELLOW_R, YELLOW_G, YELLOW_B);
            for (int waitTick = ZERO_INT; waitTick < FOUR_INT; waitTick++) {
                robot.tick(DT_TWENTY_FIVE_MS);
            }
            probe.setReading(DIST_ENTER_MM, YELLOW_R, YELLOW_G, YELLOW_B);
            robot.tick(DT_TWENTY_FIVE_MS);
        }

        assertEquals(FOUR_INT, state.pollenCount);
        assertEquals(IntakeState.FULL, state.intakeState);
        assertEquals(ZERO_DOUBLE, robot.getIntakeTransfer().getTransferMotor().getTargetPower(), EPS);

        long pollsWhenFull = state.colorSensorPollCount;
        for (int i = ZERO_INT; i < TEN_INT; i++) {
            robot.tick(DT_TWENTY_FIVE_MS);
        }
        assertEquals(pollsWhenFull, state.colorSensorPollCount);

        state.shootRequested = true;
        robot.getIntakeTransfer().startFeeding();
        for (int i = ZERO_INT; i < THIRTY_INT; i++) {
            robot.tick(DT_TWENTY_FIVE_MS);
            if (state.intakeState == IntakeState.IDLE) {
                break;
            }
        }
        assertEquals(IntakeState.IDLE, state.intakeState);
        assertEquals(ZERO_INT, state.pollenCount);
        assertEquals(FOUR_INT, state.scoredPollenCount);
        assertFalse(state.hiveTipped);

        FieldConfig.HiveCell initialCell = state.activeHiveCell;
        robot.getIntakeTransfer().setPollenCount(FOUR_INT);
        robot.getIntakeTransfer().startFeeding();
        for (int i = ZERO_INT; i < THIRTY_INT; i++) {
            robot.tick(DT_TWENTY_FIVE_MS);
            if (state.intakeState == IntakeState.IDLE) {
                break;
            }
        }
        assertEquals(EIGHT_INT, state.scoredPollenCount);
        assertTrue(state.hiveTipped);
        assertNotEquals(initialCell, state.activeHiveCell);
        assertEquals(FieldConfig.getOppositeHiveCell(initialCell), state.activeHiveCell);
    }

    @Test
    public void verifyZeroPollenShootAndFeedDoesNotDeadlock() {
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                new MockColorProbe(),
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                true);
        RobotState state = robot.getRobotState();
        robot.getLocalization().resetPose(TEST_AIM_POSE_X_M, TEST_AIM_POSE_Y_M, ZERO_DOUBLE);

        robot.tick(DT_TWENTY_FIVE_MS);
        assertEquals(ZERO_INT, state.pollenCount);
        assertEquals(IntakeState.IDLE, state.intakeState);
        assertEquals(TurretConfig.FLYWHEEL_WARM_IDLE_POWER, state.commandedFlywheelPower, EPS);
        assertFalse(state.flywheelAtScoringSpeed);
        assertFalse(state.readyToFire);

        state.shootRequested = true;
        robot.getIntakeTransfer().startFeeding();
        boolean observedOuttakeDriving = false;
        boolean returnedToIdle = false;
        for (int i = ZERO_INT; i < THIRTY_INT; i++) {
            robot.tick(DT_TWENTY_FIVE_MS);
            if (robot.getIntakeTransfer().getOuttakeMotor().getLastAppliedPower() > EPS) {
                observedOuttakeDriving = true;
            }
            if (observedOuttakeDriving && state.intakeState == IntakeState.IDLE) {
                returnedToIdle = true;
                break;
            }
        }
        assertTrue(state.commandedFlywheelPower > TurretConfig.FLYWHEEL_WARM_IDLE_POWER);
        assertTrue(state.flywheelAtScoringSpeed);
        assertTrue(state.readyToFire);
        assertTrue(observedOuttakeDriving);
        assertTrue(returnedToIdle);
        assertEquals(IntakeState.IDLE, state.intakeState);
        assertEquals(ZERO_INT, state.scoredPollenCount);
    }
}
