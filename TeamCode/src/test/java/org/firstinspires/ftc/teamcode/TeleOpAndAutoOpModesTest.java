package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.io.File;
import java.io.IOException;
import java.lang.reflect.Proxy;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.IntakeConfig;
import org.firstinspires.ftc.teamcode.config.MatchSession;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.opmodes.auto.BioBuzzAutoEngine;
import org.firstinspires.ftc.teamcode.opmodes.auto.BluePartnerAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.BlueSoloAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.RedPartnerAuto;
import org.firstinspires.ftc.teamcode.opmodes.auto.RedSoloAuto;
import org.firstinspires.ftc.teamcode.opmodes.teleop.BioBuzzTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;
import org.kodikas.ftclib.hardware.TelemetryCompact;
import org.kodikas.ftclib.math.MathUtil;

public final class TeleOpAndAutoOpModesTest {

    private static final int ZERO_INT = 0;
    private static final int ONE_INT = 1;
    private static final int TWO_INT = 2;
    private static final int FOUR_INT = 4;
    private static final int EIGHT_INT = 8;
    private static final int TWELVE_INT = 12;
    private static final int TAP_HOLD_TICKS = 5;
    private static final int SUSTAINED_HOLD_TICKS = 16;
    private static final int POLL_ENTER_TICKS = 4;
    private static final int POLL_CLEAR_TICKS = 7;
    private static final int MAX_SHOOT_TICKS = 400;
    private static final int MAX_AUTO_TICKS = 4000;
    private static final int TELEMETRY_RAPID_TICK_COUNT = 20;

    private static final double ZERO_DOUBLE = 0.0;
    private static final double ONE_DOUBLE = 1.0;
    private static final double QUARTER = 0.25;
    private static final double EPS = 1.0e-5;
    private static final double DT_TWENTY_MS = 0.020;
    private static final double DT_TEN_MS = 0.010;

    private static final float ZERO_FLOAT = 0.0f;
    private static final float STICK_FWD_INPUT = -0.8f;
    private static final float STICK_LEFT_INPUT = -0.5f;
    private static final float STICK_CCW_INPUT = -0.6f;
    private static final float TRIGGER_SHOOT_INPUT = 0.8f;

    private static final double TELEOP_AIM_X_M = 0.84;
    private static final double TELEOP_AIM_Y_M = 1.4366;
    private static final double TELEOP_BLIND_SPOT_HEADING_RAD = Math.PI;
    private static final double DIST_CLEAR_MM = 65.0;
    private static final double DIST_ENTER_MM = 28.0;
    private static final float YELLOW_R = 0.90f;
    private static final float YELLOW_G = 0.70f;
    private static final float YELLOW_B = 0.10f;

    private static final double VALID_LATENCY_MS = 20.0;
    private static final long VALID_STALENESS_MS = 10L;
    private static final double VALID_TAG_AREA = 1.5;
    private static final double VALID_TAG_DIST_M = 1.6;
    private static final int RED_SCORING_TAG_ID = FieldConfig.RED_SCORING_TAG_IDS[ZERO_INT];

    private static final String EXPECTED_GROUP = "BIOBUZZ";
    private static final String EXPECTED_PRESELECT_TELEOP = "BioBuzz TeleOp";
    private static final String SESSION_FILE_NAME = "kodi_match_pose.json";
    private static final String METHOD_SET_INTERVAL = "setMsTransmissionInterval";
    private static final String METHOD_UPDATE = "update";

    @Rule
    public TemporaryFolder tempFolder = new TemporaryFolder();

    private static final class MockColorProbe implements IntakeTransferSubsystem.ColorProximityProbe {

        private double distanceMm = DIST_CLEAR_MM;
        private float red = YELLOW_R;
        private float green = YELLOW_G;
        private float blue = YELLOW_B;

        private void setDistanceMm(double distanceMm) {
            this.distanceMm = distanceMm;
        }

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
    public void verifyBioBuzzTeleOpAllSixPlayStationControlsAndAutomation() {
        MockColorProbe probe = new MockColorProbe();
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                probe,
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                false);
        RobotState state = robot.getRobotState();
        robot.getLocalization().resetPose(TELEOP_AIM_X_M, TELEOP_AIM_Y_M, ZERO_DOUBLE);

        int[] configuredIntervalMs = new int[] {ZERO_INT};
        int[] telemetryUpdateCount = new int[] {ZERO_INT};
        Telemetry proxyTelemetry = (Telemetry) Proxy.newProxyInstance(
                Telemetry.class.getClassLoader(),
                new Class<?>[] {Telemetry.class},
                (proxy, method, args) -> {
                    String name = method.getName();
                    if (METHOD_SET_INTERVAL.equals(name) && args != null && args.length > ZERO_INT) {
                        configuredIntervalMs[ZERO_INT] = (Integer) args[ZERO_INT];
                    } else if (METHOD_UPDATE.equals(name)) {
                        telemetryUpdateCount[ZERO_INT] += ONE_INT;
                        return true;
                    }
                    return null;
                });

        Gamepad gamepad = new Gamepad();
        BioBuzzTeleOp teleOp = new BioBuzzTeleOp();
        teleOp.telemetry = proxyTelemetry;
        teleOp.setRobotForTest(robot);
        teleOp.setGamepadForTest(gamepad);
        teleOp.initialize();
        teleOp.startMatch();

        assertEquals(TelemetryCompact.DEFAULT_TRANSMISSION_INTERVAL_MS, configuredIntervalMs[ZERO_INT]);

        teleOp.tickLoop(DT_TWENTY_MS);
        assertEquals(TurretConfig.FLYWHEEL_WARM_IDLE_POWER, state.commandedFlywheelPower, EPS);

        gamepad.right_stick_y = STICK_FWD_INPUT;
        gamepad.right_stick_x = STICK_LEFT_INPUT;
        teleOp.tickLoop(DT_TWENTY_MS);
        double expectedVx = MathUtil.cubicCurve(
                -STICK_FWD_INPUT,
                ChassisConfig.JOYSTICK_DEADBAND,
                ChassisConfig.TRANSLATION_CUBIC_WEIGHT);
        double expectedVy = MathUtil.cubicCurve(
                -STICK_LEFT_INPUT,
                ChassisConfig.JOYSTICK_DEADBAND,
                ChassisConfig.TRANSLATION_CUBIC_WEIGHT);
        assertTrue(state.desiredDriveInput.vx > ZERO_DOUBLE);
        assertTrue(state.desiredDriveInput.vy > ZERO_DOUBLE);
        assertEquals(expectedVx, state.desiredDriveInput.vx, EPS);
        assertEquals(expectedVy, state.desiredDriveInput.vy, EPS);

        gamepad.left_stick_x = STICK_CCW_INPUT;
        teleOp.tickLoop(DT_TWENTY_MS);
        double expectedOmega = MathUtil.cubicCurve(
                -STICK_CCW_INPUT,
                ChassisConfig.JOYSTICK_DEADBAND,
                ChassisConfig.TURN_CUBIC_WEIGHT);
        assertTrue(state.desiredDriveInput.omega > ZERO_DOUBLE);
        assertEquals(expectedOmega, state.desiredDriveInput.omega, EPS);

        gamepad.right_stick_y = ZERO_FLOAT;
        gamepad.right_stick_x = STICK_LEFT_INPUT;
        gamepad.left_stick_x = ZERO_FLOAT;
        gamepad.right_bumper = false;
        teleOp.tickLoop(DT_TWENTY_MS);
        assertFalse(state.slowModeActive);
        double normalWheelFr = state.wheelPowerFR;
        assertTrue(normalWheelFr > ZERO_DOUBLE);

        gamepad.right_bumper = true;
        teleOp.tickLoop(DT_TWENTY_MS);
        assertTrue(state.slowModeActive);
        assertEquals(normalWheelFr * ChassisConfig.SLOW_MODE_SCALE, state.wheelPowerFR, EPS);

        gamepad.right_stick_x = ZERO_FLOAT;
        gamepad.right_bumper = false;
        teleOp.tickLoop(DT_TWENTY_MS);

        tapCross(teleOp, gamepad);
        assertEquals(IntakeState.INTAKE, state.intakeState);
        assertTrue(state.commandedFlywheelPower > TurretConfig.FLYWHEEL_WARM_IDLE_POWER);

        tapCross(teleOp, gamepad);
        assertEquals(IntakeState.IDLE, state.intakeState);

        tapCross(teleOp, gamepad);
        assertEquals(IntakeState.INTAKE, state.intakeState);
        robot.getIntakeTransfer().setPollenCount(FOUR_INT);
        teleOp.tickLoop(DT_TWENTY_MS);
        assertEquals(IntakeState.FULL, state.intakeState);
        assertEquals(FOUR_INT, state.pollenCount);

        tapCross(teleOp, gamepad);
        assertEquals(IntakeState.INTAKE, state.intakeState);

        tapCross(teleOp, gamepad);
        assertEquals(IntakeState.IDLE, state.intakeState);

        gamepad.cross = true;
        for (int i = ZERO_INT; i < SUSTAINED_HOLD_TICKS; i++) {
            teleOp.tickLoop(DT_TWENTY_MS);
        }
        assertEquals(IntakeState.REVERSING, state.intakeState);
        gamepad.cross = false;
        teleOp.tickLoop(DT_TWENTY_MS);
        assertEquals(IntakeState.IDLE, state.intakeState);

        tapCircle(teleOp, gamepad);
        assertTrue(state.plowDeployed);
        assertEquals(
                IntakeConfig.PLOW_DEPLOYED_POSITION,
                robot.getIntakeTransfer().getPlowServo().getLastWrittenPosition(),
                EPS);

        gamepad.circle = true;
        for (int i = ZERO_INT; i < SUSTAINED_HOLD_TICKS; i++) {
            teleOp.tickLoop(DT_TWENTY_MS);
        }
        assertFalse(state.plowDeployed);
        gamepad.circle = false;
        teleOp.tickLoop(DT_TWENTY_MS);
        assertFalse(state.plowDeployed);
        assertEquals(
                IntakeConfig.PLOW_LIFTED_POSITION,
                robot.getIntakeTransfer().getPlowServo().getLastWrittenPosition(),
                EPS);

        robot.getLocalization().resetPose(
                TELEOP_AIM_X_M,
                TELEOP_AIM_Y_M,
                TELEOP_BLIND_SPOT_HEADING_RAD);
        robot.getIntakeTransfer().setPollenCount(FOUR_INT);
        teleOp.tickLoop(DT_TWENTY_MS);
        assertTrue(state.shotSolution.inRearBlindSpot);

        gamepad.right_trigger = TRIGGER_SHOOT_INPUT;
        boolean observedBlindSpotAssist = false;
        boolean observedFeeding = false;
        boolean observedVolleyCleared = false;

        for (int tick = ZERO_INT; tick < MAX_SHOOT_TICKS; tick++) {
            teleOp.tickLoop(DT_TWENTY_MS);
            integrateWheelKinematics(robot, DT_TWENTY_MS);
            if (state.shootRequested
                    && Math.abs(state.shotSolution.headingAssistOmegaRadPerSec) > EPS) {
                observedBlindSpotAssist = true;
            }
            if (state.intakeState == IntakeState.FEEDINGTOTURRET) {
                observedFeeding = true;
            }
            if (observedFeeding
                    && state.intakeState == IntakeState.IDLE
                    && state.pollenCount == ZERO_INT
                    && state.scoredPollenCount == FOUR_INT) {
                observedVolleyCleared = true;
                break;
            }
        }

        assertTrue(observedBlindSpotAssist);
        assertTrue(observedFeeding);
        assertTrue(observedVolleyCleared);
        assertEquals(ZERO_INT, state.pollenCount);
        assertEquals(FOUR_INT, state.scoredPollenCount);

        gamepad.right_trigger = ZERO_FLOAT;
        teleOp.tickLoop(DT_TWENTY_MS);
        assertFalse(state.shootRequested);

        int updatesBeforeBurst = telemetryUpdateCount[ZERO_INT];
        for (int i = ZERO_INT; i < TELEMETRY_RAPID_TICK_COUNT; i++) {
            teleOp.tickLoop(DT_TWENTY_MS);
        }
        int updatesAfterBurst = telemetryUpdateCount[ZERO_INT];
        assertTrue(updatesAfterBurst - updatesBeforeBurst < TELEMETRY_RAPID_TICK_COUNT);

        teleOp.stopMatch();
    }

    @Test
    public void verifyFourAutoPresetsAnnotationsInitialTagCheckHiveTipAndPoseHandoff()
            throws IOException {
        RedSoloAuto redSolo = new RedSoloAuto();
        BlueSoloAuto blueSolo = new BlueSoloAuto();
        RedPartnerAuto redPartner = new RedPartnerAuto();
        BluePartnerAuto bluePartner = new BluePartnerAuto();

        verifyPresetMetadata(
                redSolo,
                FieldConfig.Alliance.RED,
                BioBuzzAutoEngine.AutoRoutine.SOLO,
                FieldConfig.POSE_NUMBER_GARDEN);
        verifyPresetMetadata(
                blueSolo,
                FieldConfig.Alliance.BLUE,
                BioBuzzAutoEngine.AutoRoutine.SOLO,
                FieldConfig.POSE_NUMBER_GARDEN);
        verifyPresetMetadata(
                redPartner,
                FieldConfig.Alliance.RED,
                BioBuzzAutoEngine.AutoRoutine.PARTNER,
                FieldConfig.POSE_NUMBER_SCORING);
        verifyPresetMetadata(
                bluePartner,
                FieldConfig.Alliance.BLUE,
                BioBuzzAutoEngine.AutoRoutine.PARTNER,
                FieldConfig.POSE_NUMBER_SCORING);

        File sessionFile = tempFolder.newFile(SESSION_FILE_NAME);
        MockColorProbe probe = new MockColorProbe();
        BioBuzzRobot autoRobot = new BioBuzzRobot(
                null,
                null,
                probe,
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_SCORING,
                true);
        RobotState autoState = autoRobot.getRobotState();
        autoState.activeHiveCell = FieldConfig.HiveCell.AUDIENCE;
        autoState.refreshActiveHiveTarget();

        redPartner.telemetry = null;
        redPartner.setRobotForTest(autoRobot);
        redPartner.setSessionFileForTest(sessionFile);
        redPartner.initialize();

        assertTrue(autoRobot.getVision().processAprilTagObservation(
                FieldConfig.RED_START_POSE_2_X_M - FieldConfig.FIELD_HALF_METERS,
                FieldConfig.RED_START_POSE_2_Y_M - FieldConfig.FIELD_HALF_METERS,
                FieldConfig.RED_START_POSE_2_HEADING_RAD,
                VALID_LATENCY_MS,
                VALID_STALENESS_MS,
                VALID_TAG_AREA,
                VALID_TAG_DIST_M,
                RED_SCORING_TAG_ID));
        redPartner.tickInitLoop();

        assertTrue(autoState.initialHiveTagConfirmed);
        assertEquals(FieldConfig.HiveCell.SCORING, autoState.activeHiveCell);

        redPartner.startMatch();

        boolean observedVolley1 = false;
        boolean observedHiveTipSwitch = false;
        boolean observedPlowDeployed = false;
        boolean observedVolley3 = false;
        int intakeCycleTick = ZERO_INT;

        for (int tick = ZERO_INT; tick < MAX_AUTO_TICKS; tick++) {
            if (autoState.intakeState == IntakeState.INTAKE) {
                int period = POLL_ENTER_TICKS + POLL_CLEAR_TICKS;
                int phase = intakeCycleTick % period;
                probe.setDistanceMm((phase < POLL_ENTER_TICKS) ? DIST_ENTER_MM : DIST_CLEAR_MM);
                intakeCycleTick++;
            } else {
                probe.setDistanceMm(DIST_CLEAR_MM);
                intakeCycleTick = ZERO_INT;
            }

            redPartner.tickLoop(DT_TEN_MS);
            integrateWheelKinematics(autoRobot, DT_TEN_MS);

            if (autoState.scoredPollenCount == FOUR_INT) {
                observedVolley1 = true;
            }
            if (autoState.scoredPollenCount == EIGHT_INT
                    && autoState.hiveTipped
                    && autoState.activeHiveCell == FieldConfig.HiveCell.AUDIENCE) {
                observedHiveTipSwitch = true;
            }
            if (autoState.plowDeployed) {
                observedPlowDeployed = true;
            }
            if (autoState.scoredPollenCount == TWELVE_INT) {
                observedVolley3 = true;
            }
            if (redPartner.isAutoFinished()) {
                break;
            }
        }

        assertTrue(observedVolley1);
        assertTrue(observedHiveTipSwitch);
        assertTrue(observedPlowDeployed);
        assertTrue(observedVolley3);
        assertTrue(redPartner.isAutoFinished());
        assertEquals(TWELVE_INT, autoState.scoredPollenCount);
        assertTrue(autoState.hiveTipped);
        assertEquals(FieldConfig.HiveCell.AUDIENCE, autoState.activeHiveCell);

        double expectedEndX = autoState.pose.x;
        double expectedEndY = autoState.pose.y;
        double expectedEndHeading = autoState.pose.heading;

        redPartner.stopMatch();
        assertTrue(redPartner.isAutoPoseSaved());

        MatchSession.SessionData loadedSession = MatchSession.loadFromFile(sessionFile);
        assertTrue(loadedSession.hasAutoEndPose);
        assertEquals(FieldConfig.Alliance.RED, loadedSession.resolveAlliance());
        assertEquals(FieldConfig.POSE_NUMBER_SCORING, loadedSession.poseNumber);
        assertEquals(expectedEndX, loadedSession.x, EPS);
        assertEquals(expectedEndY, loadedSession.y, EPS);
        assertEquals(expectedEndHeading, loadedSession.headingRad, EPS);

        BioBuzzRobot teleOpRobot = new BioBuzzRobot(
                null,
                loadedSession,
                new MockColorProbe(),
                loadedSession.resolveAlliance(),
                loadedSession.poseNumber,
                false);
        RobotState teleOpState = teleOpRobot.getRobotState();
        assertEquals(expectedEndX, teleOpState.pose.x, EPS);
        assertEquals(expectedEndY, teleOpState.pose.y, EPS);
        assertEquals(expectedEndHeading, teleOpState.pose.heading, EPS);
    }

    private static void verifyPresetMetadata(
            BioBuzzAutoEngine autoOpMode,
            FieldConfig.Alliance expectedAlliance,
            BioBuzzAutoEngine.AutoRoutine expectedRoutine,
            int expectedPoseNumber) {
        Autonomous annotation = autoOpMode.getClass().getAnnotation(Autonomous.class);
        assertNotNull(annotation);
        assertEquals(EXPECTED_GROUP, annotation.group());
        assertEquals(EXPECTED_PRESELECT_TELEOP, annotation.preselectTeleOp());
        assertEquals(expectedAlliance, autoOpMode.getAlliance());
        assertEquals(expectedRoutine, autoOpMode.getRoutine());
        assertEquals(expectedPoseNumber, autoOpMode.getStartPoseNumber());
    }

    private static void tapCross(BioBuzzTeleOp teleOp, Gamepad gamepad) {
        gamepad.cross = true;
        for (int i = ZERO_INT; i < TAP_HOLD_TICKS; i++) {
            teleOp.tickLoop(DT_TWENTY_MS);
        }
        gamepad.cross = false;
        teleOp.tickLoop(DT_TWENTY_MS);
    }

    private static void tapCircle(BioBuzzTeleOp teleOp, Gamepad gamepad) {
        gamepad.circle = true;
        for (int i = ZERO_INT; i < TAP_HOLD_TICKS; i++) {
            teleOp.tickLoop(DT_TWENTY_MS);
        }
        gamepad.circle = false;
        teleOp.tickLoop(DT_TWENTY_MS);
    }

    private static void integrateWheelKinematics(BioBuzzRobot robot, double dtSeconds) {
        RobotState state = robot.getRobotState();

        double vxNorm = QUARTER * (state.wheelPowerFL
                + state.wheelPowerFR
                + state.wheelPowerBL
                + state.wheelPowerBR);
        double vyComp = QUARTER * (-state.wheelPowerFL
                + state.wheelPowerFR
                + state.wheelPowerBL
                - state.wheelPowerBR);
        double vyNorm = vyComp / ChassisConfig.LATERAL_MULTIPLIER;
        double omegaNorm = QUARTER * (-state.wheelPowerFL
                + state.wheelPowerFR
                - state.wheelPowerBL
                + state.wheelPowerBR);

        double vxRobot = vxNorm * ChassisConfig.MAX_FREE_SPEED_M_PER_S;
        double vyRobot = vyNorm * ChassisConfig.MAX_FREE_SPEED_M_PER_S;
        double omega = omegaNorm * ChassisConfig.MAX_OMEGA_RAD_PER_SEC;

        double cosH = Math.cos(state.pose.heading);
        double sinH = Math.sin(state.pose.heading);
        double vxField = vxRobot * cosH - vyRobot * sinH;
        double vyField = vxRobot * sinH + vyRobot * cosH;

        double nextX = state.rawPinpointPose.x + vxField * dtSeconds;
        double nextY = state.rawPinpointPose.y + vyField * dtSeconds;
        double nextHeading = MathUtil.wrapAngleRad(state.rawPinpointPose.heading + omega * dtSeconds);

        robot.getLocalization().injectSimulatedOdometry(
                nextX,
                nextY,
                nextHeading,
                vxField,
                vyField,
                omega,
                dtSeconds);
    }
}
