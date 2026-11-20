package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.commands.AutoTrajectories;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.PlowDislodgeAndSweepCommand;
import org.firstinspires.ftc.teamcode.commands.ShootVolleyCommand;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.PursuitConfig;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.junit.Test;
import org.kodikas.ftclib.kodipursuit.KodiPathChain;
import org.kodikas.ftclib.kodipursuit.KodiPathSegment;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

public final class CommandsAndTrajectoriesTest {

    private static final int ZERO_INT = 0;
    private static final int ONE_INT = 1;
    private static final int TWO_INT = 2;
    private static final int FOUR_INT = 4;
    private static final int POLL_ENTER_TICKS = 4;
    private static final int POLL_CLEAR_TICKS = 7;
    private static final int TURRET_PRE_AIM_TICKS = 30;
    private static final int MAX_FOLLOW_TICKS = 1000;
    private static final int MAX_PLOW_SWEEP_TICKS = 1500;
    private static final int MAX_SHOOT_VOLLEY_TICKS = 800;
    private static final int TRAJECTORY_SAMPLE_COUNT = 201;
    private static final int TRAJECTORY_SAMPLE_DIVISOR = TRAJECTORY_SAMPLE_COUNT - ONE_INT;

    private static final double ZERO_DOUBLE = 0.0;
    private static final double ONE_DOUBLE = 1.0;
    private static final double QUARTER = 0.25;
    private static final double EPS = 1.0e-5;
    private static final double SYMMETRY_EPS = 1.0e-6;
    private static final double DT_TEN_MS = 0.010;
    private static final double PRE_AIM_HEADING_RAD = -0.45;
    private static final double SHOOT_BLIND_SPOT_X_M = 0.84;
    private static final double SHOOT_BLIND_SPOT_Y_M = 1.4366;
    private static final double SHOOT_BLIND_SPOT_HEADING_RAD = Math.PI;
    private static final double DIST_CLEAR_MM = 65.0;
    private static final double DIST_ENTER_MM = 28.0;
    private static final float YELLOW_R = 0.90f;
    private static final float YELLOW_G = 0.70f;
    private static final float YELLOW_B = 0.10f;

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
    public void verifyFollowPathCommandConvergesToSegmentEndAndStopsDrivetrain() {
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                new MockColorProbe(),
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                true);
        RobotState state = robot.getRobotState();
        AutoTrajectories.SoloTrajectorySet solo = AutoTrajectories.buildSolo(FieldConfig.Alliance.RED);
        KodiPathChain path = solo.gardenSweep;
        FollowPathCommand followCommand = new FollowPathCommand(
                path,
                robot.getDrivetrain(),
                state,
                robot.getScheduler());

        assertTrue(state.turretAssistEnabled);
        robot.getScheduler().schedule(followCommand);
        assertFalse(state.turretAssistEnabled);

        for (int tick = ZERO_INT; tick < MAX_FOLLOW_TICKS; tick++) {
            stepKinematicRobot(robot, DT_TEN_MS);
            if (followCommand.isFinished() && !robot.getScheduler().isScheduled(followCommand)) {
                break;
            }
        }

        assertTrue(followCommand.isFinished());
        assertFalse(robot.getScheduler().isScheduled(followCommand));
        assertTrue(state.turretAssistEnabled);
        assertEquals(ZERO_DOUBLE, state.desiredDriveInput.vx, EPS);
        assertEquals(ZERO_DOUBLE, state.desiredDriveInput.vy, EPS);
        assertEquals(ZERO_DOUBLE, state.desiredDriveInput.omega, EPS);

        double distToEnd = Math.hypot(
                AutoTrajectories.SOLO_GARDEN_END_X_M - state.pose.x,
                AutoTrajectories.SOLO_GARDEN_END_Y_M - state.pose.y);
        double headingErr = Math.abs(MathUtil.angleDiffRad(
                AutoTrajectories.SOLO_GARDEN_END_HEADING_RAD,
                state.pose.heading));
        assertTrue(distToEnd <= PursuitConfig.POSITION_TOLERANCE_M + EPS);
        assertTrue(headingErr <= PursuitConfig.HEADING_TOLERANCE_RAD + EPS);
    }

    @Test
    public void verifyPlowDislodgeAndSweepStowsTurretFirstThenDeploysPlowAndRacesFullEarlyExit() {
        MockColorProbe probe = new MockColorProbe();
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                probe,
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                true);
        RobotState state = robot.getRobotState();
        AutoTrajectories.SoloTrajectorySet solo = AutoTrajectories.buildSolo(FieldConfig.Alliance.RED);

        robot.getLocalization().resetPose(
                AutoTrajectories.AUDIENCE_FLOWER_STAGE_X_M,
                AutoTrajectories.AUDIENCE_FLOWER_Y_M,
                PRE_AIM_HEADING_RAD);
        for (int i = ZERO_INT; i < TURRET_PRE_AIM_TICKS; i++) {
            robot.tick(DT_TEN_MS);
        }
        assertTrue(Math.abs(state.estimatedTurretYawRad) > TurretConfig.TURRET_STOW_TOLERANCE_RAD);

        PlowDislodgeAndSweepCommand command = new PlowDislodgeAndSweepCommand(
                solo.enterAudienceFlower,
                solo.reverseAudienceFlower,
                solo.sweepAudienceFlower,
                robot.getDrivetrain(),
                robot.getIntakeTransfer(),
                robot.getVision(),
                state,
                robot.getScheduler());

        robot.getScheduler().schedule(command);

        boolean observedStowGateBlockedPlow = false;
        boolean observedPlowDeployedAfterStow = false;
        boolean observedEnterReached = false;
        boolean observedReverseReached = false;
        boolean observedPollenFloorSweepArmed = false;
        boolean observedFullEarlyExit = false;
        int intakeCycleTick = ZERO_INT;

        for (int tick = ZERO_INT; tick < MAX_PLOW_SWEEP_TICKS; tick++) {
            if (Math.abs(state.estimatedTurretYawRad) > TurretConfig.TURRET_STOW_TOLERANCE_RAD) {
                assertFalse(state.plowDeployed);
                observedStowGateBlockedPlow = true;
            }

            if (state.intakeState == IntakeState.INTAKE) {
                int cyclePeriod = POLL_ENTER_TICKS + POLL_CLEAR_TICKS;
                int phase = intakeCycleTick % cyclePeriod;
                probe.setDistanceMm((phase < POLL_ENTER_TICKS) ? DIST_ENTER_MM : DIST_CLEAR_MM);
                intakeCycleTick++;
            } else {
                probe.setDistanceMm(DIST_CLEAR_MM);
                intakeCycleTick = ZERO_INT;
            }

            stepKinematicRobot(robot, DT_TEN_MS);

            if (state.plowDeployed) {
                assertTrue(Math.abs(state.estimatedTurretYawRad) <= TurretConfig.TURRET_STOW_TOLERANCE_RAD);
                observedPlowDeployedAfterStow = true;
            }
            if (observedPlowDeployedAfterStow
                    && Math.abs(state.pose.x - AutoTrajectories.AUDIENCE_FLOWER_ENTER_X_M)
                            <= PursuitConfig.POSITION_TOLERANCE_M + EPS) {
                observedEnterReached = true;
            }
            if (observedEnterReached
                    && Math.abs(state.pose.x - AutoTrajectories.AUDIENCE_FLOWER_REVERSE_X_M)
                            <= PursuitConfig.POSITION_TOLERANCE_M + EPS) {
                observedReverseReached = true;
            }
            if (observedReverseReached
                    && !state.plowDeployed
                    && robot.getVision().getMode() == VisionSubsystem.VisionMode.POLLEN_FLOOR
                    && state.intakeState == IntakeState.INTAKE) {
                observedPollenFloorSweepArmed = true;
            }
            if (state.intakeState == IntakeState.FULL) {
                observedFullEarlyExit = true;
            }
            if (command.isFinished() && !robot.getScheduler().isScheduled(command)) {
                break;
            }
        }

        assertTrue(observedStowGateBlockedPlow);
        assertTrue(observedPlowDeployedAfterStow);
        assertTrue(observedEnterReached);
        assertTrue(observedReverseReached);
        assertTrue(observedPollenFloorSweepArmed);
        assertTrue(observedFullEarlyExit);
        assertTrue(command.isFinished());
        assertFalse(robot.getScheduler().isScheduled(command));
        assertEquals(FOUR_INT, state.pollenCount);
        assertEquals(IntakeState.FULL, state.intakeState);
        assertFalse(state.plowDeployed);
        assertEquals(VisionSubsystem.VisionMode.APRILTAG_HORIZON, robot.getVision().getMode());
        assertTrue(state.pose.x > AutoTrajectories.AUDIENCE_FLOWER_SWEEP_X_M
                + PursuitConfig.POSITION_TOLERANCE_M);
        assertEquals(ZERO_DOUBLE, state.desiredDriveInput.vx, EPS);
        assertEquals(ZERO_DOUBLE, state.desiredDriveInput.vy, EPS);
        assertEquals(ZERO_DOUBLE, state.desiredDriveInput.omega, EPS);
    }

    @Test
    public void verifyShootVolleyCommandBlindSpotTurnInSchmittLockAndOuttakeDisplacementClear() {
        BioBuzzRobot robot = new BioBuzzRobot(
                null,
                null,
                new MockColorProbe(),
                FieldConfig.Alliance.RED,
                FieldConfig.POSE_NUMBER_GARDEN,
                true);
        RobotState state = robot.getRobotState();
        robot.getLocalization().resetPose(
                SHOOT_BLIND_SPOT_X_M,
                SHOOT_BLIND_SPOT_Y_M,
                SHOOT_BLIND_SPOT_HEADING_RAD);
        robot.getIntakeTransfer().setPollenCount(FOUR_INT);

        robot.tick(DT_TEN_MS);
        assertTrue(state.shotSolution.inRearBlindSpot);

        ShootVolleyCommand shootCommand = new ShootVolleyCommand(robot.getIntakeTransfer(), state);
        robot.getScheduler().schedule(shootCommand);

        boolean observedBlindSpotTurnIn = false;
        boolean observedSchmittLock = false;
        boolean observedFeeding = false;
        boolean observedFeedDisplacement = false;

        for (int tick = ZERO_INT; tick < MAX_SHOOT_VOLLEY_TICKS; tick++) {
            stepKinematicRobot(robot, DT_TEN_MS);
            if (Math.abs(state.shotSolution.headingAssistOmegaRadPerSec) > EPS) {
                observedBlindSpotTurnIn = true;
            }
            if (state.readyToFire) {
                observedSchmittLock = true;
            }
            if (state.intakeState == IntakeState.FEEDINGTOTURRET) {
                observedFeeding = true;
            }
            if (state.outtakeFeedDisplacementM > EPS) {
                observedFeedDisplacement = true;
            }
            if (shootCommand.isFinished() && !robot.getScheduler().isScheduled(shootCommand)) {
                break;
            }
        }

        assertTrue(observedBlindSpotTurnIn);
        assertTrue(observedSchmittLock);
        assertTrue(observedFeeding);
        assertTrue(observedFeedDisplacement);
        assertTrue(shootCommand.isFinished());
        assertFalse(robot.getScheduler().isScheduled(shootCommand));
        assertEquals(ZERO_INT, state.pollenCount);
        assertEquals(IntakeState.IDLE, state.intakeState);
        assertEquals(FOUR_INT, state.scoredPollenCount);
    }

    @Test
    public void verifyAutoTrajectoriesBoundsHiveKeepoutSymmetryAndPartnerSafety() {
        AutoTrajectories.SoloTrajectorySet redSolo =
                AutoTrajectories.buildSolo(FieldConfig.Alliance.RED);
        AutoTrajectories.SoloTrajectorySet blueSolo =
                AutoTrajectories.buildSolo(FieldConfig.Alliance.BLUE);
        AutoTrajectories.PartnerTrajectorySet redPartner =
                AutoTrajectories.buildPartner(FieldConfig.Alliance.RED);
        AutoTrajectories.PartnerTrajectorySet bluePartner =
                AutoTrajectories.buildPartner(FieldConfig.Alliance.BLUE);

        verifyTrajectorySetGeometryAndSymmetry(
                redSolo.allChains(),
                blueSolo.allChains(),
                false);
        verifyTrajectorySetGeometryAndSymmetry(
                redPartner.allChains(),
                bluePartner.allChains(),
                true);
    }

    private static void verifyTrajectorySetGeometryAndSymmetry(
            KodiPathChain[] redChains,
            KodiPathChain[] blueChains,
            boolean enforcePartnerYCorridor) {
        assertEquals(redChains.length, blueChains.length);
        Vec2d redPt = new Vec2d();
        Vec2d bluePt = new Vec2d();

        for (int chainIdx = ZERO_INT; chainIdx < redChains.length; chainIdx++) {
            KodiPathChain redChain = redChains[chainIdx];
            KodiPathChain blueChain = blueChains[chainIdx];
            assertEquals(redChain.size(), blueChain.size());
            assertEquals(redChain.totalLength(), blueChain.totalLength(), SYMMETRY_EPS);

            Pose2d redStart = redChain.getStartPose();
            Pose2d blueStart = blueChain.getStartPose();
            assertEquals(FieldConfig.FIELD_SIZE_METERS, redStart.x + blueStart.x, SYMMETRY_EPS);
            assertEquals(FieldConfig.FIELD_SIZE_METERS, redStart.y + blueStart.y, SYMMETRY_EPS);

            for (int segIdx = ZERO_INT; segIdx < redChain.size(); segIdx++) {
                KodiPathSegment redSeg = redChain.getSegment(segIdx);
                KodiPathSegment blueSeg = blueChain.getSegment(segIdx);

                for (int sampleIdx = ZERO_INT; sampleIdx < TRAJECTORY_SAMPLE_COUNT; sampleIdx++) {
                    double t = (double) sampleIdx / (double) TRAJECTORY_SAMPLE_DIVISOR;
                    redSeg.curve.evalInto(t, redPt);
                    blueSeg.curve.evalInto(t, bluePt);

                    assertTrue(FieldConfig.isInsideFieldBounds(redPt.x, redPt.y));
                    assertTrue(FieldConfig.isInsideFieldBounds(bluePt.x, bluePt.y));
                    assertTrue(FieldConfig.isOutsideHiveKeepout(redPt.x, redPt.y));
                    assertTrue(FieldConfig.isOutsideHiveKeepout(bluePt.x, bluePt.y));
                    assertTrue(Double.isFinite(Math.abs(redSeg.curve.curvature(t))));
                    assertTrue(Double.isFinite(Math.abs(blueSeg.curve.curvature(t))));

                    assertEquals(FieldConfig.FIELD_SIZE_METERS, redPt.x + bluePt.x, SYMMETRY_EPS);
                    assertEquals(FieldConfig.FIELD_SIZE_METERS, redPt.y + bluePt.y, SYMMETRY_EPS);

                    if (enforcePartnerYCorridor) {
                        assertTrue(redPt.y >= FieldConfig.PARTNER_RED_MIN_Y_M - SYMMETRY_EPS);
                        assertTrue(bluePt.y <= FieldConfig.PARTNER_BLUE_MAX_Y_M + SYMMETRY_EPS);
                    }
                }
            }
        }
    }

    private static void stepKinematicRobot(BioBuzzRobot robot, double dtSeconds) {
        robot.tick(dtSeconds);
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
