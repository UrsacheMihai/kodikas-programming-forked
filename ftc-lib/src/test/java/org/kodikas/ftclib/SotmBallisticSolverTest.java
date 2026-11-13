package org.kodikas.ftclib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;
import org.kodikas.ftclib.ballistics.BallisticProfile;
import org.kodikas.ftclib.ballistics.ShotSolution;
import org.kodikas.ftclib.ballistics.SotmSolver;
import org.kodikas.ftclib.math.ChassisSpeeds;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Unit test suite for {@link BallisticProfile}, {@link ShotSolution}, and {@link SotmSolver}.
 */
public class SotmBallisticSolverTest {

    @Test
    public void testBallisticProfileMonotonicInterpolationAndHoodClamp() {
        double[] distances = new double[] {1.00, 1.80, 2.60, 3.40};
        double[] pitches = new double[] {0.01, 0.38, 0.72, 0.99};
        double[] tofs = new double[] {0.20, 0.35, 0.51, 0.68};
        double[] powers = new double[] {0.75, 0.81, 0.87, 0.93};

        BallisticProfile profile = new BallisticProfile(distances, pitches, tofs, powers);

        // Verify strict +/-5% [0.05, 0.95] software clamp on hood pitch knots and extreme distances
        assertEquals(0.05, profile.evaluateHoodPitch(0.30), 1.0e-9);
        assertEquals(0.05, profile.evaluateHoodPitch(1.00), 1.0e-9);
        assertEquals(0.38, profile.evaluateHoodPitch(1.80), 1.0e-9);
        assertEquals(0.72, profile.evaluateHoodPitch(2.60), 1.0e-9);
        assertEquals(0.95, profile.evaluateHoodPitch(3.40), 1.0e-9);
        assertEquals(0.95, profile.evaluateHoodPitch(5.00), 1.0e-9);

        // Verify strict monotonicity across 200 fine distance samples
        double prevPitch = profile.evaluateHoodPitch(1.00);
        double prevTof = profile.evaluateTimeOfFlight(1.00);
        double prevPower = profile.evaluateScoringPower(1.00);

        for (int i = 1; i <= 200; i++) {
            double d = 1.00 + (2.40 * i) / 200.0;
            double pitch = profile.evaluateHoodPitch(d);
            double tof = profile.evaluateTimeOfFlight(d);
            double power = profile.evaluateScoringPower(d);

            assertTrue("Hood pitch must be monotonic non-decreasing", pitch >= prevPitch - 1.0e-12);
            assertTrue("Hood pitch must stay within [0.05, 0.95]", pitch >= 0.05 && pitch <= 0.95);
            assertTrue("Time of flight must be strictly increasing", tof >= prevTof - 1.0e-12);
            assertTrue("Scoring power must be monotonic non-decreasing", power >= prevPower - 1.0e-12);

            prevPitch = pitch;
            prevTof = tof;
            prevPower = power;
        }
    }

    @Test
    public void testTwoStageAutomaticFlywheelSpeedProfile() {
        BallisticProfile profile = BallisticProfile.createDefault();
        double dist = 2.00;
        double expectedScoringPower = profile.evaluateScoringPower(dist);

        // Stage 1: Warm idle (0.55) when empty (0 pieces) and intake inactive
        assertEquals(
                BallisticProfile.DEFAULT_WARM_IDLE_POWER,
                profile.computeTargetFlywheelPower(dist, 0, false),
                1.0e-9);

        // Stage 2a: Immediate boost to scoring speed as soon as intake becomes active (even with 0 pieces)
        assertEquals(
                expectedScoringPower,
                profile.computeTargetFlywheelPower(dist, 0, true),
                1.0e-9);

        // Stage 2b: Remains at scoring speed whenever gamePieceCount >= 1 (even if intake stops)
        assertEquals(
                expectedScoringPower,
                profile.computeTargetFlywheelPower(dist, 1, false),
                1.0e-9);
        assertEquals(
                expectedScoringPower,
                profile.computeTargetFlywheelPower(dist, 4, false),
                1.0e-9);
    }

    @Test
    public void testFourStepFixedPointContractionConvergenceAndPivotOffset() {
        BallisticProfile profile = BallisticProfile.createDefault();
        double turretOffsetX = -0.055;
        double turretOffsetY = 0.020;
        SotmSolver solver = new SotmSolver(profile, turretOffsetX, turretOffsetY);

        Pose2d robotPose = Pose2d.of(0.65, 1.10, Math.toRadians(28.0));
        ChassisSpeeds fieldVel = ChassisSpeeds.of(1.35, -0.85, 1.40);
        Vec2d targetPos = Vec2d.of(2.45, 1.85);
        ShotSolution solution = new ShotSolution();

        solver.solve(robotPose, fieldVel, targetPos, 2, false, false, true, solution);

        // Compute exact launch point and velocity with turret offset
        double cos = Math.cos(robotPose.heading);
        double sin = Math.sin(robotPose.heading);
        double rx = turretOffsetX * cos - turretOffsetY * sin;
        double ry = turretOffsetX * sin + turretOffsetY * cos;
        double launchX = robotPose.x + rx;
        double launchY = robotPose.y + ry;
        double launchVx = fieldVel.vx - fieldVel.omega * ry;
        double launchVy = fieldVel.vy + fieldVel.omega * rx;

        double convergedDist = Math.hypot(
                solution.virtualTargetX - launchX,
                solution.virtualTargetY - launchY);
        double expectedTof = profile.evaluateTimeOfFlight(convergedDist);

        double expectedVirtX = targetPos.x - launchVx * expectedTof;
        double expectedVirtY = targetPos.y - launchVy * expectedTof;

        double residualMeters = Math.hypot(
                solution.virtualTargetX - expectedVirtX,
                solution.virtualTargetY - expectedVirtY);

        assertTrue(
                "4-step fixed-point contraction residual must be < 0.5 mm (0.0005 m), got " + residualMeters,
                residualMeters < 5.0e-4);
        assertEquals(convergedDist, solution.virtualDistanceMeters, 1.0e-6);
    }

    @Test
    public void testServoPhaseLagLeadCompensation() {
        BallisticProfile profile = BallisticProfile.createDefault();
        SotmSolver solver = new SotmSolver(profile, 0.0, 0.0);
        Pose2d pose = Pose2d.of(0.0, 0.0, 0.0);
        Vec2d target = Vec2d.of(2.0, 0.0);

        // Pure chassis rotation at +2.0 rad/s (CCW) while stationary at origin
        ChassisSpeeds rotatingVel = ChassisSpeeds.of(0.0, 0.0, 2.0);
        ShotSolution solWithLag = new ShotSolution();
        solver.setServoPhaseLagSec(0.035);
        solver.solve(pose, rotatingVel, target, 1, false, false, true, solWithLag);

        // Turret must lead by -omega * tau_servo = -2.0 * 0.035 = -0.070 rad
        assertEquals(-2.0 * 0.035, solWithLag.turretYawRad, 1.0e-6);
    }

    @Test
    public void testTurretHardStopSoftLimitHeadingAssistAndStowLaws() {
        BallisticProfile profile = BallisticProfile.createDefault();
        SotmSolver solver = new SotmSolver(profile, 0.0, 0.0);
        ShotSolution solution = new ShotSolution();

        // 1. Verify 350.0 / (2 * 2.75) = 63.63636363636363 deg physical hard stop & servo PWM mapping
        assertEquals(350.0 / 5.5, solver.getPhysicalHardStopDeg(), 1.0e-9);
        assertEquals(0.50, solver.yawRadToServoNormalized(0.0), 1.0e-9);
        assertEquals(1.00, solver.yawRadToServoNormalized(solver.getPhysicalHardStopRad()), 1.0e-9);
        assertEquals(0.00, solver.yawRadToServoNormalized(-solver.getPhysicalHardStopRad()), 1.0e-9);

        ChassisSpeeds zeroVel = ChassisSpeeds.of(0.0, 0.0, 0.0);
        Vec2d targetAhead = Vec2d.of(2.0, 0.0);

        // 2. Inside +/-55 deg independent cone (e.g., robot heading = -40 deg -> desired turret yaw = +40 deg)
        Pose2d pose40 = Pose2d.of(0.0, 0.0, Math.toRadians(-40.0));
        solver.solve(pose40, zeroVel, targetAhead, 1, false, false, false, solution);
        assertFalse(solution.stowed);
        assertFalse(solution.inSoftStopAssistZone);
        assertEquals(Math.toRadians(40.0), solution.turretYawRad, 1.0e-6);
        assertEquals(0.0, solution.headingAssistOmegaRadPerSec, 1.0e-9);

        // 3. Between 55 deg and 70 deg (e.g., robot heading = -64 deg -> desired turret yaw = +64 deg)
        // Turret clamps at +60.0 deg soft limit while drivetrain receives positive CCW heading assist
        Pose2d pose64 = Pose2d.of(0.0, 0.0, Math.toRadians(-64.0));
        solver.solve(pose64, zeroVel, targetAhead, 1, false, false, false, solution);
        assertFalse(solution.stowed);
        assertTrue(solution.inSoftStopAssistZone);
        assertEquals(Math.toRadians(60.0), solution.turretYawRad, 1.0e-6);
        assertTrue("Heading assist must rotate chassis CCW (+omega)", solution.headingAssistOmegaRadPerSec > 0.1);

        // 4. Rear blind spot (> 70 deg, e.g., desired yaw = +110 deg) while shootRequested == false -> STOWED at 0 deg
        Pose2d pose110 = Pose2d.of(0.0, 0.0, Math.toRadians(-110.0));
        solver.solve(pose110, zeroVel, targetAhead, 1, false, false, false, solution);
        assertTrue(solution.inRearBlindSpot);
        assertTrue(solution.stowed);
        assertEquals(0.0, solution.turretYawRad, 1.0e-9);
        assertEquals(0.50, solution.turretServoNormalized, 1.0e-9);
        assertEquals(0.0, solution.headingAssistOmegaRadPerSec, 1.0e-9);
        assertFalse(solution.isReadyToFire);

        // 5. Rear blind spot (> 70 deg) when shootRequested == true -> auto-rotates chassis into firing cone
        solver.solve(pose110, zeroVel, targetAhead, 1, false, false, true, solution);
        assertFalse(solution.stowed);
        assertTrue(solution.inSoftStopAssistZone);
        assertEquals(Math.toRadians(60.0), solution.turretYawRad, 1.0e-6);
        assertTrue(solution.headingAssistOmegaRadPerSec > 0.5);

        // 6. Front plow deployed while shootRequested == false -> STOWED at 0.0 deg centerline
        solver.solve(pose40, zeroVel, targetAhead, 1, true, true, false, solution);
        assertTrue(solution.stowed);
        assertEquals(0.0, solution.turretYawRad, 1.0e-9);
        assertFalse(solution.isReadyToFire);
    }

    @Test
    public void testSchmittTriggerAimReadinessHysteresis() {
        BallisticProfile profile = BallisticProfile.createDefault();
        SotmSolver solver = new SotmSolver(profile, 0.0, 0.0);
        solver.resetReadinessLatch();

        // 1. Error = 2.2 deg (> 1.8 deg entry threshold) -> NOT ready
        assertFalse(solver.evaluateAimReadiness(Math.toRadians(2.2), false));

        // 2. Error drops to 1.75 deg (<= 1.8 deg entry threshold) -> LATCHES ready
        assertTrue(solver.evaluateAimReadiness(Math.toRadians(1.75), false));

        // 3. Error widens to 3.4 deg (<= 3.5 deg hold threshold) -> HOLDS ready
        assertTrue(solver.evaluateAimReadiness(Math.toRadians(3.40), false));

        // 4. Error exceeds 3.6 deg (> 3.5 deg hold threshold) -> DROPS ready
        assertFalse(solver.evaluateAimReadiness(Math.toRadians(3.60), false));

        // 5. Error returns to 2.5 deg (inside hold, but outside 1.8 deg entry) -> stays NOT ready
        assertFalse(solver.evaluateAimReadiness(Math.toRadians(2.50), false));

        // 6. Error re-enters 1.5 deg (<= 1.8 deg entry) -> LATCHES ready again
        assertTrue(solver.evaluateAimReadiness(Math.toRadians(1.50), false));

        // 7. Stowing immediately clears latch
        assertFalse(solver.evaluateAimReadiness(Math.toRadians(0.20), true));
    }
}
