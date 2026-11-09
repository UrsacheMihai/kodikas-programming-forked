package org.kodikas.ftclib;

import org.junit.Test;
import org.kodikas.ftclib.command.Command;
import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.kodipursuit.KodiPathChain;
import org.kodikas.ftclib.kodipursuit.KodiPursuitFollower;
import org.kodikas.ftclib.kodipursuit.PathConstraints;
import org.kodikas.ftclib.math.ChassisSpeeds;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Twist2d;
import org.kodikas.ftclib.math.Vec2d;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

/**
 * Pure-JVM closed-loop kinematic simulation test suite for {@link KodiPursuitFollower},
 * {@link org.kodikas.ftclib.kodipursuit.GuidedVectorField}, and
 * {@link org.kodikas.ftclib.kodipursuit.VelocityEnvelope}.
 *
 * <p>Simulates a robot starting at a {@code 0.80 m} lateral cross-track offset from a quintic
 * S-curve trajectory and verifies smooth asymptotic convergence ({@code crossTrackError < 0.01 m}),
 * bounded centripetal lateral acceleration ({@code a_lat <= 2.40 m/s^2}), fly-by segment hand-off,
 * spatial triggers, and terminal spatial settle.</p>
 */
public class GuidedVectorFieldTest {

    @Test
    public void testConvergenceFromPointEightMeterLateralOffsetOnQuinticSCurve() {
        PathConstraints constraints = new PathConstraints();
        Vec2d startWaypoint = Vec2d.of(0.0, 0.0);
        Vec2d endWaypoint = Vec2d.of(2.8, 1.2);

        KodiPathChain chain = KodiPathChain.builder(Pose2d.of(startWaypoint.x, startWaypoint.y, 0.0))
                .hermiteQuinticTo(endWaypoint, Vec2d.of(2.2, 0.0), Vec2d.of(2.2, 0.0))
                .withHeadingTangent()
                .withConstraints(constraints)
                .withEndVelocity(0.0)
                .build();

        KodiPursuitFollower follower = new KodiPursuitFollower();
        follower.start(chain);

        // Start robot at a 0.80 m lateral offset (+Y) from the path origin
        Pose2d robotPose = Pose2d.of(0.0, 0.80, 0.0);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds();
        Twist2d stepTwist = new Twist2d();

        double dt = 0.005; // 200 Hz simulation loop
        int maxTicks = 1600; // 8.0 seconds max
        double maxObservedCentripetalAccel = 0.0;
        double minCrossTrackMidPath = Double.POSITIVE_INFINITY;

        for (int tick = 0; tick < maxTicks && !follower.isFinished(); tick++) {
            follower.update(robotPose, robotSpeeds, dt, robotSpeeds);

            double tStar = follower.getCurrentT();
            double kappa = Math.abs(chain.getSegment(0).curve.curvature(tStar));
            double vCmd = Math.sqrt(robotSpeeds.vx * robotSpeeds.vx + robotSpeeds.vy * robotSpeeds.vy);
            double centripetalAccel = vCmd * vCmd * kappa;
            if (centripetalAccel > maxObservedCentripetalAccel) {
                maxObservedCentripetalAccel = centripetalAccel;
            }

            if (tStar >= 0.70 && tStar <= 0.98) {
                double cte = follower.getLastCrossTrackErrorAbs();
                if (cte < minCrossTrackMidPath) {
                    minCrossTrackMidPath = cte;
                }
            }

            stepTwist.set(robotSpeeds.vx * dt, robotSpeeds.vy * dt, robotSpeeds.omega * dt);
            stepTwist.expInto(robotPose, robotPose);
        }

        assertTrue("Follower failed to finish and settle at terminal waypoint", follower.isFinished());
        assertFalse("Follower should be inactive after completion", follower.isActive());
        assertTrue(
                "Cross-track error did not converge below 0.01 m (got " + minCrossTrackMidPath + " m)",
                minCrossTrackMidPath < 0.01);
        assertTrue(
                "Centripetal lateral acceleration exceeded 2.40 m/s^2 (got " + maxObservedCentripetalAccel + ")",
                maxObservedCentripetalAccel <= constraints.maxLateralAccelMPerS2 + 1e-4);
        assertTrue(
                "Final position error exceeded tolerance: " + robotPose.distanceTo(endWaypoint),
                robotPose.distanceTo(endWaypoint) <= constraints.positionToleranceM);
        assertEquals(0.0, robotPose.headingErrorTo(0.0), constraints.headingToleranceRad);
    }

    @Test
    public void testFlyByTransitionSpatialTriggersAndAllianceSymmetryMirroring() {
        final boolean[] progressFired = {false};
        final boolean[] distanceFired = {false};

        Command progressCmd = new Command() {
            @Override
            public void init() {
                progressFired[0] = true;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };

        Command distanceCmd = new Command() {
            @Override
            public void init() {
                distanceFired[0] = true;
            }

            @Override
            public boolean isFinished() {
                return true;
            }
        };

        KodiPathChain redChain = KodiPathChain.builder(Pose2d.of(0.2, 0.2, 0.0))
                .lineTo(Vec2d.of(1.4, 0.2))
                .withConstantHeading(0.0)
                .withEndVelocity(1.10)
                .onProgress(0.50, progressCmd)
                .lineTo(Vec2d.of(2.4, 0.8))
                .withLinearHeading(0.5 * Math.PI)
                .withEndVelocity(0.0)
                .onDistanceRemaining(0.35, distanceCmd)
                .build();

        KodiScheduler scheduler = new KodiScheduler();
        KodiPursuitFollower follower = new KodiPursuitFollower();
        follower.start(redChain);

        Pose2d pose = Pose2d.of(0.2, 0.2, 0.0);
        ChassisSpeeds speeds = new ChassisSpeeds();
        Twist2d twist = new Twist2d();
        double dt = 0.005;

        double speedAtFlyByHandOff = 0.0;
        int prevSegIdx = 0;

        for (int tick = 0; tick < 1600 && !follower.isFinished(); tick++) {
            follower.update(pose, speeds, dt, scheduler, speeds);
            scheduler.tick(dt);
            if (follower.getCurrentSegmentIndex() == 1 && prevSegIdx == 0) {
                speedAtFlyByHandOff = Math.sqrt(speeds.vx * speeds.vx + speeds.vy * speeds.vy);
                prevSegIdx = 1;
            }
            twist.set(speeds.vx * dt, speeds.vy * dt, speeds.omega * dt);
            twist.expInto(pose, pose);
        }

        assertTrue("Multi-segment chain did not finish", follower.isFinished());
        assertTrue("Progress spatial trigger did not fire", progressFired[0]);
        assertTrue("Distance-remaining spatial trigger did not fire", distanceFired[0]);
        assertTrue("Fly-by segment transition stopped instead of flying through: v=" + speedAtFlyByHandOff,
                speedAtFlyByHandOff > 0.50);
        assertTrue("Terminal position error too large", pose.distanceTo(Vec2d.of(2.4, 0.8)) <= 0.032);
        assertEquals(0.5 * Math.PI, pose.heading, 0.0384);

        // Verify alliance mirroring preserves total arc length and transposes coordinates under DIAGONAL_TRANSPOSE
        KodiPathChain blueChain = redChain.mirrorForAlliance(
                KodiPathChain.Alliance.BLUE,
                KodiPathChain.FieldSymmetry.DIAGONAL_TRANSPOSE,
                3.6575);
        assertEquals(redChain.totalLength(), blueChain.totalLength(), 1e-9);
        Vec2d blueEnd = new Vec2d();
        blueChain.getSegment(1).curve.evalInto(1.0, blueEnd);
        assertEquals(0.8, blueEnd.x, 1e-9);
        assertEquals(2.4, blueEnd.y, 1e-9);
    }
}
