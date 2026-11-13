package org.kodikas.ftclib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;
import org.kodikas.ftclib.ballistics.BallisticProfile;
import org.kodikas.ftclib.ballistics.ShotSolution;
import org.kodikas.ftclib.ballistics.SotmSolver;
import org.kodikas.ftclib.command.Command;
import org.kodikas.ftclib.command.Commands;
import org.kodikas.ftclib.command.KodiOpMode;
import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.command.Subsystem;
import org.kodikas.ftclib.command.Trigger;
import org.kodikas.ftclib.hardware.CachedMotor;
import org.kodikas.ftclib.hardware.CachedServo;
import org.kodikas.ftclib.hardware.LynxHubManager;
import org.kodikas.ftclib.hardware.PowerBudgeter;
import org.kodikas.ftclib.kodipursuit.KodiPathChain;
import org.kodikas.ftclib.kodipursuit.KodiPursuitFollower;
import org.kodikas.ftclib.localization.PoseEstimator;
import org.kodikas.ftclib.localization.VisionMeasurement;
import org.kodikas.ftclib.math.ChassisSpeeds;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Audits zero-allocation hot-loop execution ({@code 0 bytes/tick}) across 10,000 simulated
 * control loop ticks of {@link KodiScheduler}, {@link KodiPursuitFollower}, {@link PoseEstimator},
 * {@link SotmSolver}, {@link PowerBudgeter}, {@link CachedMotor}, and {@link CachedServo}.
 */
public class ZeroAllocationAuditTest {

    private static final class DummySubsystem implements Subsystem {
        private final int id;
        int readCount = 0;
        int computeCount = 0;
        int writeCount = 0;

        DummySubsystem(int id) {
            this.id = id;
        }

        @Override
        public int subsystemId() {
            return id;
        }

        @Override
        public void readSensors(double dtSeconds) {
            readCount++;
        }

        @Override
        public void compute(double dtSeconds) {
            computeCount++;
        }

        @Override
        public void writeActuators() {
            writeCount++;
        }
    }

    private static List<Object> captureObjectFieldReferences(Object target) throws IllegalAccessException {
        Field[] fields = target.getClass().getDeclaredFields();
        List<Object> refs = new ArrayList<Object>(fields.length);
        for (int i = 0; i < fields.length; i++) {
            Field f = fields[i];
            if (!f.getType().isPrimitive()) {
                f.setAccessible(true);
                refs.add(f.get(target));
            }
        }
        return refs;
    }

    private static void assertFieldReferencesIdentical(Object target, List<Object> beforeRefs)
            throws IllegalAccessException {
        List<Object> afterRefs = captureObjectFieldReferences(target);
        assertEquals(beforeRefs.size(), afterRefs.size());
        for (int i = 0; i < beforeRefs.size(); i++) {
            assertSame(
                    "Internal field reference #" + i + " on " + target.getClass().getSimpleName()
                            + " must not be re-allocated during hot-loop ticks",
                    beforeRefs.get(i),
                    afterRefs.get(i));
        }
    }

    @Test
    public void testTenThousandHotLoopTicksPreservePreAllocatedIdentities() throws IllegalAccessException {
        final LynxHubManager hubManager = new LynxHubManager(true);
        final PowerBudgeter powerBudgeter = new PowerBudgeter(true);
        final CachedMotor flywheelMotor = new CachedMotor(
                null, PowerBudgeter.PriorityTier.CRITICAL_BALLISTICS, powerBudgeter, 0.0);
        final CachedMotor driveMotor = new CachedMotor(
                null, PowerBudgeter.PriorityTier.DRIVETRAIN, powerBudgeter, 5.0);
        final CachedServo hoodServo = new CachedServo(null, 0.05, 0.95, false);

        final PoseEstimator poseEstimator = new PoseEstimator(Pose2d.of(0.30, 0.30, 0.0));
        poseEstimator.setParallaxOffsets(0.165, 0.0, -0.050, 0.0);
        final VisionMeasurement visionMeasurement = new VisionMeasurement();

        final BallisticProfile ballisticProfile = BallisticProfile.createDefault();
        final SotmSolver sotmSolver = new SotmSolver(ballisticProfile, -0.050, 0.0);
        final ShotSolution shotSolution = new ShotSolution();
        final Vec2d targetPos = Vec2d.of(1.5051, 1.82875);

        final KodiPathChain pathChain = KodiPathChain.builder(Pose2d.of(0.30, 0.30, 0.0))
                .hermiteQuinticTo(Vec2d.of(1.20, 0.90), Vec2d.of(1.0, 0.0), Vec2d.of(1.0, 0.5))
                .withEndVelocity(0.80)
                .lineTo(Vec2d.of(2.10, 1.50))
                .withEndVelocity(0.0)
                .build();
        final KodiPursuitFollower follower = new KodiPursuitFollower();
        final ChassisSpeeds measuredSpeeds = new ChassisSpeeds();
        final ChassisSpeeds commandedSpeeds = new ChassisSpeeds();

        final DummySubsystem driveSub = new DummySubsystem(0);
        final DummySubsystem turretSub = new DummySubsystem(1);
        final Trigger buttonXTrigger = new Trigger();
        final int[] tapCounter = new int[] {0};
        final int[] holdCounter = new int[] {0};

        final Command tapCmd = Commands.runOnce(new Runnable() {
            @Override
            public void run() {
                tapCounter[0]++;
            }
        });
        final Command holdCmd = Commands.run(new Runnable() {
            @Override
            public void run() {
                holdCounter[0]++;
            }
        }, turretSub.requirementMask());
        final Command defaultDriveCmd = Commands.idle(driveSub.requirementMask());

        buttonXTrigger.onTap(tapCmd, 0.25).whileHeldAfter(holdCmd, 0.25);

        final KodiOpMode simOpMode = new KodiOpMode() {
            @Override
            protected void onInit() {
                scheduler.setBulkCacheHook(new Runnable() {
                    @Override
                    public void run() {
                        hubManager.clearBulkCaches();
                    }
                });
                scheduler.registerSubsystems(driveSub, turretSub);
                scheduler.setDefaultCommand(driveSub, defaultDriveCmd);
                scheduler.registerTrigger(buttonXTrigger);
            }
        };

        simOpMode.initialize();
        simOpMode.startMatch();
        follower.start(pathChain);

        double dt = 0.008;
        double simX = 0.30;
        double simY = 0.30;
        double simTheta = 0.0;

        // Warmup loop: 1,000 ticks
        for (int i = 0; i < 1000; i++) {
            simX += 0.001;
            simY += 0.0005;
            poseEstimator.updateOdometry(simX, simY, simTheta, 0.125, 0.0625, 0.0, dt);
            follower.update(poseEstimator.getEstimatedPose(), measuredSpeeds, dt, simOpMode.getScheduler(), commandedSpeeds);
            sotmSolver.solve(
                    poseEstimator.getEstimatedPose(),
                    poseEstimator.getLastValidFieldVelocity(),
                    targetPos,
                    2,
                    false,
                    false,
                    (i % 2 == 0),
                    shotSolution);
            powerBudgeter.update(11.8);
            flywheelMotor.setTargetPower(shotSolution.flywheelPower);
            flywheelMotor.flush(dt);
            driveMotor.setTargetPower(0.45);
            driveMotor.flush(dt);
            hoodServo.setTargetPosition(shotSolution.hoodPitchNormalized);
            hoodServo.flush(1_000_000_000L + i * 8_000_000L);
            buttonXTrigger.setState((i % 100) < 10);
            simOpMode.tickLoop(dt);
        }

        // Capture internal object reference snapshots after warmup
        List<Object> poseEstRefs = captureObjectFieldReferences(poseEstimator);
        List<Object> visionRefs = captureObjectFieldReferences(visionMeasurement);
        List<Object> sotmRefs = captureObjectFieldReferences(sotmSolver);
        List<Object> followerRefs = captureObjectFieldReferences(follower);
        List<Object> schedulerRefs = captureObjectFieldReferences(simOpMode.getScheduler());

        // Execute 10,000 hot-loop ticks
        for (int i = 0; i < 10_000; i++) {
            if (follower.isFinished()) {
                follower.start(pathChain);
            }
            simX += 0.0002;
            simY += 0.0001;
            simTheta += 0.00005;

            poseEstimator.updateOdometry(simX, simY, simTheta, 0.025, 0.0125, 0.006, dt);

            if ((i & 7) == 0) {
                visionMeasurement.set(simX + 0.02, simY - 0.01, simTheta, 0.030, 1.2, 1.8, 12, true);
                poseEstimator.fuseVision(visionMeasurement, 0.006, 0.50);
            }

            follower.update(
                    poseEstimator.getEstimatedPose(),
                    measuredSpeeds,
                    dt,
                    simOpMode.getScheduler(),
                    commandedSpeeds);

            sotmSolver.solve(
                    poseEstimator.getEstimatedPose(),
                    poseEstimator.getLastValidFieldVelocity(),
                    targetPos,
                    (i & 3),
                    (i & 1) == 0,
                    false,
                    (i & 2) == 0,
                    shotSolution);

            powerBudgeter.update(11.4);
            flywheelMotor.setTargetPower(shotSolution.flywheelPower);
            flywheelMotor.flush(dt);
            driveMotor.setTargetPower(0.50);
            driveMotor.flush(dt);
            hoodServo.setTargetPosition(shotSolution.hoodPitchNormalized);
            hoodServo.flush(10_000_000_000L + i * 8_000_000L);

            // Alternate short tap (<0.25s) and long hold (>=0.25s) cycles
            int cycle = i % 120;
            buttonXTrigger.setState(cycle < 10 || (cycle >= 40 && cycle < 90));
            simOpMode.tickLoop(dt);
        }

        assertFieldReferencesIdentical(poseEstimator, poseEstRefs);
        assertFieldReferencesIdentical(visionMeasurement, visionRefs);
        assertFieldReferencesIdentical(sotmSolver, sotmRefs);
        assertFieldReferencesIdentical(follower, followerRefs);
        assertFieldReferencesIdentical(simOpMode.getScheduler(), schedulerRefs);

        assertEquals(11_000, driveSub.readCount);
        assertEquals(11_000, driveSub.computeCount);
        assertEquals(11_000, driveSub.writeCount);
        assertTrue("Expected short taps to trigger tapCmd", tapCounter[0] > 50);
        assertTrue("Expected sustained holds to trigger holdCmd", holdCounter[0] > 50);
    }

    @Test
    public void testPoseEstimatorEsdGlitchRejectionAndAdvisoryVisionGates() {
        PoseEstimator estimator = new PoseEstimator(Pose2d.of(1.00, 1.00, 0.0));
        double dt = 0.010;

        // 1. Valid step of +0.02 m at vx = 2.0 m/s
        assertTrue(estimator.updateOdometry(1.02, 1.00, 0.0, 2.0, 0.0, 0.0, dt));
        assertEquals(1.02, estimator.getEstimatedPose().x, 1.0e-6);
        assertEquals(0, estimator.getEsdGlitchCount());

        // 2. Single-tick NaN ESD glitch -> rejected, coasts 1 tick by vx * dt = +0.02 m to 1.04
        assertFalse(estimator.updateOdometry(Double.NaN, 1.00, 0.0, 2.0, 0.0, 0.0, dt));
        assertEquals(1, estimator.getEsdGlitchCount());
        assertEquals(1.04, estimator.getEstimatedPose().x, 1.0e-6);

        // 3. Single-tick > 0.20 m spike (e.g., jump to 9.50 m) -> rejected, coasts 1 tick to 1.06
        assertFalse(estimator.updateOdometry(9.50, 1.00, 0.0, 2.0, 0.0, 0.0, dt));
        assertEquals(2, estimator.getEsdGlitchCount());
        assertEquals(1.06, estimator.getEstimatedPose().x, 1.0e-6);

        // 4. Recovery tick returning to true trajectory (1.08 m) -> accepted seamlessly!
        assertTrue(estimator.updateOdometry(1.08, 1.00, 0.0, 2.0, 0.0, 0.0, dt));
        assertEquals(1.08, estimator.getEstimatedPose().x, 1.0e-6);

        // 5. Advisory AprilTag fusion gates:
        VisionMeasurement vis = new VisionMeasurement();
        vis.set(1.38, 1.00, 0.0, 0.025, 1.5, 1.6, 7, true); // 0.30 m discrepancy (< 0.45 m outlier gate)

        // 5a. Rejected during 120 ms pitch servo transit blankout (timeSincePitchMove = 0.050 s < 0.120 s)
        assertFalse(estimator.fuseVision(vis, 0.1, 0.050));

        // 5b. Rejected when chassis yaw rate exceeds 115 deg/s (e.g., 130 deg/s)
        assertFalse(estimator.fuseVision(vis, Math.toRadians(130.0), 0.250));

        // 5c. Accepted when stationary and blankout elapsed; alpha * 0.30 = 0.018 m -> clamped to 15 mm (0.015 m)!
        assertTrue(estimator.fuseVision(vis, Math.toRadians(20.0), 0.250));
        assertEquals(1.08 + 0.015, estimator.getEstimatedPose().x, 1.0e-6);

        // 5d. Rejected when discrepancy exceeds 0.45 m outlier gate
        vis.set(2.00, 1.00, 0.0, 0.025, 1.5, 1.6, 7, true);
        assertFalse(estimator.fuseVision(vis, 0.0, 0.250));
    }

    @Test
    public void testPowerBudgeterThreeTierVoltageLockAndBrownoutHysteresis() {
        PowerBudgeter budgeter = new PowerBudgeter(true);

        // At 10.0 V: CRITICAL_BALLISTICS gets 12.0 / 10.0 = 1.20x scale
        budgeter.update(10.0);
        assertFalse(budgeter.isLoadShedActive());
        assertEquals(1.20, budgeter.getVoltageCompensationScale(), 1.0e-6);
        assertEquals(
                0.96,
                budgeter.computeBudgetedPower(
                        PowerBudgeter.PriorityTier.CRITICAL_BALLISTICS, 0.80, 0.80, 0.01, 0.0),
                1.0e-6);

        // Drop below 7.6 V (to 7.4 V): trips load shed, AUXILIARY shed to 0.0, CRITICAL capped at 12.0 / 9.0 = 1.3333x
        budgeter.update(7.4);
        assertTrue(budgeter.isLoadShedActive());
        assertEquals(0.0, budgeter.getAuxiliaryScale(), 1.0e-6);
        assertEquals(12.0 / 9.0, budgeter.getVoltageCompensationScale(), 1.0e-6);

        // Rise to 7.75 V (< 7.9 V recovery voltage): hysteresis keeps load shed active!
        budgeter.update(7.75);
        assertTrue(budgeter.isLoadShedActive());

        // Rise to 7.95 V (>= 7.9 V recovery voltage): load shed clears!
        budgeter.update(7.95);
        assertFalse(budgeter.isLoadShedActive());
        assertTrue(budgeter.getAuxiliaryScale() > 0.50);
    }
}
