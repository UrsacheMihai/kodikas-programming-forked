package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.math.ChassisSpeeds;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Zero-Allocation ({@code 0 bytes/tick}) Holistic Spatial Trajectory Follower combining
 * 3-step damped Newton-Raphson projection ({@link ArcLengthTable}), Algebraic Unit-Norm
 * Guided Vector Fields ({@link GuidedVectorField}), Analytical Curvature Dynamic Braking
 * ({@link VelocityEnvelope}), and Feedforward-PD Heading Control ({@link HeadingController}).
 *
 * <p>Distinguishes seamlessly between intermediate fly-by transitions ({@code endVelocityMPerS > 0})
 * and terminal spatial PD settle hold ({@code endVelocityMPerS == 0}) at {@code B(1.0)}.</p>
 */
public final class KodiPursuitFollower {
    private static final double DEFAULT_FLY_BY_RADIUS_M = 0.085;
    private static final double TERMINAL_HOLD_KP = 4.2;

    private final GuidedVectorField gvf = new GuidedVectorField();
    private final VelocityEnvelope velocityEnvelope = new VelocityEnvelope();
    private final HeadingController headingController = new HeadingController();

    private final Vec2d scratchRobotPos = new Vec2d();
    private final Vec2d scratchGvfDir = new Vec2d();
    private final Vec2d scratchTangent = new Vec2d();
    private final Vec2d scratchNormalLeft = new Vec2d();
    private final Vec2d scratchCentripetal = new Vec2d();
    private final Vec2d scratchEndPoint = new Vec2d();
    private final Vec2d scratchFieldVel = new Vec2d();
    private final ChassisSpeeds scratchFieldSpeeds = new ChassisSpeeds();

    private KodiPathChain activeChain = null;
    private int currentSegmentIndex = 0;
    private double currentT = 0.0;
    private double commandedSpeedMPerS = 0.0;
    private double flyByRadiusMeters = DEFAULT_FLY_BY_RADIUS_M;
    private boolean active = false;
    private boolean finished = true;

    /** Constructs a pre-allocated {@link KodiPursuitFollower}. */
    public KodiPursuitFollower() {
    }

    /**
     * Starts following {@code chain} from segment {@code 0} and resets all segment spatial triggers.
     *
     * @param chain pre-built path chain to follow
     */
    public void start(KodiPathChain chain) {
        this.activeChain = chain;
        this.currentSegmentIndex = 0;
        this.currentT = 0.0;
        this.commandedSpeedMPerS = 0.0;
        if (chain == null || chain.size() == 0) {
            this.active = false;
            this.finished = true;
            return;
        }
        for (int i = 0; i < chain.size(); i++) {
            chain.getSegment(i).resetTriggers();
        }
        this.active = true;
        this.finished = false;
    }

    /**
     * Cancels the currently active path chain and marks the follower inactive.
     */
    public void cancel() {
        this.active = false;
        this.finished = true;
        this.commandedSpeedMPerS = 0.0;
    }

    /**
     * Configures the lookahead hand-off radius in meters for intermediate fly-by segments
     * ({@code endVelocityMPerS > 0}).
     *
     * @param flyByRadiusMeters hand-off distance threshold in meters
     */
    public void setFlyByRadiusMeters(double flyByRadiusMeters) {
        this.flyByRadiusMeters = Math.max(0.01, flyByRadiusMeters);
    }

    /**
     * Executes one control loop update without a {@link KodiScheduler} reference and writes
     * robot-centric NWU {@link ChassisSpeeds} into {@code outRobotSpeeds}.
     *
     * @param currentPose         current estimated robot pose in field NWU coordinates
     * @param measuredRobotSpeeds current measured robot-centric speeds (may be {@code null})
     * @param dtSeconds           loop timestep in seconds
     * @param outRobotSpeeds      pre-allocated destination struct for robot-centric velocity commands
     */
    public void update(
            Pose2d currentPose,
            ChassisSpeeds measuredRobotSpeeds,
            double dtSeconds,
            ChassisSpeeds outRobotSpeeds) {
        update(currentPose, measuredRobotSpeeds, dtSeconds, null, outRobotSpeeds);
    }

    /**
     * Executes one control loop update with strictly {@code 0 bytes/tick} heap allocations:
     * projects pose onto the active spline, fires spatial event triggers on {@code scheduler},
     * handles fly-by vs. spatial-settle segment transitions, evaluates GVF + curvature braking +
     * heading PD/FF, and writes robot-centric NWU {@code (vx, vy, omega)} into {@code outRobotSpeeds}.
     *
     * @param currentPose         current estimated robot pose in field NWU coordinates
     * @param measuredRobotSpeeds current measured robot-centric speeds (may be {@code null})
     * @param dtSeconds           loop timestep in seconds
     * @param scheduler           command scheduler for spatial triggers (may be {@code null})
     * @param outRobotSpeeds      pre-allocated destination struct for robot-centric velocity commands
     */
    public void update(
            Pose2d currentPose,
            ChassisSpeeds measuredRobotSpeeds,
            double dtSeconds,
            KodiScheduler scheduler,
            ChassisSpeeds outRobotSpeeds) {
        if (!active || finished || activeChain == null || currentSegmentIndex >= activeChain.size()) {
            outRobotSpeeds.clear();
            active = false;
            finished = true;
            return;
        }

        currentPose.positionInto(scratchRobotPos);

        KodiPathSegment seg = activeChain.getSegment(currentSegmentIndex);
        BezierCurve curve = seg.curve;
        ArcLengthTable table = seg.arcLengthTable;
        PathConstraints constraints = seg.constraints;

        currentT = table.project(curve, scratchRobotPos, currentT);
        double sCurr = table.sAtT(currentT);
        double sTotal = table.totalLength();
        double sRem = Math.max(0.0, sTotal - sCurr);

        seg.evaluateTriggers(currentT, sRem, scheduler);

        boolean isFlyBy = seg.endVelocityMPerS > MathUtil.EPSILON;
        if (isFlyBy && (currentT >= 0.99 || sRem <= flyByRadiusMeters)) {
            seg.evaluateTriggers(1.0, 0.0, scheduler);
            if (currentSegmentIndex + 1 < activeChain.size()) {
                currentSegmentIndex++;
                seg = activeChain.getSegment(currentSegmentIndex);
                curve = seg.curve;
                table = seg.arcLengthTable;
                constraints = seg.constraints;
                currentT = table.project(curve, scratchRobotPos, 0.0);
                sCurr = table.sAtT(currentT);
                sTotal = table.totalLength();
                sRem = Math.max(0.0, sTotal - sCurr);
                seg.evaluateTriggers(currentT, sRem, scheduler);
                isFlyBy = seg.endVelocityMPerS > MathUtil.EPSILON;
            } else {
                active = false;
                finished = true;
                outRobotSpeeds.clear();
                return;
            }
        }

        gvf.computeInto(curve, currentT, scratchRobotPos, constraints.kGvf, scratchGvfDir);
        gvf.getLastUnitTangentInto(scratchTangent);
        gvf.getLastUnitNormalLeftInto(scratchNormalLeft);

        double targetSpeed = velocityEnvelope.computeTargetSpeed(
                curve, table, currentT, constraints, seg.endVelocityMPerS);

        double dt = Math.max(0.0, dtSeconds);
        if (dt > 0.0) {
            double maxStepUp = constraints.maxAccelMPerS2 * dt;
            double seedSpeed = Math.max(commandedSpeedMPerS, 0.20);
            if (targetSpeed > seedSpeed + maxStepUp) {
                commandedSpeedMPerS = seedSpeed + maxStepUp;
            } else {
                commandedSpeedMPerS = targetSpeed;
            }
        } else {
            commandedSpeedMPerS = targetSpeed;
        }

        curve.evalInto(1.0, scratchEndPoint);
        double dxEnd = scratchEndPoint.x - scratchRobotPos.x;
        double dyEnd = scratchEndPoint.y - scratchRobotPos.y;
        double distToEnd = Math.sqrt(dxEnd * dxEnd + dyEnd * dyEnd);
        double kappa = curve.curvature(currentT);

        boolean inTerminalHold = !isFlyBy && (currentT >= 0.98 || sRem <= 0.12);
        double fvx;
        double fvy;

        if (inTerminalHold) {
            double propSpeed = Math.min(constraints.maxVelMPerS, TERMINAL_HOLD_KP * distToEnd);
            double decelEnvelope = Math.sqrt(2.0 * Math.max(constraints.maxDecelMPerS2, 0.5) * distToEnd);
            double holdSpeed = Math.min(propSpeed, decelEnvelope);
            commandedSpeedMPerS = holdSpeed;
            if (distToEnd > MathUtil.EPSILON) {
                fvx = holdSpeed * (dxEnd / distToEnd);
                fvy = holdSpeed * (dyEnd / distToEnd);
            } else {
                fvx = 0.0;
                fvy = 0.0;
            }
        } else {
            velocityEnvelope.computeCentripetalFeedforwardInto(
                    commandedSpeedMPerS,
                    kappa,
                    scratchNormalLeft,
                    constraints.kCentripetal,
                    scratchCentripetal);
            double rawVx = commandedSpeedMPerS * scratchGvfDir.x + scratchCentripetal.x * dt;
            double rawVy = commandedSpeedMPerS * scratchGvfDir.y + scratchCentripetal.y * dt;
            double rawMag = Math.sqrt(rawVx * rawVx + rawVy * rawVy);
            if (rawMag > commandedSpeedMPerS && rawMag > MathUtil.EPSILON) {
                double scale = commandedSpeedMPerS / rawMag;
                rawVx *= scale;
                rawVy *= scale;
            }
            fvx = rawVx;
            fvy = rawVy;
        }

        scratchFieldVel.set(fvx, fvy);

        double progressFraction = inTerminalHold
                ? 1.0
                : ((sTotal > MathUtil.EPSILON) ? (sCurr / sTotal) : 1.0);
        double measuredOmega = (measuredRobotSpeeds != null) ? measuredRobotSpeeds.omega : 0.0;
        double omegaCmd = headingController.computeOmega(
                seg.headingMode,
                currentPose,
                measuredOmega,
                scratchTangent,
                kappa,
                commandedSpeedMPerS,
                scratchFieldVel,
                progressFraction,
                sTotal,
                seg.startHeadingRad,
                seg.endHeadingRad,
                seg.targetLockPoint,
                constraints);

        if (!isFlyBy) {
            double measuredLinearSpeed = (measuredRobotSpeeds != null)
                    ? Math.sqrt(measuredRobotSpeeds.vx * measuredRobotSpeeds.vx
                            + measuredRobotSpeeds.vy * measuredRobotSpeeds.vy)
                    : commandedSpeedMPerS;
            double headingErrorAbs = (seg.headingMode == HeadingController.HeadingMode.CONSTANT
                    || seg.headingMode == HeadingController.HeadingMode.INTERPOLATED)
                    ? Math.abs(MathUtil.angleDiffRad(seg.endHeadingRad, currentPose.heading))
                    : Math.abs(headingController.getLastHeadingErrorRad());
            if (distToEnd <= constraints.positionToleranceM
                    && headingErrorAbs <= constraints.headingToleranceRad
                    && (measuredLinearSpeed <= constraints.settleVelocityMPerS
                            || commandedSpeedMPerS <= constraints.settleVelocityMPerS)) {
                seg.evaluateTriggers(1.0, 0.0, scheduler);
                if (currentSegmentIndex + 1 < activeChain.size()) {
                    currentSegmentIndex++;
                    currentT = 0.0;
                    commandedSpeedMPerS = 0.0;
                } else {
                    active = false;
                    finished = true;
                    outRobotSpeeds.clear();
                    return;
                }
            }
        }

        scratchFieldSpeeds.set(fvx, fvy, omegaCmd);
        scratchFieldSpeeds.fieldToRobotInto(currentPose.heading, outRobotSpeeds);
    }

    /**
     * Returns {@code true} while the follower is actively tracking a path chain.
     *
     * @return {@code true} if active
     */
    public boolean isActive() {
        return active && !finished;
    }

    /**
     * Returns {@code true} once the active path chain has completed all segments and settled.
     *
     * @return {@code true} if finished
     */
    public boolean isFinished() {
        return finished;
    }

    /**
     * Returns the index of the currently tracked {@link KodiPathSegment}.
     *
     * @return current segment index
     */
    public int getCurrentSegmentIndex() {
        return currentSegmentIndex;
    }

    /**
     * Returns the most recently projected spline parameter {@code t^* in [0, 1]}.
     *
     * @return projected curve parameter
     */
    public double getCurrentT() {
        return currentT;
    }

    /**
     * Returns the most recently commanded translational speed in meters/second.
     *
     * @return commanded speed in m/s
     */
    public double getCommandedSpeedMPerS() {
        return commandedSpeedMPerS;
    }

    /**
     * Returns the signed normal cross-track error {@code e_n} from the most recent tick in meters.
     *
     * @return signed cross-track error in meters
     */
    public double getLastSignedCrossTrackError() {
        return gvf.getLastSignedCrossTrackError();
    }

    /**
     * Returns the unsigned Euclidean cross-track distance {@code ||p - B(t^*)||} from the most
     * recent tick in meters.
     *
     * @return unsigned cross-track error in meters
     */
    public double getLastCrossTrackErrorAbs() {
        return gvf.getLastCrossTrackErrorAbs();
    }

    /**
     * Returns the signed heading error in radians from the most recent tick.
     *
     * @return signed heading error in radians
     */
    public double getLastHeadingErrorRad() {
        return headingController.getLastHeadingErrorRad();
    }
}
