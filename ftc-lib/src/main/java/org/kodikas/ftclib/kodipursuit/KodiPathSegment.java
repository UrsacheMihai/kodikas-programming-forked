package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.command.Command;
import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Single parametric spline segment within a {@link KodiPathChain}, encapsulating a
 * {@link BezierCurve}, pre-integrated {@link ArcLengthTable}, {@link PathConstraints},
 * {@link HeadingController.HeadingMode} goal, terminal velocity {@code endVelocityMPerS},
 * and up to {@link #MAX_TRIGGERS} pre-allocated spatial event triggers.
 *
 * <p>Spatial event triggers ({@link #onProgress(double, Command)} and
 * {@link #onDistanceRemaining(double, Command)}) are evaluated in-place during path following
 * with strictly {@code 0 bytes/tick} heap allocations.</p>
 */
public final class KodiPathSegment {
    /** Maximum number of progress or distance-remaining triggers per segment. */
    public static final int MAX_TRIGGERS = 8;

    /** Parametric Bezier curve geometry for this segment. */
    public final BezierCurve curve;

    /** Pre-allocated Gauss-Legendre arc-length and curvature lookup table. */
    public final ArcLengthTable arcLengthTable;

    /** Kinematic and feedback gain constraints for this segment. */
    public final PathConstraints constraints;

    /** Active heading tracking mode along this segment. */
    public HeadingController.HeadingMode headingMode = HeadingController.HeadingMode.TANGENT;

    /** Segment start heading in field NWU radians (used by {@link HeadingController.HeadingMode#INTERPOLATED}). */
    public double startHeadingRad = 0.0;

    /** Segment target end heading in field NWU radians (used by {@code CONSTANT} and {@code INTERPOLATED}). */
    public double endHeadingRad = 0.0;

    /** Fixed field coordinate to face when {@link #headingMode} is {@link HeadingController.HeadingMode#TARGET_LOCK}. */
    public final Vec2d targetLockPoint = new Vec2d();

    /** Desired translational speed at the end of this segment ({@code > 0} for fly-by, {@code 0} for settle). */
    public double endVelocityMPerS = 0.0;

    private final double[] progressTriggerT = new double[MAX_TRIGGERS];
    private final Command[] progressTriggerCommands = new Command[MAX_TRIGGERS];
    private final boolean[] progressTriggerFired = new boolean[MAX_TRIGGERS];
    private int progressTriggerCount = 0;

    private final double[] distanceTriggerMeters = new double[MAX_TRIGGERS];
    private final Command[] distanceTriggerCommands = new Command[MAX_TRIGGERS];
    private final boolean[] distanceTriggerFired = new boolean[MAX_TRIGGERS];
    private int distanceTriggerCount = 0;

    /**
     * Constructs a path segment from a {@link BezierCurve} using default {@link PathConstraints}.
     *
     * @param curve parametric Bezier spline
     */
    public KodiPathSegment(BezierCurve curve) {
        this(curve, new PathConstraints());
    }

    /**
     * Constructs a path segment from a {@link BezierCurve} and custom {@link PathConstraints}.
     *
     * @param curve       parametric Bezier spline
     * @param constraints kinematic and dynamic constraints (copied into segment)
     */
    public KodiPathSegment(BezierCurve curve, PathConstraints constraints) {
        this.curve = curve;
        this.arcLengthTable = new ArcLengthTable(curve);
        this.constraints = new PathConstraints(constraints);
        Vec2d tan = new Vec2d();
        curve.unitTangentInto(0.0, tan);
        this.startHeadingRad = tan.angleRad();
        curve.unitTangentInto(1.0, tan);
        this.endHeadingRad = tan.angleRad();
    }

    /**
     * Configures this segment to track the path tangent vector {@code T(t^*)}.
     *
     * @return {@code this} for fluent chaining
     */
    public KodiPathSegment withHeadingTangent() {
        this.headingMode = HeadingController.HeadingMode.TANGENT;
        return this;
    }

    /**
     * Configures this segment to hold a constant field-frame heading.
     *
     * @param headingRad constant desired heading in NWU radians
     * @return {@code this} for fluent chaining
     */
    public KodiPathSegment withConstantHeading(double headingRad) {
        this.headingMode = HeadingController.HeadingMode.CONSTANT;
        this.startHeadingRad = MathUtil.wrapAngleRad(headingRad);
        this.endHeadingRad = this.startHeadingRad;
        return this;
    }

    /**
     * Configures this segment to linearly interpolate heading with respect to arc length
     * from {@code startHeadingRad} to {@code endHeadingRad}.
     *
     * @param startHeadingRad heading at {@code t = 0} in NWU radians
     * @param endHeadingRad   heading at {@code t = 1} in NWU radians
     * @return {@code this} for fluent chaining
     */
    public KodiPathSegment withLinearHeading(double startHeadingRad, double endHeadingRad) {
        this.headingMode = HeadingController.HeadingMode.INTERPOLATED;
        this.startHeadingRad = MathUtil.wrapAngleRad(startHeadingRad);
        this.endHeadingRad = MathUtil.wrapAngleRad(endHeadingRad);
        return this;
    }

    /**
     * Configures this segment to continuously face a fixed field coordinate {@code targetPoint}.
     *
     * @param targetPoint field point in NWU meters
     * @return {@code this} for fluent chaining
     */
    public KodiPathSegment withTargetLock(Vec2d targetPoint) {
        this.headingMode = HeadingController.HeadingMode.TARGET_LOCK;
        this.targetLockPoint.copyFrom(targetPoint);
        return this;
    }

    /**
     * Copies {@code customConstraints} into this segment's constraint set.
     *
     * @param customConstraints segment constraints
     * @return {@code this} for fluent chaining
     */
    public KodiPathSegment withConstraints(PathConstraints customConstraints) {
        this.constraints.copyFrom(customConstraints);
        return this;
    }

    /**
     * Sets the desired terminal speed at {@code t = 1.0} in meters/second.
     * A positive value enables fly-by segment transition; {@code 0.0} enforces a full spatial settle.
     *
     * @param endVelMPerS terminal velocity in m/s
     * @return {@code this} for fluent chaining
     */
    public KodiPathSegment withEndVelocity(double endVelMPerS) {
        this.endVelocityMPerS = Math.max(0.0, endVelMPerS);
        return this;
    }

    /**
     * Registers a spatial progress trigger that schedules {@code command} once when projected
     * parameter {@code t^* >= tTrigger}.
     *
     * @param tTrigger curve parameter threshold in {@code [0.0, 1.0]}
     * @param command  pre-allocated command to schedule
     * @return {@code this} for fluent chaining
     */
    public KodiPathSegment onProgress(double tTrigger, Command command) {
        if (command != null && progressTriggerCount < MAX_TRIGGERS) {
            progressTriggerT[progressTriggerCount] = MathUtil.clamp(tTrigger, 0.0, 1.0);
            progressTriggerCommands[progressTriggerCount] = command;
            progressTriggerFired[progressTriggerCount] = false;
            progressTriggerCount++;
        }
        return this;
    }

    /**
     * Registers a spatial distance-remaining trigger that schedules {@code command} once when
     * remaining arc length {@code s_rem <= metersRemaining}.
     *
     * @param metersRemaining remaining arc-length threshold in meters
     * @param command         pre-allocated command to schedule
     * @return {@code this} for fluent chaining
     */
    public KodiPathSegment onDistanceRemaining(double metersRemaining, Command command) {
        if (command != null && distanceTriggerCount < MAX_TRIGGERS) {
            distanceTriggerMeters[distanceTriggerCount] = Math.max(0.0, metersRemaining);
            distanceTriggerCommands[distanceTriggerCount] = command;
            distanceTriggerFired[distanceTriggerCount] = false;
            distanceTriggerCount++;
        }
        return this;
    }

    /**
     * Resets all spatial trigger fired flags so the segment can be re-executed.
     */
    public void resetTriggers() {
        for (int i = 0; i < progressTriggerCount; i++) {
            progressTriggerFired[i] = false;
        }
        for (int i = 0; i < distanceTriggerCount; i++) {
            distanceTriggerFired[i] = false;
        }
    }

    /**
     * Evaluates all registered spatial triggers at current progress {@code tStar} and remaining
     * arc length {@code distanceRemainingMeters}, scheduling due commands on {@code scheduler}
     * with strictly zero heap allocations.
     *
     * @param tStar                   projected parameter {@code t^* in [0, 1]}
     * @param distanceRemainingMeters remaining arc length {@code S - s(t^*)} in meters
     * @param scheduler               command scheduler (may be {@code null} in standalone tests)
     */
    public void evaluateTriggers(
            double tStar,
            double distanceRemainingMeters,
            KodiScheduler scheduler) {
        for (int i = 0; i < progressTriggerCount; i++) {
            if (!progressTriggerFired[i] && tStar >= progressTriggerT[i]) {
                progressTriggerFired[i] = true;
                fireCommand(progressTriggerCommands[i], scheduler);
            }
        }
        for (int i = 0; i < distanceTriggerCount; i++) {
            if (!distanceTriggerFired[i] && distanceRemainingMeters <= distanceTriggerMeters[i]) {
                distanceTriggerFired[i] = true;
                fireCommand(distanceTriggerCommands[i], scheduler);
            }
        }
    }

    private static void fireCommand(Command cmd, KodiScheduler scheduler) {
        if (cmd == null) {
            return;
        }
        if (scheduler != null) {
            scheduler.schedule(cmd);
        } else {
            cmd.init();
            cmd.execute(0.0);
            if (cmd.isFinished()) {
                cmd.end(false);
            }
        }
    }

    /**
     * Copies all triggers from {@code source} into this segment.
     *
     * @param source source segment
     */
    void copyTriggersFrom(KodiPathSegment source) {
        this.progressTriggerCount = source.progressTriggerCount;
        for (int i = 0; i < source.progressTriggerCount; i++) {
            this.progressTriggerT[i] = source.progressTriggerT[i];
            this.progressTriggerCommands[i] = source.progressTriggerCommands[i];
            this.progressTriggerFired[i] = false;
        }
        this.distanceTriggerCount = source.distanceTriggerCount;
        for (int i = 0; i < source.distanceTriggerCount; i++) {
            this.distanceTriggerMeters[i] = source.distanceTriggerMeters[i];
            this.distanceTriggerCommands[i] = source.distanceTriggerCommands[i];
            this.distanceTriggerFired[i] = false;
        }
    }
}
