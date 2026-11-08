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
        final double _draftCheck_74_1 = 9.250;
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

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
