package org.kodikas.ftclib.kodipursuit;

import org.kodikas.ftclib.command.Command;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Pre-allocated sequence of {@link KodiPathSegment} splines with a fluent Builder DSL
 * supporting {@code lineTo}, {@code cubicTo}, {@code quinticTo}, {@code hermiteQuinticTo},
 * inline heading/constraint/trigger modifiers, and alliance symmetry mirroring.
 *
 * <p>All arc-length lookup tables and segment constraints are precomputed at build time so
 * runtime trajectory execution in {@link KodiPursuitFollower} performs zero allocations.</p>
 */
public final class KodiPathChain {
    /** Maximum number of spline segments in a single path chain. */
    public static final int MAX_SEGMENTS = 16;

    /** Competition alliance color for trajectory mirroring. */
    public enum Alliance {
        /** Canonical reference alliance (unmirrored). */
        RED,
        /** Mirrored alliance (transformed according to {@link FieldSymmetry}). */
        BLUE
    }

    /** Field geometric symmetry transformation between {@link Alliance#RED} and {@link Alliance#BLUE}. */
    public enum FieldSymmetry {
        /** Diagonal transpose across {@code y = x}: {@code (x, y, theta) -> (y, x, pi/2 - theta)}. */
        DIAGONAL_TRANSPOSE,
        /** Lateral reflection across the longitudinal axis: {@code (x, y, theta) -> (x, L - y, -theta)}. */
        LATERAL_X_MIRROR,
        /** 180-degree rotational symmetry around the field center: {@code (x, y, theta) -> (L - x, L - y, theta + pi)}. */
        ROTATIONAL_180
    }

    private final Pose2d startPose = new Pose2d();
    private final KodiPathSegment[] segments;
    private final int segmentCount;
    private final double totalArcLength;

    private KodiPathChain(Pose2d startPose, KodiPathSegment[] sourceSegments, int count) {
        this.startPose.copyFrom(startPose);
        this.segmentCount = count;
        this.segments = new KodiPathSegment[count];
        double sumLen = 0.0;
        for (int i = 0; i < count; i++) {
            this.segments[i] = sourceSegments[i];
            sumLen += sourceSegments[i].arcLengthTable.totalLength();
        }
        this.totalArcLength = sumLen;
    }

    /**
     * Begins building a new {@link KodiPathChain} starting from {@code startPose}.
     *
     * @param startPose initial robot pose {@code (x, y, heading)} in field NWU coordinates
     * @return fluent {@link Builder}
     */
    public static Builder builder(Pose2d startPose) {
        return new Builder(startPose);
    }

    /**
     * Returns the number of spline segments in this chain.
     *
     * @return segment count
     */
    public int size() {
        return segmentCount;
    }

    /**
     * Returns the spline segment at {@code index}.
     *
     * @param index segment index in {@code [0, size() - 1]}
     * @return {@link KodiPathSegment}
     */
    public KodiPathSegment getSegment(int index) {
        return segments[index];
    }

    /**
     * Returns the initial pose of this path chain.
     *
     * @return start pose
     */
    public Pose2d getStartPose() {
        return startPose;
    }

    /**
     * Copies the initial pose of this path chain into {@code target}.
     *
     * @param target pre-allocated destination pose
     */
    public void getStartPoseInto(Pose2d target) {
        target.copyFrom(startPose);
    }

    /**
     * Returns the total integrated arc length across all segments in meters.
     *
     * @return total chain length in meters
     */
    public double totalLength() {
        return totalArcLength;
    }

    /**
     * Returns a transformed copy of this path chain mirrored for {@code alliance} under
     * {@code symmetry} and {@code fieldSizeMeters}. If {@code alliance == Alliance.RED},
     * returns {@code this} unmodified.
     *
     * @param alliance        target alliance
     * @param symmetry        field symmetry law
     * @param fieldSizeMeters field span in meters (e.g., {@code 3.6575}; or {@code 0.0} for center-origin frames)
     * @return mirrored {@link KodiPathChain}
     */
    public KodiPathChain mirrorForAlliance(
            Alliance alliance,
            FieldSymmetry symmetry,
            double fieldSizeMeters) {
        if (alliance == Alliance.RED || symmetry == null) {
            return this;
        }
        double axisSum = (startPose.x < -1e-3 || startPose.y < -1e-3 || fieldSizeMeters <= 0.0)
                ? 0.0
                : fieldSizeMeters;

        Pose2d mirroredStart = new Pose2d();
        mirrorPoseInto(startPose, symmetry, axisSum, mirroredStart);

        KodiPathSegment[] mirroredSegments = new KodiPathSegment[segmentCount];
        for (int i = 0; i < segmentCount; i++) {
            mirroredSegments[i] = mirrorSegment(segments[i], symmetry, axisSum);
        }
        return new KodiPathChain(mirroredStart, mirroredSegments, segmentCount);
    }

    private static KodiPathSegment mirrorSegment(
            KodiPathSegment seg,
            FieldSymmetry symmetry,
            double axisSum) {
        BezierCurve mirroredCurve;
        if (seg.curve instanceof CubicBezier) {
            CubicBezier c = (CubicBezier) seg.curve;
            Vec2d q0 = new Vec2d(), q1 = new Vec2d(), q2 = new Vec2d(), q3 = new Vec2d();
            mirrorPointInto(c.p0, symmetry, axisSum, q0);
            mirrorPointInto(c.p1, symmetry, axisSum, q1);
            mirrorPointInto(c.p2, symmetry, axisSum, q2);
            mirrorPointInto(c.p3, symmetry, axisSum, q3);
            mirroredCurve = new CubicBezier(q0, q1, q2, q3);
        } else if (seg.curve instanceof QuinticBezier) {
            QuinticBezier q = (QuinticBezier) seg.curve;
            Vec2d q0 = new Vec2d(), q1 = new Vec2d(), q2 = new Vec2d();
            Vec2d q3 = new Vec2d(), q4 = new Vec2d(), q5 = new Vec2d();
            mirrorPointInto(q.p0, symmetry, axisSum, q0);
            mirrorPointInto(q.p1, symmetry, axisSum, q1);
            mirrorPointInto(q.p2, symmetry, axisSum, q2);
            mirrorPointInto(q.p3, symmetry, axisSum, q3);
            mirrorPointInto(q.p4, symmetry, axisSum, q4);
            mirrorPointInto(q.p5, symmetry, axisSum, q5);
            mirroredCurve = new QuinticBezier(q0, q1, q2, q3, q4, q5);
        } else {
            mirroredCurve = seg.curve;
        }

        KodiPathSegment out = new KodiPathSegment(mirroredCurve, seg.constraints);
        out.headingMode = seg.headingMode;
        out.startHeadingRad = mirrorAngle(seg.startHeadingRad, symmetry);
        out.endHeadingRad = mirrorAngle(seg.endHeadingRad, symmetry);
        mirrorPointInto(seg.targetLockPoint, symmetry, axisSum, out.targetLockPoint);
        out.endVelocityMPerS = seg.endVelocityMPerS;
        out.copyTriggersFrom(seg);
        return out;
    }

    private static void mirrorPointInto(
            Vec2d src,
            FieldSymmetry symmetry,
            double axisSum,
            Vec2d target) {
        double sx = src.x;
        double sy = src.y;
        if (symmetry == FieldSymmetry.DIAGONAL_TRANSPOSE) {
            target.x = sy;
            target.y = sx;
        } else if (symmetry == FieldSymmetry.LATERAL_X_MIRROR) {
            target.x = sx;
            target.y = axisSum - sy;
        } else {
            target.x = axisSum - sx;
            target.y = axisSum - sy;
        }
    }

    private static double mirrorAngle(double angleRad, FieldSymmetry symmetry) {
        if (symmetry == FieldSymmetry.DIAGONAL_TRANSPOSE) {
            return MathUtil.wrapAngleRad(0.5 * Math.PI - angleRad);
        } else if (symmetry == FieldSymmetry.LATERAL_X_MIRROR) {
            return MathUtil.wrapAngleRad(-angleRad);
        } else {
            return MathUtil.wrapAngleRad(angleRad + Math.PI);
        }
    }

    private static void mirrorPoseInto(
            Pose2d src,
            FieldSymmetry symmetry,
            double axisSum,
            Pose2d target) {
        Vec2d pt = new Vec2d(src.x, src.y);
        mirrorPointInto(pt, symmetry, axisSum, pt);
        target.x = pt.x;
        target.y = pt.y;
        target.heading = mirrorAngle(src.heading, symmetry);
    }

    /** Fluent Builder DSL for constructing {@link KodiPathChain} instances. */
    public static final class Builder {
        private final Pose2d startPose = new Pose2d();
        private final Vec2d currentEndpoint = new Vec2d();
        private double currentHeadingRad;
        private PathConstraints defaultConstraints = new PathConstraints();

        private final KodiPathSegment[] segments = new KodiPathSegment[MAX_SEGMENTS];
        private final boolean[] explicitEndVelocity = new boolean[MAX_SEGMENTS];
        private int count = 0;

        private Alliance mirrorAlliance = Alliance.RED;
        private FieldSymmetry mirrorSymmetry = FieldSymmetry.DIAGONAL_TRANSPOSE;
        private double mirrorFieldSizeMeters = 3.6575;

        private Builder(Pose2d startPose) {
            this.startPose.copyFrom(startPose);
            this.currentEndpoint.set(startPose.x, startPose.y);
            this.currentHeadingRad = startPose.heading;
        }

        /**
         * Appends a collinear cubic straight-line segment from the current endpoint to {@code endPoint}.
         *
         * @param endPoint destination point in field NWU meters
         * @return {@code this} for fluent chaining
         */
        public Builder lineTo(Vec2d endPoint) {
            CubicBezier line = CubicBezier.fromLine(currentEndpoint, endPoint);
            appendSegment(line, endPoint);
            return this;
        }

        /**
         * Appends a cubic Bezier segment starting from the current endpoint with control points
         * {@code p1, p2} and endpoint {@code p3}.
         *
         * @param p1 first interior control point
         * @param p2 second interior control point
         * @param p3 segment endpoint
         * @return {@code this} for fluent chaining
         */
        public Builder cubicTo(Vec2d p1, Vec2d p2, Vec2d p3) {
            CubicBezier cubic = new CubicBezier(currentEndpoint, p1, p2, p3);
            appendSegment(cubic, p3);
            return this;
        }

        /**
         * Appends a quintic Bezier segment starting from the current endpoint with control points
         * {@code p1, p2, p3, p4} and endpoint {@code p5}.
         *
         * @param p1 control point 1
         * @param p2 control point 2
         * @param p3 control point 3
         * @param p4 control point 4
         * @param p5 segment endpoint
         * @return {@code this} for fluent chaining
         */
        public Builder quinticTo(Vec2d p1, Vec2d p2, Vec2d p3, Vec2d p4, Vec2d p5) {
            QuinticBezier quintic = new QuinticBezier(currentEndpoint, p1, p2, p3, p4, p5);
            appendSegment(quintic, p5);
            return this;
        }

        /**
         * Appends an automatic {@code C^2}-continuous Hermite quintic Bezier segment from the
         * current endpoint to {@code endPoint} with specified start and end tangent vectors.
         *
         * @param endPoint     destination waypoint {@code B(1)}
         * @param startTangent initial derivative vector {@code B'(0)}
         * @param endTangent   terminal derivative vector {@code B'(1)}
         * @return {@code this} for fluent chaining
         */
        public Builder hermiteQuinticTo(Vec2d endPoint, Vec2d startTangent, Vec2d endTangent) {
            QuinticBezier quintic = QuinticBezier.fromWaypointsAutoC2(
                    currentEndpoint, startTangent, endPoint, endTangent);
            appendSegment(quintic, endPoint);
            return this;
        }

        /**
         * Appends a full 2nd-order Hermite quintic Bezier segment from the current endpoint to
         * {@code endPoint} with explicit first and second boundary derivatives.
         *
         * @param endPoint   destination waypoint {@code B(1)}
         * @param startVel   initial first derivative {@code B'(0)}
         * @param startAccel initial second derivative {@code B''(0)}
         * @param endVel     terminal first derivative {@code B'(1)}
         * @param endAccel   terminal second derivative {@code B''(1)}
         * @return {@code this} for fluent chaining
         */
        public Builder hermiteQuinticTo(
                Vec2d endPoint,
                Vec2d startVel,
                Vec2d startAccel,
                Vec2d endVel,
                Vec2d endAccel) {
            QuinticBezier quintic = QuinticBezier.fromHermite(
                    currentEndpoint, startVel, startAccel, endPoint, endVel, endAccel);
            appendSegment(quintic, endPoint);
            return this;
        }

        private void appendSegment(BezierCurve curve, Vec2d newEndpoint) {
            if (count >= MAX_SEGMENTS) {
                return;
            }
            KodiPathSegment seg = new KodiPathSegment(curve, defaultConstraints);
            seg.startHeadingRad = currentHeadingRad;
            Vec2d endTan = new Vec2d();
            curve.unitTangentInto(1.0, endTan);
            seg.endHeadingRad = endTan.angleRad();
            currentHeadingRad = seg.endHeadingRad;
            currentEndpoint.copyFrom(newEndpoint);
            segments[count] = seg;
            explicitEndVelocity[count] = false;
            count++;
        }

        /**
         * Configures the most recently added segment to follow the path tangent {@code T(t^*)}.
         *
         * @return {@code this} for fluent chaining
         */
        public Builder withHeadingTangent() {
            if (count > 0) {
                KodiPathSegment seg = segments[count - 1];
                seg.withHeadingTangent();
                Vec2d endTan = new Vec2d();
                seg.curve.unitTangentInto(1.0, endTan);
                currentHeadingRad = endTan.angleRad();
            }
            return this;
        }

        /**
         * Configures the most recently added segment to hold a constant field heading.
         *
         * @param headingRad constant desired heading in NWU radians
         * @return {@code this} for fluent chaining
         */
        public Builder withConstantHeading(double headingRad) {
            if (count > 0) {
                segments[count - 1].withConstantHeading(headingRad);
                currentHeadingRad = MathUtil.wrapAngleRad(headingRad);
            }
            return this;
        }

        /**
         * Configures the most recently added segment to linearly interpolate heading from the
         * previous segment's end heading to {@code endHeadingRad}.
         *
         * @param endHeadingRad target heading at the end of the segment in NWU radians
         * @return {@code this} for fluent chaining
         */
        public Builder withLinearHeading(double endHeadingRad) {
            if (count > 0) {
                KodiPathSegment seg = segments[count - 1];
                double startH = (count == 1) ? startPose.heading : segments[count - 2].endHeadingRad;
                seg.withLinearHeading(startH, endHeadingRad);
                currentHeadingRad = MathUtil.wrapAngleRad(endHeadingRad);
            }
            return this;
        }

        /**
         * Configures the most recently added segment to linearly interpolate heading from
         * {@code startHeadingRad} to {@code endHeadingRad}.
         *
         * @param startHeadingRad start heading in NWU radians
         * @param endHeadingRad   end heading in NWU radians
         * @return {@code this} for fluent chaining
         */
        public Builder withLinearHeading(double startHeadingRad, double endHeadingRad) {
            if (count > 0) {
                segments[count - 1].withLinearHeading(startHeadingRad, endHeadingRad);
                currentHeadingRad = MathUtil.wrapAngleRad(endHeadingRad);
            }
            return this;
        }

        /**
         * Configures the most recently added segment to lock chassis heading onto {@code targetPoint}.
         *
         * @param targetPoint fixed field point in NWU meters
         * @return {@code this} for fluent chaining
         */
        public Builder withTargetLock(Vec2d targetPoint) {
            if (count > 0) {
                segments[count - 1].withTargetLock(targetPoint);
            }
            return this;
        }

        /**
         * Applies {@code constraints} to the most recently added segment (or sets the default
         * constraints if called before any segments are added).
         *
         * @param constraints kinematic and feedback constraints
         * @return {@code this} for fluent chaining
         */
        public Builder withConstraints(PathConstraints constraints) {
            this.defaultConstraints.copyFrom(constraints);
            if (count > 0) {
                segments[count - 1].withConstraints(constraints);
            }
            return this;
        }

        /**
         * Sets the terminal velocity {@code endVelMPerS} on the most recently added segment.
         *
         * @param endVelMPerS end velocity in m/s ({@code > 0} for fly-by, {@code 0} for settle)
         * @return {@code this} for fluent chaining
         */
        public Builder withEndVelocity(double endVelMPerS) {
            if (count > 0) {
                segments[count - 1].withEndVelocity(endVelMPerS);
                explicitEndVelocity[count - 1] = true;
            }
            return this;
        }

        /**
         * Attaches a spatial progress trigger {@code t^* >= t} to the most recently added segment.
         *
         * @param t   curve parameter threshold in {@code [0, 1]}
         * @param cmd pre-allocated command to schedule
         * @return {@code this} for fluent chaining
         */
        public Builder onProgress(double t, Command cmd) {
            if (count > 0) {
                segments[count - 1].onProgress(t, cmd);
            }
            return this;
        }

        /**
         * Attaches a spatial distance-remaining trigger {@code s_rem <= meters} to the most
         * recently added segment.
         *
         * @param meters remaining arc-length threshold in meters
         * @param cmd    pre-allocated command to schedule
         * @return {@code this} for fluent chaining
         */
        public Builder onDistanceRemaining(double meters, Command cmd) {
            if (count > 0) {
                segments[count - 1].onDistanceRemaining(meters, cmd);
            }
            return this;
        }

        /**
         * Configures alliance symmetry mirroring to be applied when {@link #build()} is invoked.
         *
         * @param alliance        target alliance
         * @param symmetry        field symmetry transformation
         * @param fieldSizeMeters field span in meters
         * @return {@code this} for fluent chaining
         */
        public Builder mirrorForAlliance(
                Alliance alliance,
                FieldSymmetry symmetry,
                double fieldSizeMeters) {
            this.mirrorAlliance = alliance;
            this.mirrorSymmetry = symmetry;
            this.mirrorFieldSizeMeters = fieldSizeMeters;
            return this;
        }

        /**
         * Finalizes and builds the {@link KodiPathChain}.
         *
         * @return pre-allocated {@link KodiPathChain}
         */
        public KodiPathChain build() {
            if (count > 0 && !explicitEndVelocity[count - 1]) {
                segments[count - 1].endVelocityMPerS = 0.0;
            }
            KodiPathChain chain = new KodiPathChain(startPose, segments, count);
            if (mirrorAlliance == Alliance.BLUE) {
                return chain.mirrorForAlliance(mirrorAlliance, mirrorSymmetry, mirrorFieldSizeMeters);
            }
            return chain;
        }
    }
}
