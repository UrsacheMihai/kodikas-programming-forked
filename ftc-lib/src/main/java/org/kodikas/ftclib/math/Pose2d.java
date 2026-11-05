package org.kodikas.ftclib.math;

/**
 * Mutable rigid-body 2D pose {@code (x, y, heading)} in the right-handed NWU frame
 * ({@code +X = Forward} in meters, {@code +Y = Left} in meters,
 * {@code +heading = Counter-Clockwise} in radians with {@code 0 rad = +X}).
 *
 * <p>Provides zero-allocation {@code *Into} frame transformations for high-frequency
 * odometry, path following, and ballistic solvers.</p>
 */
public final class Pose2d {
    /** Forward (+X) coordinate in meters. */
    public double x;

    /** Left (+Y) coordinate in meters. */
    public double y;

    /** Counter-clockwise heading in radians ({@code 0.0 = +X}). */
    public double heading;

    /** Constructs an origin pose {@code (0.0, 0.0, 0.0)}. */
    public Pose2d() {
        this(0.0, 0.0, 0.0);
    }

    /**
     * Constructs a pose initialized to {@code (x, y, heading)}.
     *
     * @param x       forward (+X) position in meters
     * @param y       left (+Y) position in meters
     * @param heading counter-clockwise heading in radians
     */
    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Static factory method for readable pose construction outside hot loops.
     *
     * @param x       forward (+X) position in meters
     * @param y       left (+Y) position in meters
     * @param heading counter-clockwise heading in radians
     * @return newly allocated {@link Pose2d}
     */
    public static Pose2d of(double x, double y, double heading) {
        return new Pose2d(x, y, heading);
    }

    /**
     * Updates this pose in-place with zero allocations.
     *
     * @param x       forward (+X) position in meters
     * @param y       left (+Y) position in meters
     * @param heading counter-clockwise heading in radians
     * @return {@code this} for fluent chaining
     */
    public Pose2d set(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        return this;
    }

    /**
     * Copies {@code other} into this pose in-place.
     *
     * @param other source pose
     * @return {@code this} for fluent chaining
     */
    public Pose2d copyFrom(Pose2d other) {
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
        return this;
    }

    /**
     * Writes the translational {@code (x, y)} position of this pose into {@code target}.
     *
     * @param target pre-allocated destination vector
     */
    public void positionInto(Vec2d target) {
        target.x = this.x;
        target.y = this.y;
    }

    /**
     * Computes the Euclidean translational distance between this pose and {@code other}.
     *
     * @param other target pose
     * @return translational distance in meters
     */
    public double distanceTo(Pose2d other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Computes the Euclidean translational distance between this pose and {@code point}.
     *
     * @param point target 2D point
     * @return translational distance in meters
     */
    public double distanceTo(Vec2d point) {
        double dx = this.x - point.x;
        double dy = this.y - point.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Computes the shortest wrapped angular difference {@code wrapAngleRad(targetHeadingRad - this.heading)}
     * in {@code [-pi, pi]}.
     *
     * @param targetHeadingRad desired heading in radians
     * @return shortest signed heading error in radians
     */
    public double headingErrorTo(double targetHeadingRad) {
        return MathUtil.angleDiffRad(targetHeadingRad, this.heading);
    }

    /**
     * Transforms a robot-local offset {@code (lx, ly)} into field coordinates using this pose:
     * {@code [x + lx cos(heading) - ly sin(heading), y + lx sin(heading) + ly cos(heading)]}.
     *
     * @param localOffset offset expressed in robot NWU frame
     * @param target      pre-allocated destination vector in field NWU frame (may alias {@code localOffset})
     */
    public void transformLocalToFieldInto(Vec2d localOffset, Vec2d target) {
        double cos = Math.cos(this.heading);
        double sin = Math.sin(this.heading);
        double fx = this.x + localOffset.x * cos - localOffset.y * sin;
        double fy = this.y + localOffset.x * sin + localOffset.y * cos;
        target.x = fx;
        target.y = fy;
    }

    /**
     * Transforms a field-frame point {@code (fx, fy)} into this pose's robot-local NWU frame:
     * {@code [(fx - x) cos(heading) + (fy - y) sin(heading), -(fx - x) sin(heading) + (fy - y) cos(heading)]}.
     *
     * @param fieldPoint point expressed in field NWU frame
     * @param target     pre-allocated destination vector in robot NWU frame (may alias {@code fieldPoint})
     */
    public void transformFieldToLocalInto(Vec2d fieldPoint, Vec2d target) {
        double dx = fieldPoint.x - this.x;
        double dy = fieldPoint.y - this.y;
        double cos = Math.cos(this.heading);
        double sin = Math.sin(this.heading);
        double lx = dx * cos + dy * sin;
        double ly = -dx * sin + dy * cos;
        target.x = lx;
        target.y = ly;
    }

    @Override
    public String toString() {
        return "Pose2d(" + x + ", " + y + ", " + heading + ")";
    }
}
