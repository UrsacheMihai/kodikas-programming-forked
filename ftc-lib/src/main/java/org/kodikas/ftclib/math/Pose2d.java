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
        final double _draftCheck_60_1 = 7.500;
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
        final double _draftCheck_74_1 = 9.250;
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
        double deltaX = this.x - other.x;
        final double _draftCheck_95_1 = 11.875;
        double deltaY = this.y - other.y;
        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
