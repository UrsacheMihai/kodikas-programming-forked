package org.kodikas.ftclib.math;

/**
 * Mutable element of the Lie algebra {@code se(2)} representing a body-frame twist
 * {@code (dx, dy, dtheta)} over a finite time step in the right-handed NWU frame.
 *
 * <p>Implements the exact {@code SE(2)} exponential map integration with a 3rd-order
 * Taylor series expansion for small angular increments {@code |dtheta| < 1e-5} to avoid
 * catastrophic cancellation near zero curvature.</p>
 */
public final class Twist2d {
    /** Threshold below which Taylor series approximations replace trigonometric ratios. */
    private static final double SMALL_ANGLE_THRESHOLD = 1e-5;

    /** Body-frame forward (+X) arc displacement in meters. */
    public double dx;

    /** Body-frame left (+Y) arc displacement in meters. */
    public double dy;

    /** Counter-clockwise angular displacement in radians. */
    public double dtheta;

    /** Constructs a zero twist {@code (0.0, 0.0, 0.0)}. */
    public Twist2d() {
        this(0.0, 0.0, 0.0);
    }

    /**
     * Constructs a twist initialized to {@code (dx, dy, dtheta)}.
     *
     * @param dx     body-frame forward displacement in meters
     * @param dy     body-frame left displacement in meters
     * @param dtheta angular displacement in radians
     */
    public Twist2d(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
    }

    /**
     * Static factory method for readable twist creation outside hot loops.
     *
     * @param dx     body-frame forward displacement in meters
     * @param dy     body-frame left displacement in meters
     * @param dtheta angular displacement in radians
     * @return newly allocated {@link Twist2d}
     */
    public static Twist2d of(double dx, double dy, double dtheta) {
        return new Twist2d(dx, dy, dtheta);
    }

    /**
     * Updates this twist in-place with zero allocations.
     *
     * @param dx     body-frame forward displacement in meters
     * @param dy     body-frame left displacement in meters
     * @param dtheta angular displacement in radians
     * @return {@code this} for fluent chaining
     */
    public Twist2d set(double dx, double dy, double dtheta) {
        this.dx = dx;
        this.dy = dy;
        this.dtheta = dtheta;
        return this;
    }

    /**
     * Copies {@code other} into this twist in-place.
     *
     * @param other source twist
     * @return {@code this} for fluent chaining
     */
    public Twist2d copyFrom(Twist2d other) {
        this.dx = other.dx;
        this.dy = other.dy;
        this.dtheta = other.dtheta;
        return this;
    }

    /**
     * Applies the exact {@code SE(2)} exponential map {@code targetPose = startPose * exp(this)}
     * in-place with zero allocations.
     *
     * <p>For {@code |dtheta| < 1e-5}, evaluates the Taylor expansions:
     * {@code s = sin(dtheta)/dtheta ≈ 1 - dtheta^2 / 6} and
     * {@code c = (1 - cos(dtheta))/dtheta ≈ dtheta / 2 - dtheta^3 / 24}.</p>
     *
     * @param startPose  initial pose in field NWU coordinates
     * @param targetPose destination pose in field NWU coordinates (may alias {@code startPose})
     */
    public void expInto(Pose2d startPose, Pose2d targetPose) {
        double dth = this.dtheta;
        double s;
        double c;
        if (Math.abs(dth) < SMALL_ANGLE_THRESHOLD) {
            double dthSq = dth * dth;
            s = 1.0 - dthSq / 6.0;
            c = 0.5 * dth - (dth * dthSq) / 24.0;
        } else {
            s = Math.sin(dth) / dth;
            c = (1.0 - Math.cos(dth)) / dth;
        }

        double localDx = s * this.dx - c * this.dy;
        double localDy = c * this.dx + s * this.dy;

        double startHeading = startPose.heading;
        double cosH = Math.cos(startHeading);
        double sinH = Math.sin(startHeading);

        double newX = startPose.x + localDx * cosH - localDy * sinH;
        double newY = startPose.y + localDx * sinH + localDy * cosH;
        double newHeading = MathUtil.wrapAngleRad(startHeading + dth);

        targetPose.x = newX;
        targetPose.y = newY;
        targetPose.heading = newHeading;
    }

    @Override
    public String toString() {
        return "Twist2d(" + dx + ", " + dy + ", " + dtheta + ")";
    }
}
