package org.kodikas.ftclib.math;

/**
 * Mutable 2D planar velocity vector {@code (vx, vy, omega)} in the right-handed NWU frame
 * ({@code vx} forward in m/s, {@code vy} left in m/s, {@code omega} counter-clockwise in rad/s).
 *
 * <p>Supports zero-allocation conversions between field-centric and robot-centric frames.</p>
 */
public final class ChassisSpeeds {
    /** Forward (+X) linear velocity in meters/second. */
    public double vx;

    /** Left (+Y) linear velocity in meters/second. */
    public double vy;

    /** Counter-clockwise angular velocity in radians/second. */
    public double omega;

    /** Constructs a zero velocity struct {@code (0.0, 0.0, 0.0)}. */
    public ChassisSpeeds() {
        this(0.0, 0.0, 0.0);
    }

    /**
     * Constructs a velocity struct initialized to {@code (vx, vy, omega)}.
     *
     * @param vx    forward (+X) velocity in m/s
     * @param vy    left (+Y) velocity in m/s
     * @param omega counter-clockwise angular velocity in rad/s
     */
    public ChassisSpeeds(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
    }

    /**
     * Static factory method for readable velocity creation outside hot loops.
     *
     * @param vx    forward (+X) velocity in m/s
     * @param vy    left (+Y) velocity in m/s
     * @param omega counter-clockwise angular velocity in rad/s
     * @return newly allocated {@link ChassisSpeeds}
     */
    public static ChassisSpeeds of(double vx, double vy, double omega) {
        return new ChassisSpeeds(vx, vy, omega);
    }

    /**
     * Updates velocity components in-place with zero allocations.
     *
     * @param vx    forward (+X) velocity in m/s
     * @param vy    left (+Y) velocity in m/s
     * @param omega counter-clockwise angular velocity in rad/s
     * @return {@code this} for fluent chaining
     */
    public ChassisSpeeds set(double vx, double vy, double omega) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
        return this;
    }

    /**
     * Copies {@code other} into this instance in-place.
     *
     * @param other source velocity struct
     * @return {@code this} for fluent chaining
     */
    public ChassisSpeeds copyFrom(ChassisSpeeds other) {
        this.vx = other.vx;
        this.vy = other.vy;
        this.omega = other.omega;
        return this;
    }

    /**
     * Resets all velocity components to {@code 0.0}.
     *
     * @return {@code this} for fluent chaining
     */
    public ChassisSpeeds clear() {
        this.vx = 0.0;
        this.vy = 0.0;
        this.omega = 0.0;
        return this;
    }

    /**
     * Rotates field-centric velocities {@code (vx, vy)} by {@code -robotHeadingRad} into
     * robot-centric NWU velocities in {@code target}.
     *
     * @param robotHeadingRad robot heading in field NWU radians
     * @param target          pre-allocated destination struct (may alias {@code this})
     */
    public void fieldToRobotInto(double robotHeadingRad, ChassisSpeeds target) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        double rvx = this.vx * cos + this.vy * sin;
        double rvy = -this.vx * sin + this.vy * cos;
        target.vx = rvx;
        target.vy = rvy;
        target.omega = this.omega;
    }

    /**
     * Rotates robot-centric velocities {@code (vx, vy)} by {@code +robotHeadingRad} into
     * field-centric NWU velocities in {@code target}.
     *
     * @param robotHeadingRad robot heading in field NWU radians
     * @param target          pre-allocated destination struct (may alias {@code this})
     */
    public void robotToFieldInto(double robotHeadingRad, ChassisSpeeds target) {
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        double fvx = this.vx * cos - this.vy * sin;
        double fvy = this.vx * sin + this.vy * cos;
        target.vx = fvx;
        target.vy = fvy;
        target.omega = this.omega;
    }

    @Override
    public String toString() {
        return "ChassisSpeeds(" + vx + ", " + vy + ", " + omega + ")";
    }
}
