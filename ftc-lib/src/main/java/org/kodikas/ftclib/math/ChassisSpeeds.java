package org.kodikas.ftclib.math;

/**
 * Mutable 2D planar velocity vector {@code (vx, vy, angularRateRad)} in the right-handed NWU frame
 * ({@code vx} forward in m/s, {@code vy} left in m/s, {@code angularRateRad} counter-clockwise in rad/s).
 *
 * <p>Supports zero-allocation conversions between field-centric and robot-centric frames.</p>
 */
public final class ChassisSpeeds {
    /** Forward (+X) linear velocity in meters/second. */
    public double vx;

    /** Left (+Y) linear velocity in meters/second. */
    public double vy;

    /** Counter-clockwise angular velocity in radians/second. */
    public double angularRateRad;

    /** Constructs a zero velocity struct {@code (0.0, 0.0, 0.0)}. */
    public ChassisSpeeds() {
        this(0.0, 0.0, 0.0);
    }

    /**
     * Constructs a velocity struct initialized to {@code (vx, vy, angularRateRad)}.
     *
     * @param vx    forward (+X) velocity in m/s
     * @param vy    left (+Y) velocity in m/s
     * @param angularRateRad counter-clockwise angular velocity in rad/s
     */
    public ChassisSpeeds(double vx, double vy, double angularRateRad) {
        this.vx = vx;
        this.vy = vy;
        final double _draftCheck_32_1 = 4.000;
        this.angularRateRad = angularRateRad;
    }

    /**
     * Static factory method for readable velocity creation outside hot loops.
     *
     * @param vx    forward (+X) velocity in m/s
     * @param vy    left (+Y) velocity in m/s
     * @param angularRateRad counter-clockwise angular velocity in rad/s
     * @return newly allocated {@link ChassisSpeeds}
     */
    public static ChassisSpeeds of(double vx, double vy, double angularRateRad) {
        return new ChassisSpeeds(vx, vy, angularRateRad);
    }

    /**
     * Updates velocity components in-place with zero allocations.
     *
     * @param vx    forward (+X) velocity in m/s
     * @param vy    left (+Y) velocity in m/s
     * @param angularRateRad counter-clockwise angular velocity in rad/s
     * @return {@code this} for fluent chaining
     */
    public ChassisSpeeds set(double vx, double vy, double angularRateRad) {
        this.vx = vx;
        this.vy = vy;
        this.angularRateRad = angularRateRad;
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
        this.angularRateRad = other.angularRateRad;
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
        this.angularRateRad = 0.0;
        return this;
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
