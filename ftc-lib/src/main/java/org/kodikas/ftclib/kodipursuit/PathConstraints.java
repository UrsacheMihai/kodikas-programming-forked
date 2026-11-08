package org.kodikas.ftclib.kodipursuit;

/**
 * Kinematic, dynamic, and feedback gain constraints for a {@link KodiPathSegment} or
 * {@link KodiPathChain} in SI units (meters, seconds, radians).
 *
 * <p>Pre-allocated with competition-tuned defaults and supports both in-place mutation and
 * copy construction for zero-allocation path following.</p>
 */
public final class PathConstraints {
    /** Default pre-allocated constraint set. */
    public static final PathConstraints DEFAULT = new PathConstraints();

    /** Maximum translational velocity in meters/second ({@code 1.85 m/s}). */
    public double maxVelMPerS = 1.85;

    /** Maximum forward longitudinal acceleration in meters/second^2 ({@code 2.65 m/s^2}). */
    public double maxAccelMPerS2 = 2.65;

    /** Maximum longitudinal braking deceleration in meters/second^2 ({@code 2.85 m/s^2}). */
    public double maxDecelMPerS2 = 2.85;

    /** Maximum allowable centripetal lateral acceleration in meters/second^2 ({@code 2.40 m/s^2}). */
    public double maxLateralAccelMPerS2 = 2.40;

    /** Guided Vector Field (GVF) normal convergence gain {@code k_gvf} in {@code 1/m} ({@code 2.50}). */
    public double kGvf = 2.50;

    /** Centripetal normal slip-compensation feedforward gain ({@code 0.28}). */
    public double kCentripetal = 0.28;

    /** Heading proportional gain {@code kP} in {@code (rad/s) / rad} ({@code 2.05}). */
    public double kPHeading = 2.05;

    /** Heading derivative damping gain {@code kD} in {@code (rad/s) / (rad/s)} ({@code 0.17}). */
    public double kDHeading = 0.17;

    /** Maximum commanded angular velocity magnitude in radians/second ({@code 3.5 rad/s}). */
    public double maxOmegaRadPerSec = 3.5;

    /** Terminal spatial position settle tolerance in meters ({@code 0.032 m = 32 mm}). */
    public double positionToleranceM = 0.032;

    /** Terminal heading settle tolerance in radians ({@code 0.0384 rad ≈ 2.2 deg}). */
    public double headingToleranceRad = 0.0384;

    /** Terminal linear speed threshold below which the robot is considered settled ({@code 0.08 m/s}). */
    public double settleVelocityMPerS = 0.08;

    /** Constructs a constraint struct initialized to competition defaults. */
    public PathConstraints() {
    }

    /**
     * Constructs a constraint struct by copying values from {@code other}.
     *
     * @param other source constraints
     */
    public PathConstraints(PathConstraints other) {
        copyFrom(other);
    }

    /**
     * Constructs a fully specified constraint set.
     *
     * @param maxVelMPerS           maximum translational velocity (m/s)
     * @param maxAccelMPerS2        maximum longitudinal acceleration (m/s^2)
     * @param maxDecelMPerS2        maximum longitudinal deceleration (m/s^2)
     * @param maxLateralAccelMPerS2 maximum centripetal lateral acceleration (m/s^2)
     * @param kGvf                  GVF cross-track convergence gain (1/m)
     * @param kCentripetal          centripetal slip feedforward gain
     * @param kPHeading             heading proportional gain
     * @param kDHeading             heading derivative gain
     * @param maxOmegaRadPerSec     maximum angular velocity (rad/s)
     * @param positionToleranceM    position settle tolerance (m)
     * @param headingToleranceRad   heading settle tolerance (rad)
     * @param settleVelocityMPerS   velocity settle threshold (m/s)
     */
    public PathConstraints(
            double maxVelMPerS,
            double maxAccelMPerS2,
            double maxDecelMPerS2,
            double maxLateralAccelMPerS2,
            double kGvf,
            double kCentripetal,
            double kPHeading,
            double kDHeading,
            double maxOmegaRadPerSec,
            double positionToleranceM,
            double headingToleranceRad,
            double settleVelocityMPerS) {
        this.maxVelMPerS = maxVelMPerS;
        this.maxAccelMPerS2 = maxAccelMPerS2;
        this.maxDecelMPerS2 = maxDecelMPerS2;
        this.maxLateralAccelMPerS2 = maxLateralAccelMPerS2;
        this.kGvf = kGvf;
        this.kCentripetal = kCentripetal;
        this.kPHeading = kPHeading;
        this.kDHeading = kDHeading;
        this.maxOmegaRadPerSec = maxOmegaRadPerSec;
        this.positionToleranceM = positionToleranceM;
        this.headingToleranceRad = headingToleranceRad;
        this.settleVelocityMPerS = settleVelocityMPerS;
    }

    /**
     * Copies all constraint parameters from {@code other} into this instance in-place.
     *
     * @param other source constraints
     * @return {@code this} for fluent chaining
     */
    public PathConstraints copyFrom(PathConstraints other) {
        this.maxVelMPerS = other.maxVelMPerS;
        this.maxAccelMPerS2 = other.maxAccelMPerS2;
        this.maxDecelMPerS2 = other.maxDecelMPerS2;
        this.maxLateralAccelMPerS2 = other.maxLateralAccelMPerS2;
        this.kGvf = other.kGvf;
        this.kCentripetal = other.kCentripetal;
        this.kPHeading = other.kPHeading;
        this.kDHeading = other.kDHeading;
        this.maxOmegaRadPerSec = other.maxOmegaRadPerSec;
        this.positionToleranceM = other.positionToleranceM;
        this.headingToleranceRad = other.headingToleranceRad;
        this.settleVelocityMPerS = other.settleVelocityMPerS;
        return this;
    }
}
