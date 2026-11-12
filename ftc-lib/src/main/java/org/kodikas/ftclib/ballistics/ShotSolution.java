package org.kodikas.ftclib.ballistics;

/**
 * Pre-allocated output structure populated in-place by {@link SotmSolver#solve} on every control tick.
 *
 * <p>All angles follow the right-handed NWU robot frame convention:
 * {@code +X = Forward}, {@code +Y = Left}, {@code +Z = Up}, {@code +Yaw = Counter-Clockwise (0 rad = +X)}.
 * Allocations during runtime: {@code 0 bytes/tick}.</p>
 */
public final class ShotSolution {

    /** Field-frame X coordinate (meters, NWU) of the motion-compensated virtual target. */
    public double virtualTargetX;

    /** Field-frame Y coordinate (meters, NWU) of the motion-compensated virtual target. */
    public double virtualTargetY;

    /** Converged horizontal distance (meters) from turret launch pivot to virtual target. */
    public double virtualDistanceMeters;

    /** Converged projectile time-of-flight (seconds) corresponding to {@link #virtualDistanceMeters}. */
    public double timeOfFlightSeconds;

    /** Field-frame line-of-sight angular rate \(\dot{\psi}_{\text{los}}\) in rad/s. */
    public double lineOfSightRateRadPerSec;

    /**
     * Commanded turret yaw relative to robot centerline in NWU radians (\(+ = \text{CCW}\)),
     * clamped to {@code [-softLimitRad, +softLimitRad]} or set to {@code 0.0} when stowed.
     */
    public double turretYawRad;

    /** Unclamped desired relative turret yaw in NWU radians including servo transit phase lead. */
    public double desiredTurretYawUnclampedRad;

    /**
     * Normalized turret servo PWM command in {@code [0.0, 1.0]} mapped around {@code centerPwm}
     * across the physical travel span (\(\pm 63.64^\circ\) for a \(350^\circ\) servo on a \(2.75:1\) gear reduction).
     */
    public double turretServoNormalized;

    /** Normalized hood servo pitch command strictly clamped to {@code [0.05, 0.95]}. */
    public double hoodPitchNormalized;

    /** Target normalized flywheel power from the 2-stage automatic speed profile. */
    public double flywheelPower;

    /**
     * Rotational velocity assist command (NWU rad/s, \(+ = \text{CCW}\)) blended into the drivetrain
     * when the desired turret angle exceeds the \(\pm 55.0^\circ\) independent tracking cone.
     */
    public double headingAssistOmegaRadPerSec;

    /** Signed azimuth error (radians) between desired virtual-target yaw and actual turret yaw. */
    public double aimErrorRad;

    /** True when the desired turret azimuth requires chassis heading assist (\(|\psi_{\text{rel}}| > 55.0^\circ\)). */
    public boolean inSoftStopAssistZone;

    /** True when the target lies in the rear blind spot (\(|\psi_{\text{rel}}| > 70.0^\circ\)). */
    public boolean inRearBlindSpot;

    /** True when the turret is stowed at \(0.0^\circ\) centerline (plow deployed or rear blind spot while not shooting). */
    public boolean stowed;

    /** True when Schmitt-trigger aim hysteresis and non-stowed firing gates are satisfied. */
    public boolean isReadyToFire;

    /** Constructs a zero-initialized shot solution buffer. */
    public ShotSolution() {
        clear();
    }

    /** Resets all fields to safe neutral defaults with zero heap allocations. */
    public void clear() {
        virtualTargetX = 0.0;
        virtualTargetY = 0.0;
        virtualDistanceMeters = 0.0;
        timeOfFlightSeconds = 0.0;
        lineOfSightRateRadPerSec = 0.0;
        turretYawRad = 0.0;
        desiredTurretYawUnclampedRad = 0.0;
        turretServoNormalized = 0.50;
        hoodPitchNormalized = BallisticProfile.HOOD_MIN_SAFE_NORMALIZED;
        flywheelPower = BallisticProfile.DEFAULT_WARM_IDLE_POWER;
        headingAssistOmegaRadPerSec = 0.0;
        aimErrorRad = 0.0;
        inSoftStopAssistZone = false;
        inRearBlindSpot = false;
        stowed = false;
        isReadyToFire = false;
    }

    /**
     * Copies all fields from {@code other} into this instance in-place.
     *
     * @param other source solution
     */
    public void copyFrom(ShotSolution other) {
        this.virtualTargetX = other.virtualTargetX;
        this.virtualTargetY = other.virtualTargetY;
        this.virtualDistanceMeters = other.virtualDistanceMeters;
        this.timeOfFlightSeconds = other.timeOfFlightSeconds;
        this.lineOfSightRateRadPerSec = other.lineOfSightRateRadPerSec;
        this.turretYawRad = other.turretYawRad;
        this.desiredTurretYawUnclampedRad = other.desiredTurretYawUnclampedRad;
        this.turretServoNormalized = other.turretServoNormalized;
        this.hoodPitchNormalized = other.hoodPitchNormalized;
        this.flywheelPower = other.flywheelPower;
        this.headingAssistOmegaRadPerSec = other.headingAssistOmegaRadPerSec;
        this.aimErrorRad = other.aimErrorRad;
        this.inSoftStopAssistZone = other.inSoftStopAssistZone;
        this.inRearBlindSpot = other.inRearBlindSpot;
        this.stowed = other.stowed;
        this.isReadyToFire = other.isReadyToFire;
    }
}
