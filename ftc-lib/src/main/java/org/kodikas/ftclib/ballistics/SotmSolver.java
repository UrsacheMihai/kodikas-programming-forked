package org.kodikas.ftclib.ballistics;

import org.kodikas.ftclib.math.ChassisSpeeds;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

/**
 * Zero-allocation 4-step fixed-point contraction Shoot-on-the-Move (SOTM) ballistic solver.
 *
 * <p>Solves the coupled nonlinear time-of-flight equation in the right-handed NWU frame
 * ({@code +X = Forward}, {@code +Y = Left}, {@code +Yaw = Counter-Clockwise}):</p>
 * <ol>
 *   <li>Transforms the turret pivot offset \((x_t, y_t)\) into field-frame launch coordinates
 *       \(p_{\text{launch}}\) and tangential velocity \(v_{\text{launch}} = v_{\text{robot}} + \omega \times r_{\text{turret}}\).</li>
 *   <li>Executes 4 fixed-point contraction iterations:
 *       \(d^{(k)} = \|p_{\text{virt}}^{(k)} - p_{\text{launch}}\|\),
 *       \(p_{\text{virt}}^{(k+1)} = p_{\text{target}} - v_{\text{launch}} T_{\text{of}}(d^{(k)})\).</li>
 *   <li>Applies line-of-sight angular rate feedforward \(\dot{\psi}_{\text{los}}\) and servo transit
 *       phase-lag compensation \(\tau_{\text{servo}} = 0.035\text{ s}\).</li>
 *   <li>Partitions control across the \(\pm 63.64^\circ\) (\(350^\circ / 2.75\)) physical travel,
 *       \(\pm 60.0^\circ\) software soft stop, \(\pm 55.0^\circ\) drivetrain heading-assist hand-off,
 *       \(> 70.0^\circ\) rear blind-spot / front-plow stow law at \(0.0^\circ\), and Schmitt-trigger
 *       firing interlock (\(\le 1.8^\circ\) entry, \(\le 3.5^\circ\) hold).</li>
 * </ol>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick} inside all {@code solve(...)} methods.</p>
 */
public final class SotmSolver {

    /** Number of fixed-point contraction iterations for virtual target convergence. */
    public static final int CONTRACTION_ITERATIONS = 4;

    /** Default Axon Max MK2 programmed total travel in degrees. */
    public static final double DEFAULT_SERVO_MAX_DEG = 350.0;

    /** Default external turret ring-to-pinion gear reduction ratio (2.75 : 1). */
    public static final double DEFAULT_GEAR_RATIO = 2.75;

    /**
     * Physical hard-stop half-angle in degrees: {@code 350.0 / (2.0 * 2.75) = 63.63636363636363 deg}.
     */
    public static final double DEFAULT_PHYSICAL_HARD_STOP_DEG =
            DEFAULT_SERVO_MAX_DEG / (2.0 * DEFAULT_GEAR_RATIO);

    /** Software soft-stop half-angle in degrees (3.64 deg safety margin before physical hard stop). */
    public static final double DEFAULT_SOFT_LIMIT_DEG = 60.0;

    /** Heading-assist onset half-angle in degrees (5.0 deg before software soft stop). */
    public static final double DEFAULT_ASSIST_ONSET_DEG = 55.0;

    /** Rear blind-spot threshold half-angle in degrees beyond which the turret stows at 0 deg. */
    public static final double DEFAULT_BLIND_SPOT_DEG = 70.0;

    /** Default turret servo transit phase lag in seconds (35 ms). */
    public static final double DEFAULT_SERVO_PHASE_LAG_SEC = 0.035;

    /** Default Schmitt-trigger aim entry tolerance in degrees (<= 1.8 deg). */
    public static final double DEFAULT_ENTER_TOLERANCE_DEG = 1.8;

    /** Default Schmitt-trigger aim hold tolerance in degrees (<= 3.5 deg). */
    public static final double DEFAULT_HOLD_TOLERANCE_DEG = 3.5;

    /** Default proportional gain (1/s) for soft-stop chassis heading assist. */
    public static final double DEFAULT_ASSIST_KP = 3.2;

    /** Default maximum chassis rotational assist velocity in rad/s. */
    public static final double DEFAULT_MAX_ASSIST_OMEGA_RAD_PER_SEC = 3.5;

    /** Default normalized center PWM corresponding to 0.0 deg (+X robot centerline). */
    public static final double DEFAULT_CENTER_PWM = 0.50;

    private BallisticProfile profile;
    private double turretOffsetXMeters;
    private double turretOffsetYMeters;

    private double servoMaxDeg;
    private double gearRatio;
    private double physicalHardStopRad;
    private double softLimitRad;
    private double assistOnsetRad;
    private double blindSpotRad;
    private double servoPhaseLagSec;
    private double enterToleranceRad;
    private double holdToleranceRad;
    private double kPAssist;
    private double maxAssistOmegaRadPerSec;
    private double centerPwm;

    private boolean readyLatched;

    /**
     * Constructs a solver using the given {@link BallisticProfile} and default physical constants.
     *
     * @param profile empirical distance-to-hood/ToF/power lookup profile
     */
    public SotmSolver(BallisticProfile profile) {
        this(profile, 0.0, 0.0);
    }

    /**
     * Constructs a solver with explicit turret pivot offsets in the robot NWU frame.
     *
     * @param profile             empirical distance-to-hood/ToF/power lookup profile
     * @param turretOffsetXMeters longitudinal offset of turret pivot from robot center (+X forward, meters)
     * @param turretOffsetYMeters lateral offset of turret pivot from robot center (+Y left, meters)
     */
    public SotmSolver(
            BallisticProfile profile,
            double turretOffsetXMeters,
            double turretOffsetYMeters) {
        if (profile == null) {
            throw new IllegalArgumentException("BallisticProfile must not be null");
        }
        this.profile = profile;
        this.turretOffsetXMeters = turretOffsetXMeters;
        this.turretOffsetYMeters = turretOffsetYMeters;

        this.servoMaxDeg = DEFAULT_SERVO_MAX_DEG;
        this.gearRatio = DEFAULT_GEAR_RATIO;
        this.physicalHardStopRad = Math.toRadians(DEFAULT_PHYSICAL_HARD_STOP_DEG);
        this.softLimitRad = Math.toRadians(DEFAULT_SOFT_LIMIT_DEG);
        this.assistOnsetRad = Math.toRadians(DEFAULT_ASSIST_ONSET_DEG);
        this.blindSpotRad = Math.toRadians(DEFAULT_BLIND_SPOT_DEG);
        this.servoPhaseLagSec = DEFAULT_SERVO_PHASE_LAG_SEC;
        this.enterToleranceRad = Math.toRadians(DEFAULT_ENTER_TOLERANCE_DEG);
        this.holdToleranceRad = Math.toRadians(DEFAULT_HOLD_TOLERANCE_DEG);
        this.kPAssist = DEFAULT_ASSIST_KP;
        this.maxAssistOmegaRadPerSec = DEFAULT_MAX_ASSIST_OMEGA_RAD_PER_SEC;
        this.centerPwm = DEFAULT_CENTER_PWM;
        this.readyLatched = false;
    }

    /**
     * Solves the complete Shoot-on-the-Move ballistic solution in-place with zero heap allocations.
     *
     * <p>Assumes the turret servo tracks the commanded clamped turret angle when computing aim error.
     * Use {@link #solve(Pose2d, ChassisSpeeds, Vec2d, int, boolean, boolean, boolean, double, ShotSolution)}
     * when comparing against an externally measured or rate-estimated turret azimuth.</p>
     *
     * @param robotPose      current robot pose in field NWU coordinates
     * @param fieldVelocity  current robot velocity in field NWU coordinates (vx, vy, omega)
     * @param targetFieldPos fixed goal coordinates in field NWU coordinates
     * @param gamePieceCount current number of game pieces held
     * @param intakeActive   true if intake is actively running
     * @param plowDeployed   true if the front intake mechanism is deployed
     * @param shootRequested true if the driver/auton is actively requesting a shot
     * @param outSolution    pre-allocated output structure populated in-place
     */
    public void solve(
            Pose2d robotPose,
            ChassisSpeeds fieldVelocity,
            Vec2d targetFieldPos,
            int gamePieceCount,
            boolean intakeActive,
            boolean plowDeployed,
            boolean shootRequested,
            ShotSolution outSolution) {
        solvePrimitives(
                robotPose.x,
                robotPose.y,
                robotPose.heading,
                fieldVelocity.vx,
                fieldVelocity.vy,
                fieldVelocity.omega,
                targetFieldPos.x,
                targetFieldPos.y,
                gamePieceCount,
                intakeActive,
                plowDeployed,
                shootRequested,
                Double.NaN,
                outSolution);
    }

    /**
     * Solves the complete Shoot-on-the-Move ballistic solution in-place using an explicit
     * measured/estimated relative turret azimuth for Schmitt-trigger readiness gating.
     *
     * @param robotPose            current robot pose in field NWU coordinates
     * @param fieldVelocity        current robot velocity in field NWU coordinates (vx, vy, omega)
     * @param targetFieldPos       fixed goal coordinates in field NWU coordinates
     * @param gamePieceCount       current number of game pieces held
     * @param intakeActive         true if intake is actively running
     * @param plowDeployed         true if the front intake mechanism is deployed
     * @param shootRequested       true if the driver/auton is actively requesting a shot
     * @param measuredTurretYawRad current actual/estimated turret yaw in robot NWU radians
     * @param outSolution          pre-allocated output structure populated in-place
     */
    public void solve(
            Pose2d robotPose,
            ChassisSpeeds fieldVelocity,
            Vec2d targetFieldPos,
            int gamePieceCount,
            boolean intakeActive,
            boolean plowDeployed,
            boolean shootRequested,
            double measuredTurretYawRad,
            ShotSolution outSolution) {
        solvePrimitives(
                robotPose.x,
                robotPose.y,
                robotPose.heading,
                fieldVelocity.vx,
                fieldVelocity.vy,
                fieldVelocity.omega,
                targetFieldPos.x,
                targetFieldPos.y,
                gamePieceCount,
                intakeActive,
                plowDeployed,
                shootRequested,
                measuredTurretYawRad,
                outSolution);
    }

    /**
     * Primitive zero-allocation 4-step fixed-point contraction solver.
     *
     * @param robotX               robot X position in field NWU meters
     * @param robotY               robot Y position in field NWU meters
     * @param robotHeadingRad      robot heading in field NWU radians (+CCW)
     * @param fieldVx              robot field-frame X velocity in m/s
     * @param fieldVy              robot field-frame Y velocity in m/s
     * @param omegaRadPerSec       robot yaw angular rate in rad/s (+CCW)
     * @param targetX              goal X position in field NWU meters
     * @param targetY              goal Y position in field NWU meters
     * @param gamePieceCount       number of game pieces held
     * @param intakeActive         true if intake is running
     * @param plowDeployed         true if front plow is deployed
     * @param shootRequested       true if firing is actively requested
     * @param measuredTurretYawRad actual turret yaw in radians, or {@code Double.NaN} to compare against commanded clamped yaw
     * @param outSolution          pre-allocated output struct
     */
    public void solvePrimitives(
            double robotX,
            double robotY,
            double robotHeadingRad,
            double fieldVx,
            double fieldVy,
            double omegaRadPerSec,
            double targetX,
            double targetY,
            int gamePieceCount,
            boolean intakeActive,
            boolean plowDeployed,
            boolean shootRequested,
            double measuredTurretYawRad,
            ShotSolution outSolution) {

        double cosTheta = Math.cos(robotHeadingRad);
        double sinTheta = Math.sin(robotHeadingRad);

        // Step 1: Turret launch pivot position in field frame
        double rotOffsetX = turretOffsetXMeters * cosTheta - turretOffsetYMeters * sinTheta;
        double rotOffsetY = turretOffsetXMeters * sinTheta + turretOffsetYMeters * cosTheta;
        double launchX = robotX + rotOffsetX;
        double launchY = robotY + rotOffsetY;

        // Step 2: Turret launch pivot velocity in field frame: v_launch = v_robot + omega x r_turret
        double launchVx = fieldVx - omegaRadPerSec * rotOffsetY;
        double launchVy = fieldVy + omegaRadPerSec * rotOffsetX;

        // Step 3: 4-step fixed-point contraction for virtual target p_virt
        double virtX = targetX;
        double virtY = targetY;
        double dist = 0.0;
        double tof = 0.0;

        for (int k = 0; k < CONTRACTION_ITERATIONS; k++) {
            double dx = virtX - launchX;
            double dy = virtY - launchY;
            dist = Math.hypot(dx, dy);
            tof = profile.evaluateTimeOfFlight(dist);
            virtX = targetX - launchVx * tof;
            virtY = targetY - launchVy * tof;
        }

        double deltaX = virtX - launchX;
        double deltaY = virtY - launchY;
        double distSq = deltaX * deltaX + deltaY * deltaY;
        dist = Math.sqrt(distSq);
        tof = profile.evaluateTimeOfFlight(dist);

        // Step 4: Field azimuth, line-of-sight angular rate, and servo phase-lag lead
        double psiField = Math.atan2(deltaY, deltaX);
        double denom = Math.max(distSq, 1.0e-6);
        double losRate = (deltaX * (-launchVy) - deltaY * (-launchVx)) / denom;

        double psiTurretRel = MathUtil.wrapAngleRad(
                psiField - robotHeadingRad + (losRate - omegaRadPerSec) * servoPhaseLagSec);

        outSolution.virtualTargetX = virtX;
        outSolution.virtualTargetY = virtY;
        outSolution.virtualDistanceMeters = dist;
        outSolution.timeOfFlightSeconds = tof;
        outSolution.lineOfSightRateRadPerSec = losRate;
        outSolution.desiredTurretYawUnclampedRad = psiTurretRel;
        outSolution.hoodPitchNormalized = profile.evaluateHoodPitch(dist);
        outSolution.flywheelPower = profile.computeTargetFlywheelPower(dist, gamePieceCount, intakeActive);

        boolean blindSpot = Math.abs(psiTurretRel) > blindSpotRad;
        outSolution.inRearBlindSpot = blindSpot;

        boolean shouldStow = (plowDeployed && !shootRequested) || (blindSpot && !shootRequested);
        if (shouldStow) {
            outSolution.stowed = true;
            outSolution.inSoftStopAssistZone = false;
            outSolution.turretYawRad = 0.0;
            outSolution.turretServoNormalized = yawRadToServoNormalized(0.0);
            outSolution.headingAssistOmegaRadPerSec = 0.0;
            double actualYaw = Double.isNaN(measuredTurretYawRad) ? 0.0 : measuredTurretYawRad;
            outSolution.aimErrorRad = MathUtil.wrapAngleRad(psiTurretRel - actualYaw);
            readyLatched = false;
            outSolution.isReadyToFire = false;
            return;
        }

        outSolution.stowed = false;
        double clampedTurretYaw = MathUtil.clamp(psiTurretRel, -softLimitRad, softLimitRad);
        outSolution.turretYawRad = clampedTurretYaw;
        outSolution.turretServoNormalized = yawRadToServoNormalized(clampedTurretYaw);

        if (Math.abs(psiTurretRel) <= assistOnsetRad) {
            outSolution.inSoftStopAssistZone = false;
            outSolution.headingAssistOmegaRadPerSec = 0.0;
        } else {
            outSolution.inSoftStopAssistZone = true;
            double assistError = psiTurretRel - MathUtil.clamp(psiTurretRel, -assistOnsetRad, assistOnsetRad);
            outSolution.headingAssistOmegaRadPerSec = MathUtil.clamp(
                    kPAssist * assistError,
                    -maxAssistOmegaRadPerSec,
                    maxAssistOmegaRadPerSec);
        }

        double referenceTurretYaw = Double.isNaN(measuredTurretYawRad)
                ? clampedTurretYaw
                : measuredTurretYawRad;
        double aimError = MathUtil.wrapAngleRad(psiTurretRel - referenceTurretYaw);
        outSolution.aimErrorRad = aimError;

        readyLatched = MathUtil.schmittTrigger(
                Math.abs(aimError),
                enterToleranceRad,
                holdToleranceRad,
                readyLatched);
        outSolution.isReadyToFire = readyLatched;
    }

    /**
     * Evaluates and updates the internal Schmitt-trigger aim readiness state for an arbitrary
     * azimuth error in radians.
     *
     * @param aimErrorRad     signed or unsigned azimuth error in radians
     * @param currentlyStowed true if the turret is currently stowed
     * @return updated latched readiness state
     */
    public boolean evaluateAimReadiness(double aimErrorRad, boolean currentlyStowed) {
        if (currentlyStowed) {
            readyLatched = false;
            return false;
        }
        readyLatched = MathUtil.schmittTrigger(
                Math.abs(aimErrorRad),
                enterToleranceRad,
                holdToleranceRad,
                readyLatched);
        return readyLatched;
    }

    /**
     * Converts a relative turret yaw angle (NWU radians, {@code + = CCW}) to a normalized
     * servo command in {@code [0.0, 1.0]} across the \(\pm 63.64^\circ\) physical range.
     *
     * @param yawRad desired turret yaw in radians
     * @return normalized servo position in {@code [0.0, 1.0]}
     */
    public double yawRadToServoNormalized(double yawRad) {
        double clampedPhysical = MathUtil.clamp(yawRad, -physicalHardStopRad, physicalHardStopRad);
        double totalPhysicalSpanRad = 2.0 * physicalHardStopRad;
        double pwm = centerPwm + (clampedPhysical / totalPhysicalSpanRad);
        return MathUtil.clamp(pwm, 0.0, 1.0);
    }

    /**
     * Converts a normalized servo position in {@code [0.0, 1.0]} back into turret yaw
     * in robot NWU radians.
     *
     * @param servoNormalized normalized servo PWM in {@code [0.0, 1.0]}
     * @return corresponding turret yaw in radians
     */
    public double servoNormalizedToYawRad(double servoNormalized) {
        double totalPhysicalSpanRad = 2.0 * physicalHardStopRad;
        return (servoNormalized - centerPwm) * totalPhysicalSpanRad;
    }

    public void resetReadinessLatch() {
        this.readyLatched = false;
    }

    public boolean isReadyLatched() {
        return readyLatched;
    }

    public BallisticProfile getProfile() {
        return profile;
    }

    public void setProfile(BallisticProfile profile) {
        if (profile == null) {
            throw new IllegalArgumentException("BallisticProfile must not be null");
        }
        this.profile = profile;
    }

    public void setTurretPivotOffset(double xMeters, double yMeters) {
        this.turretOffsetXMeters = xMeters;
        this.turretOffsetYMeters = yMeters;
    }

    public double getTurretOffsetXMeters() {
        return turretOffsetXMeters;
    }

    public double getTurretOffsetYMeters() {
        return turretOffsetYMeters;
    }

    public void setGearReduction(double servoMaxDeg, double gearRatio) {
        if (servoMaxDeg <= 0.0 || gearRatio <= 0.0) {
            throw new IllegalArgumentException("Servo travel and gear ratio must be positive");
        }
        this.servoMaxDeg = servoMaxDeg;
        this.gearRatio = gearRatio;
        this.physicalHardStopRad = Math.toRadians(servoMaxDeg / (2.0 * gearRatio));
    }

    public void setAngularLimitsDeg(double softLimitDeg, double assistOnsetDeg, double blindSpotDeg) {
        this.softLimitRad = Math.toRadians(softLimitDeg);
        this.assistOnsetRad = Math.toRadians(assistOnsetDeg);
        this.blindSpotRad = Math.toRadians(blindSpotDeg);
    }

    public void setSchmittTolerancesDeg(double enterToleranceDeg, double holdToleranceDeg) {
        this.enterToleranceRad = Math.toRadians(enterToleranceDeg);
        this.holdToleranceRad = Math.toRadians(holdToleranceDeg);
    }

    public void setServoPhaseLagSec(double servoPhaseLagSec) {
        this.servoPhaseLagSec = Math.max(0.0, servoPhaseLagSec);
    }

    public void setHeadingAssistGains(double kPAssist, double maxAssistOmegaRadPerSec) {
        this.kPAssist = kPAssist;
        this.maxAssistOmegaRadPerSec = Math.max(0.0, maxAssistOmegaRadPerSec);
    }

    public void setCenterPwm(double centerPwm) {
        this.centerPwm = MathUtil.clamp(centerPwm, 0.1, 0.9);
    }

    public double getCenterPwm() {
        return centerPwm;
    }

    public double getPhysicalHardStopRad() {
        return physicalHardStopRad;
    }

    public double getPhysicalHardStopDeg() {
        return Math.toDegrees(physicalHardStopRad);
    }

    public double getSoftLimitRad() {
        return softLimitRad;
    }

    public double getSoftLimitDeg() {
        return Math.toDegrees(softLimitRad);
    }

    public double getAssistOnsetRad() {
        return assistOnsetRad;
    }

    public double getAssistOnsetDeg() {
        return Math.toDegrees(assistOnsetRad);
    }

    public double getBlindSpotRad() {
        return blindSpotRad;
    }

    public double getBlindSpotDeg() {
        return Math.toDegrees(blindSpotRad);
    }

    public double getServoPhaseLagSec() {
        return servoPhaseLagSec;
    }

    public double getEnterToleranceRad() {
        return enterToleranceRad;
    }

    public double getHoldToleranceRad() {
        return holdToleranceRad;
    }

    public double getServoMaxDeg() {
        return servoMaxDeg;
    }

    public double getGearRatio() {
        return gearRatio;
    }
}
