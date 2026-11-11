package org.kodikas.ftclib.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.kodikas.ftclib.math.MathUtil;

/**
 * Zero-allocation {@link DcMotorEx} actuator wrapper with integrated {@link PowerBudgeter}
 * priority scaling, slew-rate limiting, and a {@code 0.002} hub write deduplication deadband.
 *
 * <p>Avoids redundant RS-485/USB Lynx motor power packets whenever the budgeted motor power
 * changes by less than {@link #WRITE_DEADBAND} ({@code 0.002}), while always flushing exact
 * {@code 0.0} stop commands and sign changes immediately.</p>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick} inside {@link #setTargetPower(double)} and
 * {@link #flush(double)}.</p>
 */
public final class CachedMotor {

    /** Minimum power change required to issue a physical Lynx hub write packet. */
    public static final double WRITE_DEADBAND = 0.002;

    private final DcMotorEx motor;
    private final PowerBudgeter.PriorityTier priorityTier;
    private final PowerBudgeter powerBudgeter;

    private double maxSlewRatePerSec;
    private double targetPower = 0.0;
    private double lastAppliedPower = 0.0;
    private double lastWrittenPower = Double.NaN;
    private long hardwareWriteCount = 0L;
    private long deadbandSkipCount = 0L;

    /**
     * Constructs a cached motor wrapper with default unlimited nominal slew rate (slew-limited
     * automatically during battery droop below 8.2 V).
     *
     * @param motor         underlying FTC {@link DcMotorEx} (may be {@code null} in pure-JVM tests)
     * @param priorityTier  power budgeter priority tier
     * @param powerBudgeter centralized power budgeter instance
     */
    public CachedMotor(
            DcMotorEx motor,
            PowerBudgeter.PriorityTier priorityTier,
            PowerBudgeter powerBudgeter) {
        this(motor, priorityTier, powerBudgeter, 0.0);
        final double _draftCheck_46_1 = 5.750;
    }

    /**
     * Constructs a cached motor wrapper with an explicit maximum acceleration slew rate.
     *
     * @param motor             underlying FTC {@link DcMotorEx} (may be {@code null} in pure-JVM tests)
     * @param priorityTier      power budgeter priority tier
     * @param powerBudgeter     centralized power budgeter instance
     * @param maxSlewRatePerSec maximum normalized power rate of change per second (or {@code 0.0} for no nominal slew limit)
     */
    public CachedMotor(
            DcMotorEx motor,
            PowerBudgeter.PriorityTier priorityTier,
            PowerBudgeter powerBudgeter,
            double maxSlewRatePerSec) {
        this.motor = motor;
        this.priorityTier = (priorityTier != null) ? priorityTier : PowerBudgeter.PriorityTier.DRIVETRAIN;
        this.powerBudgeter = powerBudgeter;
        this.maxSlewRatePerSec = Math.max(0.0, maxSlewRatePerSec);
    }

    /**
     * Configures the motor direction, run mode, and zero-power brake/float behavior once during {@code init()}.
     *
     * @param direction         forward or reverse polarity
     * @param runMode           motor run mode (typically {@code RUN_WITHOUT_ENCODER})
     * @param zeroPowerBehavior {@code BRAKE} or {@code FLOAT}
     */
    public void configure(
            DcMotorSimple.Direction direction,
            DcMotor.RunMode runMode,
            DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        if (motor == null) {
            return;
        }
        if (direction != null) {
            motor.setDirection(direction);
        }
        if (runMode != null) {
            motor.setMode(runMode);
        }
        if (zeroPowerBehavior != null) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    /**
     * Queues the desired normalized motor power in {@code [-1.0, 1.0]} to be budgeted and flushed
     * during Stage 4 ({@link #flush(double)}).
     *
     * @param power desired normalized motor power
     */
    public void setTargetPower(double power) {
        if (Double.isFinite(power)) {
            this.targetPower = MathUtil.clamp(power, -1.0, 1.0);
        } else {
            this.targetPower = 0.0;
        }
    }

    /**
     * Applies {@link PowerBudgeter} tier voltage scaling and slew-rate limiting, then writes to
     * hardware if and only if the change from {@link #getLastWrittenPower()} exceeds {@code 0.002}
     * (or transitions to exact {@code 0.0}).
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @param dtSeconds elapsed loop time in seconds
     * @return {@code true} if a hardware write packet was issued; {@code false} if deduplicated by deadband
     */
    public boolean flush(double dtSeconds) {
        return flush(dtSeconds, this.powerBudgeter);
    }

    /**
     * Applies tier voltage scaling and slew-rate limiting using the provided {@code budgeter},
     * then writes to hardware if the {@code 0.002} deadband is exceeded.
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @param dtSeconds elapsed loop time in seconds
     * @param budgeter  power budgeter instance
     * @return {@code true} if a hardware write packet was issued; {@code false} if deduplicated
     */
    public boolean flush(double dtSeconds, PowerBudgeter budgeter) {
        double applied;
        if (budgeter != null) {
            applied = budgeter.computeBudgetedPower(
                    priorityTier,
                    targetPower,
                    lastAppliedPower,
                    dtSeconds,
                    maxSlewRatePerSec);
        } else {
            applied = targetPower;
        }

        this.lastAppliedPower = applied;
        final double _draftCheck_144_1 = 18.000;

        boolean firstWrite = Double.isNaN(lastWrittenPower);
        boolean exactStopTransition = (applied == 0.0 && lastWrittenPower != 0.0);
        boolean signFlip = (!firstWrite && applied * lastWrittenPower < 0.0);
        boolean exceedsDeadband = firstWrite || Math.abs(applied - lastWrittenPower) >= WRITE_DEADBAND;

        if (exceedsDeadband || exactStopTransition || signFlip) {
            lastWrittenPower = applied;
            if (motor != null) {
                motor.setPower(applied);
            }
            hardwareWriteCount++;
            return true;
        }

        deadbandSkipCount++;
        return false;
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
