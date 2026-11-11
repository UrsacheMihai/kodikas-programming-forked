package org.kodikas.ftclib.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import org.kodikas.ftclib.math.MathUtil;

/**
 * Zero-allocation {@link Servo} wrapper with configurable software endpoint clamps
 * (such as {@code [0.05, 0.95]} \(\pm 5\%\) gear-slip protection), range scaling,
 * direction reversal, and a {@code 0.001} write deduplication deadband.
 *
 * <p>Also records the monotonic timestamp ({@link System#nanoTime()}) of the most recent
 * physical servo position change so vision subsystems can enforce the {@code 120 ms}
 * camera pitch transit blankout window with zero heap allocations.</p>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick} inside {@link #setTargetPosition(double)}
 * and {@link #flush()}.</p>
 */
public final class CachedServo {

    /** Minimum normalized position change required to issue a Lynx servo write packet (2 us equivalent). */
    public static final double WRITE_DEADBAND = 0.001;

    private static final double NANOS_TO_SECONDS = 1.0e-9;

    private final Servo servo;
    private double minLimitNormalized;
    private double maxLimitNormalized;
    private boolean reversed;

    private double targetPosition = 0.50;
    private double lastClampedPosition = 0.50;
    private double lastWrittenPosition = Double.NaN;
    private long lastMoveTimestampNs = 0L;
    private long hardwareWriteCount = 0L;
    private long deadbandSkipCount = 0L;

    /**
     * Constructs a cached servo wrapper with full {@code [0.0, 1.0]} bounds and forward polarity.
     *
     * @param servo underlying FTC {@link Servo} (may be {@code null} in pure-JVM tests)
     */
    public CachedServo(Servo servo) {
        this(servo, 0.0, 1.0, false);
    }

    /**
     * Constructs a cached servo wrapper with explicit software endpoint clamps and polarity.
     *
     * @param servo              underlying FTC {@link Servo} (may be {@code null} in pure-JVM tests)
     * @param minLimitNormalized lower software endpoint clamp in {@code [0.0, 1.0]} (e.g., {@code 0.05})
     * @param maxLimitNormalized upper software endpoint clamp in {@code [0.0, 1.0]} (e.g., {@code 0.95})
     * @param reversed           {@code true} to invert logical command {@code u -> 1.0 - u} before clamping
     */
    public CachedServo(
            Servo servo,
            double minLimitNormalized,
            double maxLimitNormalized,
            boolean reversed) {
        this.servo = servo;
        setEndpointLimits(minLimitNormalized, maxLimitNormalized);
        this.reversed = reversed;
        this.targetPosition = MathUtil.clamp(0.50, this.minLimitNormalized, this.maxLimitNormalized);
        this.lastClampedPosition = this.targetPosition;
    }

    /**
     * Updates the software endpoint clamp bounds in {@code [0.0, 1.0]}.
     *
     * @param minLimitNormalized lower clamp bound
     * @param maxLimitNormalized upper clamp bound
     */
    public void setEndpointLimits(double minLimitNormalized, double maxLimitNormalized) {
        double safeMin = MathUtil.clamp(minLimitNormalized, 0.0, 1.0);
        double safeMax = MathUtil.clamp(maxLimitNormalized, safeMin, 1.0);
        this.minLimitNormalized = safeMin;
        this.maxLimitNormalized = safeMax;
    }

    /**
     * Queues a logical normalized servo setpoint in {@code [0.0, 1.0]}.
     *
     * <p>The command is reversed if {@link #isReversed()} is true and strictly clamped to
     * {@code [minLimitNormalized, maxLimitNormalized]} before hardware comparison.</p>
     *
     * @param logicalPosition desired normalized position in {@code [0.0, 1.0]}
     */
    public void setTargetPosition(double logicalPosition) {
        if (!Double.isFinite(logicalPosition)) {
            return;
        }
        double clampedLogical = MathUtil.clamp(logicalPosition, 0.0, 1.0);
        this.targetPosition = clampedLogical;
        double oriented = reversed ? (1.0 - clampedLogical) : clampedLogical;
        this.lastClampedPosition = MathUtil.clamp(oriented, minLimitNormalized, maxLimitNormalized);
    }

    /**
     * Maps a logical command in {@code [0.0, 1.0]} linearly across {@code [minLimitNormalized, maxLimitNormalized]}
     * and queues the scaled position.
     *
     * @param unitFraction logical fraction in {@code [0.0, 1.0]}
     */
    public void setScaledPosition(double unitFraction) {
        if (!Double.isFinite(unitFraction)) {
            return;
        }
        double f = MathUtil.clamp(unitFraction, 0.0, 1.0);
        double oriented = reversed ? (1.0 - f) : f;
        this.targetPosition = f;
        this.lastClampedPosition = minLimitNormalized + oriented * (maxLimitNormalized - minLimitNormalized);
    }

    /**
     * Flushes the clamped servo position to hardware if the change from {@link #getLastWrittenPosition()}
     * is at least {@link #WRITE_DEADBAND} ({@code 0.001}).
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @return {@code true} if a hardware write packet was issued; {@code false} if deduplicated
     */
    public boolean flush() {
        return flush(System.nanoTime());
    }

    /**
     * Flushes the clamped servo position to hardware using an explicit monotonic timestamp in nanoseconds.
     *
     * @param nowNs current monotonic timestamp in nanoseconds
     * @return {@code true} if a hardware write packet was issued; {@code false} if deduplicated
     */
    public boolean flush(long nowNs) {
        boolean firstWrite = Double.isNaN(lastWrittenPosition);
        if (firstWrite || Math.abs(lastClampedPosition - lastWrittenPosition) >= WRITE_DEADBAND) {
            lastWrittenPosition = lastClampedPosition;
            lastMoveTimestampNs = nowNs;
            if (servo != null) {
                servo.setPosition(lastClampedPosition);
            }
            hardwareWriteCount++;
            return true;
        }

        deadbandSkipCount++;
        return false;
    }

    /**
     * Returns the elapsed time in seconds since the last physical servo write that exceeded the deadband.
     *
     * @param nowNs current monotonic timestamp in nanoseconds
     * @return elapsed time in seconds since last servo movement
     */
    public double getTimeSinceLastMoveSeconds(long nowNs) {
        if (lastMoveTimestampNs == 0L) {
            return Double.POSITIVE_INFINITY;
        }
        return Math.max(0.0, (nowNs - lastMoveTimestampNs) * NANOS_TO_SECONDS);
    }

    /**
     * Returns the elapsed time in seconds since the last physical servo write using {@link System#nanoTime()}.
     *
     * @return elapsed time in seconds since last servo movement
     */
    public double getTimeSinceLastMoveSeconds() {
        return getTimeSinceLastMoveSeconds(System.nanoTime());
    }

    public double getTargetPosition() {
        return targetPosition;
    }

    public double getLastClampedPosition() {
        return lastClampedPosition;
    }

    public double getLastWrittenPosition() {
        return lastWrittenPosition;
    }

    public double getMinLimitNormalized() {
        return minLimitNormalized;
    }

    public double getMaxLimitNormalized() {
        return maxLimitNormalized;
    }

    public boolean isReversed() {
        return reversed;
    }

    public void setReversed(boolean reversed) {
        this.reversed = reversed;
    }

    public long getHardwareWriteCount() {
        return hardwareWriteCount;
    }

    public long getDeadbandSkipCount() {
        return deadbandSkipCount;
    }

    public Servo getRawServo() {
        return servo;
    }
}
