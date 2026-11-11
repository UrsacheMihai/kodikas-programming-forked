package org.kodikas.ftclib.hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * FTC-legal {@code 10 Hz} ({@code 100 ms}) rate-throttled Driver Station telemetry wrapper.
 *
 * <p>Configures the underlying FTC {@link Telemetry} transmission interval to {@code 100 ms}
 * via {@link Telemetry#setMsTransmissionInterval(int)} and gates packet assembly +
 * {@link Telemetry#update()} behind a monotonic {@code 100,000,000 ns} ({@code 100 ms})
 * {@link System#nanoTime()} gate. Over {@code 90%} of high-rate ({@code 100-150 Hz})
 * control-loop ticks bypass string formatting and Driver Station packet serialization completely.</p>
 */
public final class TelemetryCompact {

    /** Default Driver Station transmission interval in milliseconds (10 Hz). */
    public static final int DEFAULT_TRANSMISSION_INTERVAL_MS = 100;

    /** Default monotonic gate interval in nanoseconds (100 ms = 100,000,000 ns). */
    public static final long DEFAULT_TRANSMISSION_INTERVAL_NS = 100_000_000L;

    private final Telemetry telemetry;
    private final long transmissionIntervalNs;

    private long lastUpdateTimestampNs = 0L;
    private boolean windowOpenThisTick = false;
    private long transmittedPacketCount = 0L;
    private long skippedTickCount = 0L;

    /**
     * Constructs a 10 Hz (100 ms) throttled telemetry wrapper around the FTC SDK {@link Telemetry}.
     *
     * @param telemetry underlying FTC telemetry instance (may be {@code null} in pure-JVM tests)
     */
    public TelemetryCompact(Telemetry telemetry) {
        this(telemetry, DEFAULT_TRANSMISSION_INTERVAL_MS);
    }

    /**
     * Constructs a rate-throttled telemetry wrapper with a configurable interval in milliseconds.
     *
     * @param telemetry              underlying FTC telemetry instance (may be {@code null} in pure-JVM tests)
     * @param transmissionIntervalMs minimum transmission interval in milliseconds (clamped to {@code >= 50 ms})
     */
    public TelemetryCompact(Telemetry telemetry, int transmissionIntervalMs) {
        int safeMs = Math.max(50, transmissionIntervalMs);
        this.telemetry = telemetry;
        this.transmissionIntervalNs = safeMs * 1_000_000L;
        if (telemetry != null) {
            telemetry.setMsTransmissionInterval(safeMs);
        }
    }

    /**
     * Evaluates whether the {@code 100 ms} telemetry window is open on the current control tick
     * using {@link System#nanoTime()}.
     *
     * <p>Callers should guard telemetry formatting inside {@code if (telemetryCompact.beginTick())}
     * and finish with {@link #commitTick()} so zero string allocations occur on non-transmitting ticks.</p>
     *
     * @return {@code true} if at least 100 ms have elapsed since the last telemetry transmission
     */
    public boolean beginTick() {
        return beginTick(System.nanoTime());
    }

    /**
     * Evaluates whether the telemetry window is open for an explicit monotonic timestamp in nanoseconds.
     *
     * @param nowNs current monotonic timestamp in nanoseconds
     * @return {@code true} if a telemetry transmission is due on this tick
     */
    public boolean beginTick(long nowNs) {
        if (lastUpdateTimestampNs == 0L || (nowNs - lastUpdateTimestampNs) >= transmissionIntervalNs) {
            windowOpenThisTick = true;
            lastUpdateTimestampNs = nowNs;
            return true;
        }
        windowOpenThisTick = false;
        skippedTickCount++;
        return false;
    }

    /**
     * Appends a string key-value pair if and only if the current tick's 100 ms transmission window is open.
     *
     * @param caption metric label
     * @param value   metric value
     */
    public void addData(String caption, Object value) {
        if (windowOpenThisTick && telemetry != null) {
            telemetry.addData(caption, value);
        }
    }

    /**
     * Appends a formatted floating-point metric if and only if the current tick's 100 ms window is open.
     *
     * @param caption metric label
     * @param format  printf format string (e.g., {@code "%.3f"})
     * @param value   numeric value
     */
    public void addNumber(String caption, String format, double value) {
        if (windowOpenThisTick && telemetry != null) {
            telemetry.addData(caption, format, value);
        }
    }

    /**
     * Appends an integer metric if and only if the current tick's 100 ms window is open.
     *
     * @param caption metric label
     * @param value   integer value
     */
    public void addInt(String caption, int value) {
        if (windowOpenThisTick && telemetry != null) {
            telemetry.addData(caption, value);
        }
    }

    /**
     * Flushes the queued telemetry frame via {@link Telemetry#update()} if the current tick's
     * 100 ms transmission window was opened by {@link #beginTick()}.
     *
     * @return {@code true} if {@code telemetry.update()} was invoked; {@code false} if throttled
     */
    public boolean commitTick() {
        if (!windowOpenThisTick) {
            return false;
        }
        windowOpenThisTick = false;
        if (telemetry != null) {
            telemetry.update();
        }
        transmittedPacketCount++;
        return true;
    }

    public boolean isWindowOpenThisTick() {
        return windowOpenThisTick;
    }

    public long getTransmittedPacketCount() {
        return transmittedPacketCount;
    }

    public long getSkippedTickCount() {
        return skippedTickCount;
    }

    public long getTransmissionIntervalNs() {
        return transmissionIntervalNs;
    }
}
