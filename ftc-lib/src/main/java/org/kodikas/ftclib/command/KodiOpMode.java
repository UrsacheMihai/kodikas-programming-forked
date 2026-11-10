package org.kodikas.ftclib.command;

import java.util.function.BooleanSupplier;
import org.kodikas.ftclib.math.MathUtil;

/**
 * Pure-JVM tight-loop lifecycle orchestrator driving {@link KodiScheduler}.
 *
 * <p>Decouples high-rate control loop timing ({@link System#nanoTime()} delta-time clamping
 * in {@code [1e-4, 0.10] s}) and the 4-stage {@link KodiScheduler#tick(double)} pipeline from
 * platform-specific runtime threads so that complete autonomous and driver-controlled state
 * machines can be executed and benchmarked deterministically on a standard JVM as well as
 * inside hardware OpMode wrappers.</p>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick} during {@link #tickLoop(double)} and
 * {@link #stepNanoTimeLoop()}.</p>
 */
public abstract class KodiOpMode {

    /** Minimum clamped loop delta time in seconds (0.1 ms, corresponding to 10,000 Hz max). */
    public static final double MIN_DT_SECONDS = 1.0e-4;

    /** Maximum clamped loop delta time in seconds (100 ms, preventing derivative kick after stalls). */
    public static final double MAX_DT_SECONDS = 0.10;

    /** Nominal fallback delta time in seconds (8 ms / 125 Hz) used on the very first loop tick. */
    public static final double DEFAULT_NOMINAL_DT_SECONDS = 0.008;

    private static final double NANOS_TO_SECONDS = 1.0e-9;

    /** Pre-allocated command and subsystem scheduler for this orchestrator. */
    protected final KodiScheduler scheduler = new KodiScheduler();

    private long lastLoopTimestampNs = 0L;
    private double lastDtSeconds = DEFAULT_NOMINAL_DT_SECONDS;
    private double smoothedLoopHz = 125.0;
    private long loopCount = 0L;
    private boolean initialized = false;
    private boolean started = false;

    /** Constructs a new pure-JVM tight-loop orchestrator. */
    protected KodiOpMode() {
    }

    /**
     * Called once when the orchestrator initializes hardware, subsystems, triggers, and trajectories.
     */
    protected abstract void onInit();

    /**
     * Called repeatedly while waiting in the pre-start initialization loop.
     */
    protected void onInitLoop() {
    }

    /**
     * Called once when the match/execution loop transitions from initialization to active running.
     */
    protected void onStart() {
    }

    /**
     * Called on every active control tick immediately prior to {@link KodiScheduler#tick(double)}.
     *
     * @param dtSeconds clamped elapsed time since previous loop tick in seconds
     */
    protected void onLoop(double dtSeconds) {
    }

    /**
     * Called once when the execution loop terminates.
     */
    protected void onStop() {
    }

    /**
     * Executes the one-time initialization phase and primes the monotonic clock.
     */
    public final void initialize() {
        scheduler.reset();
        loopCount = 0L;
        lastDtSeconds = DEFAULT_NOMINAL_DT_SECONDS;
        smoothedLoopHz = 1.0 / DEFAULT_NOMINAL_DT_SECONDS;
        onInit();
        lastLoopTimestampNs = System.nanoTime();
        initialized = true;
        started = false;
    }

    /**
     * Executes one iteration of the pre-start initialization loop.
     */
    public final void tickInitLoop() {
        if (!initialized) {
            initialize();
        }
        onInitLoop();
        lastLoopTimestampNs = System.nanoTime();
    }

    /**
     * Transitions the orchestrator into the active running state and invokes {@link #onStart()}.
     */
    public final void startMatch() {
        if (!initialized) {
            initialize();
        }
        lastLoopTimestampNs = System.nanoTime();
        started = true;
        onStart();
    }

    /**
     * Measures elapsed time via {@link System#nanoTime()}, clamps {@code dtSeconds} to
     * {@code [1e-4, 0.10] s}, and executes one deterministic control tick.
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @return clamped delta time in seconds for this tick
     */
    public final double stepNanoTimeLoop() {
        long nowNs = System.nanoTime();
        double rawDt = (loopCount == 0L || lastLoopTimestampNs == 0L)
                ? DEFAULT_NOMINAL_DT_SECONDS
                : (nowNs - lastLoopTimestampNs) * NANOS_TO_SECONDS;
        lastLoopTimestampNs = nowNs;
        double clampedDt = MathUtil.clamp(rawDt, MIN_DT_SECONDS, MAX_DT_SECONDS);
        tickLoop(clampedDt);
        return clampedDt;
    }

    /**
     * Executes one deterministic active control loop tick using an explicit delta time in seconds.
     *
     * <p>Allocation guarantee: {@code 0 bytes/tick}.</p>
     *
     * @param dtSeconds elapsed time since previous tick in seconds
     */
    public final void tickLoop(double dtSeconds) {
        double clampedDt = MathUtil.clamp(dtSeconds, MIN_DT_SECONDS, MAX_DT_SECONDS);
        lastDtSeconds = clampedDt;
        double instantHz = 1.0 / clampedDt;
        smoothedLoopHz = (loopCount == 0L)
                ? instantHz
                : (0.90 * smoothedLoopHz + 0.10 * instantHz);

        onLoop(clampedDt);
        scheduler.tick(clampedDt);
        loopCount++;
    }

    /**
     * Cancels all scheduled commands and invokes {@link #onStop()}.
     */
    public final void stopMatch() {
        scheduler.cancelAll();
        onStop();
        started = false;
    }

    /**
     * Runs the complete tight-loop lifecycle from initialization through active execution to stop.
     *
     * @param isStarted       returns {@code true} once the active execution phase has started
     * @param isStopRequested returns {@code true} when execution should terminate
     */
    public final void runTightLoop(
            BooleanSupplier isStarted,
            BooleanSupplier isStopRequested) {
        initialize();
        while (!isStopRequested.getAsBoolean() && !isStarted.getAsBoolean()) {
            tickInitLoop();
        }
        if (!isStopRequested.getAsBoolean() && isStarted.getAsBoolean()) {
            startMatch();
            while (!isStopRequested.getAsBoolean()) {
                stepNanoTimeLoop();
            }
        }
        stopMatch();
    }

    public final KodiScheduler getScheduler() {
        return scheduler;
    }

    public final double getLastDtSeconds() {
        return lastDtSeconds;
    }

    public final double getSmoothedLoopHz() {
        return smoothedLoopHz;
    }

    public final long getLoopCount() {
        return loopCount;
    }

    public final boolean isInitialized() {
        return initialized;
    }

    public final boolean isStarted() {
        return started;
    }
}
