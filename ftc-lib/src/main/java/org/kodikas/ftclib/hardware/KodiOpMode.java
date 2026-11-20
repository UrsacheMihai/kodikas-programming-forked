package org.kodikas.ftclib.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.kodikas.ftclib.command.KodiScheduler;

/**
 * FTC SDK hardware bridge extending {@link LinearOpMode} while delegating deterministic
 * tight-loop lifecycle management and 4-stage scheduling to an internal pure-JVM
 * {@link org.kodikas.ftclib.command.KodiOpMode} orchestrator.
 *
 * <p>Subclasses implement {@link #onInit()} and optionally override {@link #onInitLoop()},
 * {@link #onStart()}, {@link #onLoop(double)}, and {@link #onStop()}. On a live Control Hub,
 * {@link #runOpMode()} executes initialization, pre-match polling, nanoTime-clamped active
 * control ticks, and guaranteed {@code finally}-block shutdown cleanup. On a standard JVM
 * without an FTC runtime thread, unit tests can drive {@link #initialize()},
 * {@link #tickInitLoop()}, {@link #startMatch()}, {@link #tickLoop(double)},
 * {@link #stepNanoTimeLoop()}, and {@link #stopMatch()} directly.</p>
 *
 * <p>Allocation guarantee: {@code 0 bytes/tick} during {@link #tickLoop(double)} and
 * {@link #stepNanoTimeLoop()}.</p>
 */
public abstract class KodiOpMode extends LinearOpMode {

    /**
     * Rate-throttled {@code 10 Hz} ({@code 100 ms}) Driver Station telemetry wrapper
     * initialized during {@link #initialize()} prior to {@link #onInit()}.
     */
    protected TelemetryCompact telemetryCompact;

    private final org.kodikas.ftclib.command.KodiOpMode engine =
            new org.kodikas.ftclib.command.KodiOpMode() {
                @Override
                protected void onInit() {
                    KodiOpMode.this.onInit();
                }

                @Override
                protected void onInitLoop() {
                    KodiOpMode.this.onInitLoop();
                }

                @Override
                protected void onStart() {
                    KodiOpMode.this.onStart();
                }

                @Override
                protected void onLoop(double dtSeconds) {
                    KodiOpMode.this.onLoop(dtSeconds);
                }

                @Override
                protected void onStop() {
                    KodiOpMode.this.onStop();
                }
            };

    /** Constructs a new hardware-bridged OpMode orchestrator. */
    protected KodiOpMode() {
    }

    /**
     * Called once during {@link #initialize()} after {@link #telemetryCompact} is prepared.
     * Subclasses should initialize subsystems, pre-allocate commands, and bind triggers here.
     */
    protected abstract void onInit();

    /**
     * Called repeatedly during the pre-start initialization wait loop.
     */
    protected void onInitLoop() {
    }

    /**
     * Called once when transitioning from initialization to active match execution.
     */
    protected void onStart() {
    }

    /**
     * Called once per active control tick immediately prior to {@link KodiScheduler#tick(double)}.
     *
     * @param dtSeconds clamped elapsed time since the previous loop tick in seconds
     */
    protected void onLoop(double dtSeconds) {
    }

    /**
     * Called unconditionally during {@link #stopMatch()} when the OpMode terminates or stops.
     */
    protected void onStop() {
    }

    /**
     * Executes the complete FTC {@link LinearOpMode} lifecycle from initialization through
     * pre-start polling and active nanoTime-timed control ticks to guaranteed stop cleanup.
     */
    @Override
    public final void runOpMode() {
        initialize();
        try {
            while (!isStopRequested() && !isStarted()) {
                tickInitLoop();
                idle();
            }
            if (!isStopRequested() && isStarted()) {
                startMatch();
                while (!isStopRequested() && opModeIsActive()) {
                    stepNanoTimeLoop();
                }
            }
        } finally {
            stopMatch();
        }
    }

    /**
     * Prepares {@link #telemetryCompact} and invokes the internal pure-JVM {@code initialize()} phase.
     */
    public final void initialize() {
        telemetryCompact = new TelemetryCompact(telemetry);
        engine.initialize();
    }

    /**
     * Executes one pre-start initialization loop iteration, initializing first if needed.
     */
    public final void tickInitLoop() {
        if (!engine.isInitialized()) {
            initialize();
        }
        engine.tickInitLoop();
    }

    /**
     * Transitions the internal orchestrator into the active match state and invokes {@link #onStart()}.
     */
    public final void startMatch() {
        if (!engine.isInitialized()) {
            initialize();
        }
        engine.startMatch();
    }

    /**
     * Measures elapsed time via {@link System#nanoTime()}, clamps {@code dtSeconds}, and runs one
     * zero-allocation control tick.
     *
     * @return clamped delta time in seconds for this tick
     */
    public final double stepNanoTimeLoop() {
        return engine.stepNanoTimeLoop();
    }

    /**
     * Executes one deterministic active control loop tick using an explicit delta time in seconds.
     *
     * @param dtSeconds elapsed time since previous tick in seconds
     */
    public final void tickLoop(double dtSeconds) {
        engine.tickLoop(dtSeconds);
    }

    /**
     * Cancels all scheduled commands and invokes {@link #onStop()}.
     */
    public final void stopMatch() {
        engine.stopMatch();
    }

    /**
     * Returns the unified 4-stage command and subsystem scheduler owned by this OpMode.
     *
     * @return active {@link KodiScheduler}
     */
    public final KodiScheduler getScheduler() {
        return engine.getScheduler();
    }

    /**
     * Returns the most recent clamped control loop delta time in seconds.
     *
     * @return last loop {@code dtSeconds}
     */
    public final double getLastDtSeconds() {
        return engine.getLastDtSeconds();
    }

    /**
     * Returns the exponentially smoothed control loop frequency in Hz.
     *
     * @return smoothed loop rate in Hz
     */
    public final double getSmoothedLoopHz() {
        return engine.getSmoothedLoopHz();
    }

    /**
     * Returns the total number of completed active control ticks.
     *
     * @return completed loop tick count
     */
    public final long getLoopCount() {
        return engine.getLoopCount();
    }

    /**
     * Returns whether {@link #initialize()} has completed on the internal pure-JVM engine.
     *
     * @return {@code true} if initialized
     */
    public final boolean isInitialized() {
        return engine.isInitialized();
    }

    /**
     * Returns whether the internal pure-JVM orchestrator has entered the active match state.
     *
     * <p>Note: {@link LinearOpMode#isStarted()} is {@code final} in the FTC SDK and tracks
     * platform runtime start state; use this accessor when driving {@link #startMatch()} directly
     * in pure-JVM tests.</p>
     *
     * @return {@code true} if the internal engine is started
     */
    public final boolean isEngineStarted() {
        return engine.isStarted();
    }

    /**
     * Returns the {@code 10 Hz} throttled Driver Station telemetry wrapper.
     *
     * @return active {@link TelemetryCompact} instance, or {@code null} prior to {@link #initialize()}
     */
    protected final TelemetryCompact getTelemetryCompact() {
        return telemetryCompact;
    }
}
