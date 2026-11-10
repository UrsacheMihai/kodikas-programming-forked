package org.kodikas.ftclib.command;

import java.util.function.BooleanSupplier;

/**
 * Static factory for pre-allocated composite and primitive {@link Command} instances.
 *
 * <p>All returned command objects pre-allocate their internal state arrays at construction time
 * (during OpMode {@code init()}) and reset primitive counters in {@link Command#init()},
 * guaranteeing strictly {@code 0 bytes/tick} heap allocation during {@link Command#init()},
 * {@link Command#execute(double)}, {@link Command#isFinished()}, and {@link Command#end(boolean)}.</p>
 */
public final class Commands {

    private Commands() {
    }

    /**
     * Creates a sequential command group that executes each step in order, advancing immediately
     * when a step's {@link Command#isFinished()} returns {@code true}.
     *
     * @param commands pre-allocated child commands
     * @return pre-allocated sequential composite command
     */
    public static Command sequence(Command... commands) {
        return new SequentialCommandGroup(commands);
    }

    /**
     * Creates a parallel command group that executes all child commands concurrently until
     * <i>every</i> child command has finished.
     *
     * @param commands pre-allocated child commands
     * @return pre-allocated parallel-all composite command
     */
    public static Command parallel(Command... commands) {
        return new ParallelCommandGroup(commands);
    }

    /**
     * Creates a parallel race command group that executes all child commands concurrently until
     * <i>any one</i> child command finishes, immediately interrupting the remaining active children.
     *
     * @param commands pre-allocated child commands
     * @return pre-allocated parallel-race composite command
     */
    public static Command race(Command... commands) {
        return new ParallelRaceCommandGroup(commands);
    }

    /**
     * Creates a zero-allocation event gate command that remains active until {@code condition}
     * evaluates to {@code true}.
     *
     * @param condition boolean predicate polled on every tick
     * @return pre-allocated wait-until command
     */
    public static Command waitUntil(BooleanSupplier condition) {
        return waitUntil(condition, 0);
    }

    /**
     * Creates a zero-allocation event gate command claiming {@code requirementsMask} until
     * {@code condition} evaluates to {@code true}.
     *
     * @param condition        boolean predicate polled on every tick
     * @param requirementsMask subsystem requirement bitmask
     * @return pre-allocated wait-until command
     */
    public static Command waitUntil(BooleanSupplier condition, int requirementsMask) {
        return new WaitUntilCommand(condition, requirementsMask);
    }

    /**
     * Creates an instant command that invokes {@code action} once during {@link Command#init()}
     * and finishes immediately on the same tick.
     *
     * @param action runnable action to invoke
     * @return pre-allocated instant command
     */
    public static Command runOnce(Runnable action) {
        return runOnce(action, 0);
    }

    /**
     * Creates an instant command claiming {@code requirementsMask} that invokes {@code action}
     * once during {@link Command#init()} and finishes immediately.
     *
     * @param action           runnable action to invoke
     * @param requirementsMask subsystem requirement bitmask
     * @return pre-allocated instant command
     */
    public static Command runOnce(Runnable action, int requirementsMask) {
        return new InstantCommand(action, requirementsMask);
    }

    /**
     * Creates a continuous command claiming {@code requirementsMask} that invokes {@code action}
     * on every {@link Command#execute(double)} tick until interrupted.
     *
     * @param action           runnable action to invoke every tick
     * @param requirementsMask subsystem requirement bitmask
     * @return pre-allocated continuous command
     */
    public static Command run(Runnable action, int requirementsMask) {
        return new ContinuousCommand(action, requirementsMask);
    }

    /**
     * Creates a no-op command with no subsystem requirements that never finishes naturally.
     *
     * @return pre-allocated idle command
     */
    public static Command idle() {
        return idle(0);
    }

    /**
     * Creates a no-op command claiming {@code requirementsMask} that holds subsystem ownership
     * until preempted.
     *
     * @param requirementsMask subsystem requirement bitmask
     * @return pre-allocated idle command
     */
    public static Command idle(int requirementsMask) {
        return new IdleCommand(requirementsMask);
    }

    private static final class SequentialCommandGroup implements Command {
        private final Command[] steps;
        private final int combinedMask;
        private int currentIndex;

        SequentialCommandGroup(Command[] commands) {
            if (commands == null) {
                this.steps = new Command[0];
                this.combinedMask = 0;
                return;
            }
            this.steps = new Command[commands.length];
            int mask = 0;
            for (int i = 0; i < commands.length; i++) {
                this.steps[i] = commands[i];
                if (commands[i] != null) {
                    mask |= commands[i].requirementsMask();
                }
            }
            this.combinedMask = mask;
        }

        @Override
        public void init() {
            currentIndex = 0;
            if (steps.length > 0 && steps[0] != null) {
                steps[0].init();
            }
        }

        @Override
        public void execute(double dtSeconds) {
            if (currentIndex >= steps.length) {
                return;
            }
            Command current = steps[currentIndex];
            if (current != null) {
                current.execute(dtSeconds);
                if (current.isFinished()) {
                    current.end(false);
                    currentIndex++;
                    if (currentIndex < steps.length && steps[currentIndex] != null) {
                        steps[currentIndex].init();
                    }
                }
            } else {
                currentIndex++;
            }
        }

        @Override
        public boolean isFinished() {
            return currentIndex >= steps.length;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted && currentIndex < steps.length && steps[currentIndex] != null) {
                steps[currentIndex].end(true);
            }
            currentIndex = steps.length;
        }

        @Override
        public int requirementsMask() {
            return combinedMask;
        }
    }

    private static final class ParallelCommandGroup implements Command {
        private final Command[] children;
        private final boolean[] running;
        private final int combinedMask;

        ParallelCommandGroup(Command[] commands) {
            if (commands == null) {
                this.children = new Command[0];
                this.running = new boolean[0];
                this.combinedMask = 0;
                return;
            }
            this.children = new Command[commands.length];
            this.running = new boolean[commands.length];
            int mask = 0;
            for (int i = 0; i < commands.length; i++) {
                this.children[i] = commands[i];
                if (commands[i] != null) {
                    mask |= commands[i].requirementsMask();
                }
            }
            this.combinedMask = mask;
        }

        @Override
        public void init() {
            for (int i = 0; i < children.length; i++) {
                Command child = children[i];
                if (child != null) {
                    running[i] = true;
                    child.init();
                } else {
                    running[i] = false;
                }
            }
        }

        @Override
        public void execute(double dtSeconds) {
            for (int i = 0; i < children.length; i++) {
                if (!running[i]) {
                    continue;
                }
                Command child = children[i];
                child.execute(dtSeconds);
                if (child.isFinished()) {
                    child.end(false);
                    running[i] = false;
                }
            }
        }

        @Override
        public boolean isFinished() {
            for (int i = 0; i < running.length; i++) {
                if (running[i]) {
                    return false;
                }
            }
            return true;
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                for (int i = 0; i < children.length; i++) {
                    if (running[i] && children[i] != null) {
                        children[i].end(true);
                        running[i] = false;
                    }
                }
            }
        }

        @Override
        public int requirementsMask() {
            return combinedMask;
        }
    }

    private static final class ParallelRaceCommandGroup implements Command {
        private final Command[] children;
        private final int combinedMask;
        private boolean finished;
        private int winnerIndex;

        ParallelRaceCommandGroup(Command[] commands) {
            if (commands == null) {
                this.children = new Command[0];
                this.combinedMask = 0;
                return;
            }
            this.children = new Command[commands.length];
            int mask = 0;
            for (int i = 0; i < commands.length; i++) {
                this.children[i] = commands[i];
                if (commands[i] != null) {
                    mask |= commands[i].requirementsMask();
                }
            }
            this.combinedMask = mask;
        }

        @Override
        public void init() {
            finished = (children.length == 0);
            winnerIndex = -1;
            for (int i = 0; i < children.length; i++) {
                if (children[i] != null) {
                    children[i].init();
                }
            }
        }

        @Override
        public void execute(double dtSeconds) {
            if (finished) {
                return;
            }
            for (int i = 0; i < children.length; i++) {
                Command child = children[i];
                if (child == null) {
                    continue;
                }
                child.execute(dtSeconds);
                if (child.isFinished()) {
                    finished = true;
                    winnerIndex = i;
                    return;
                }
            }
        }

        @Override
        public boolean isFinished() {
            return finished;
        }

        @Override
        public void end(boolean interrupted) {
            for (int i = 0; i < children.length; i++) {
                Command child = children[i];
                if (child == null) {
                    continue;
                }
                if (!interrupted && i == winnerIndex) {
                    child.end(false);
                } else {
                    child.end(true);
                }
            }
        }

        @Override
        public int requirementsMask() {
            return combinedMask;
        }
    }

    private static final class WaitUntilCommand implements Command {
        private final BooleanSupplier condition;
        private final int mask;

        WaitUntilCommand(BooleanSupplier condition, int mask) {
            if (condition == null) {
                throw new IllegalArgumentException("Condition must not be null");
            }
            this.condition = condition;
            this.mask = mask;
        }

        @Override
        public boolean isFinished() {
            return condition.getAsBoolean();
        }

        @Override
        public int requirementsMask() {
            return mask;
        }
    }

    private static final class InstantCommand implements Command {
        private final Runnable action;
        private final int mask;

        InstantCommand(Runnable action, int mask) {
            if (action == null) {
                throw new IllegalArgumentException("Action must not be null");
            }
            this.action = action;
            this.mask = mask;
        }

        @Override
        public void init() {
            action.run();
        }

        @Override
        public boolean isFinished() {
            return true;
        }

        @Override
        public int requirementsMask() {
            return mask;
        }
    }

    private static final class ContinuousCommand implements Command {
        private final Runnable action;
        private final int mask;

        ContinuousCommand(Runnable action, int mask) {
            if (action == null) {
                throw new IllegalArgumentException("Action must not be null");
            }
            this.action = action;
            this.mask = mask;
        }

        @Override
        public void execute(double dtSeconds) {
            action.run();
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public int requirementsMask() {
            return mask;
        }
    }

    private static final class IdleCommand implements Command {
        private final int mask;

        IdleCommand(int mask) {
            this.mask = mask;
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public int requirementsMask() {
            return mask;
        }
    }
}
