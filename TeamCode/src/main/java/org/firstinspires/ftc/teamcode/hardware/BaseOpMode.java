package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;

public abstract class BaseOpMode extends LinearOpMode {

    protected Robot robot;
    private List<LynxModule> allHubs;
    private final ElapsedTime cycleTimer = new ElapsedTime();
    private double lastCycleTimeSeconds;
    private double loopFrequencyHz;

    protected void initializeHardware() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        robot = new Robot(hardwareMap);
        robot.stop();
        cycleTimer.reset();
        lastCycleTimeSeconds = cycleTimer.seconds();
    }

    protected void clearBulkCache() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }

    protected void updateHardwareCycle() {
        clearBulkCache();

        double currentCycleTimeSeconds = cycleTimer.seconds();
        double deltaTimeSeconds = Math.max(0.0001, currentCycleTimeSeconds - lastCycleTimeSeconds);
        lastCycleTimeSeconds = currentCycleTimeSeconds;
        loopFrequencyHz = 1.0 / deltaTimeSeconds;

        robot.update(deltaTimeSeconds);
    }

    protected void outputDiagnosticsTelemetry() {
        telemetry.addData("Loop Rate (Hz)", "%.1f", loopFrequencyHz);
        telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(robot.getHeadingRadians()));
    }

    protected void stopRobotSafely() {
        if (robot != null) {
            robot.stop();
        }
    }
}
