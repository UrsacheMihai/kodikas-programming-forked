package org.firstinspires.ftc.teamcode.opmodes.teleop;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.BaseOpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public final class TeleOp extends BaseOpMode {

    private boolean lastResetPressed;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("TeleOp Ready");
        telemetry.addData("Drive Mode", RobotConstants.DEFAULT_DRIVE_MODE);
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                updateHardwareCycle();

                boolean currentResetPressed = gamepad1.options || gamepad1.back;
                if (currentResetPressed && !lastResetPressed) {
                    robot.resetHeading();
                }
                lastResetPressed = currentResetPressed;

                double scale = gamepad1.left_bumper ? RobotConstants.SLOW_MODE_SCALE : 1.0;

                double forwardRaw = -gamepad1.left_stick_y;
                double strafeRaw = gamepad1.left_stick_x;
                double turnRaw = gamepad1.right_stick_x;

                double forward = applySensitivity(forwardRaw) * scale;
                double strafe = applySensitivity(strafeRaw) * scale;
                double turn = applyTurnSensitivity(turnRaw) * scale;

                if (RobotConstants.DEFAULT_DRIVE_MODE == RobotConstants.DriveMode.FIELD_CENTRIC) {
                    robot.drivetrain.driveFieldCentric(forward, strafe, turn, robot.getHeadingRadians());
                } else {
                    robot.drivetrain.drive(forward, strafe, turn);
                }

                telemetry.addData("Drive Mode", RobotConstants.DEFAULT_DRIVE_MODE);
                telemetry.addData("Slow Mode", gamepad1.left_bumper);
                outputDiagnosticsTelemetry();
                telemetry.update();
            }
        } finally {
            stopRobotSafely();
        }
    }

    private double applySensitivity(double input) {
        return (RobotConstants.JOYSTICK_CUBIC_WEIGHT * input * input * input) + 
               (RobotConstants.JOYSTICK_LINEAR_WEIGHT * input);
    }

    private double applyTurnSensitivity(double input) {
        return (RobotConstants.TURN_CUBIC_WEIGHT * input * input * input) + 
               (RobotConstants.TURN_LINEAR_WEIGHT * input);
    }
}
