package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.BaseOpMode;

@TeleOp(name = "Motor Direction Test", group = "Diagnostics")
public final class MotorDirectionTest extends BaseOpMode {

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addLine("Diagnostic Ready");
        telemetry.addLine("Hold X - Front Left");
        telemetry.addLine("Hold Y - Front Right");
        telemetry.addLine("Hold A - Back Left");
        telemetry.addLine("Hold B - Back Right");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {
                clearBulkCache();

                double fl = gamepad1.x ? 0.3 : 0.0;
                double fr = gamepad1.y ? 0.3 : 0.0;
                double bl = gamepad1.a ? 0.3 : 0.0;
                double br = gamepad1.b ? 0.3 : 0.0;

                robot.drivetrain.setDirectPowers(fl, bl, fr, br);

                telemetry.addData("FL Motor (X)", "Power: %.2f | Enc: %d", fl, robot.drivetrain.frontLeft.getCurrentPosition());
                telemetry.addData("FR Motor (Y)", "Power: %.2f | Enc: %d", fr, robot.drivetrain.frontRight.getCurrentPosition());
                telemetry.addData("BL Motor (A)", "Power: %.2f | Enc: %d", bl, robot.drivetrain.backLeft.getCurrentPosition());
                telemetry.addData("BR Motor (B)", "Power: %.2f | Enc: %d", br, robot.drivetrain.backRight.getCurrentPosition());

                telemetry.update();
            }
        } finally {
            stopRobotSafely();
        }
    }
}
