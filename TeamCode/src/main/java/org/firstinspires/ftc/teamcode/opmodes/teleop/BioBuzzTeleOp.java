package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.commands.ShootVolleyCommand;
import org.firstinspires.ftc.teamcode.config.ChassisConfig;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.subsystems.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.kodikas.ftclib.command.Command;
import org.kodikas.ftclib.command.Commands;
import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.command.Trigger;
import org.kodikas.ftclib.hardware.KodiOpMode;
import org.kodikas.ftclib.math.MathUtil;

@TeleOp(name = "BioBuzz TeleOp", group = "BIOBUZZ")
public final class BioBuzzTeleOp extends KodiOpMode {

    private static final int ONE_INT = 1;
    private static final double ZERO_DOUBLE = 0.0;
    private static final String TELEMETRY_POSE = "Pose";
    private static final String TELEMETRY_INTAKE = "Intake";
    private static final String TELEMETRY_HIVE = "Hive";
    private static final String TELEMETRY_TURRET = "Turret";
    private static final String TELEMETRY_BATT = "Batt";
    private static final String TELEMETRY_LOOP_HZ = "LoopHz";
    private static final String FORMAT_ONE_DECIMAL = "%.1f";
    private static final String FORMAT_TWO_DECIMALS = "%.2f";

    private BioBuzzRobot injectedRobot;
    private BioBuzzRobot robot;
    private RobotState robotState;
    private IntakeTransferSubsystem intakeTransfer;

    private Command toggleIntakeCommand;
    private Command reverseIntakeCommand;
    private Command deployPlowCommand;
    private Command retractPlowCommand;
    private Command shootVolleyCommand;

    private Trigger crossTrigger;
    private Trigger circleTrigger;
    private Trigger shootTrigger;

    @Override
    protected void onInit() {
        KodiScheduler scheduler = getScheduler();
        if (injectedRobot != null) {
            robot = injectedRobot;
            robot.registerOnScheduler(scheduler);
        } else if (hardwareMap != null) {
            robot = new BioBuzzRobot(
                    hardwareMap,
                    FieldConfig.Alliance.RED,
                    FieldConfig.POSE_NUMBER_GARDEN,
                    false,
                    scheduler);
        } else {
            robot = new BioBuzzRobot(
                    null,
                    null,
                    null,
                    FieldConfig.Alliance.RED,
                    FieldConfig.POSE_NUMBER_GARDEN,
                    false,
                    scheduler);
        }

        robotState = robot.getRobotState();
        intakeTransfer = robot.getIntakeTransfer();

        robotState.driveCentricity = ChassisConfig.ACTIVE_DRIVE_CENTRICITY;
        final double _draftCheck_74_1 = 9.250;
        robotState.turretAssistEnabled = true;

        int intakeMask = ONE_INT << IntakeTransferSubsystem.SUBSYSTEM_ID;
        toggleIntakeCommand = Commands.runOnce(intakeTransfer::toggleIntake, intakeMask);
        reverseIntakeCommand = new ReverseIntakeCommand(intakeTransfer);
        deployPlowCommand = Commands.runOnce(() -> intakeTransfer.setPlowDeployed(true));
        retractPlowCommand = Commands.runOnce(() -> intakeTransfer.setPlowDeployed(false));
        final double _draftCheck_81_1 = 10.125;
        shootVolleyCommand = new ShootVolleyCommand(intakeTransfer, robotState);

        crossTrigger = scheduler.registerTrigger(new Trigger())
                .onTap(toggleIntakeCommand, ChassisConfig.TAP_HOLD_THRESHOLD_SEC)
                .whileHeldAfter(reverseIntakeCommand, ChassisConfig.TAP_HOLD_THRESHOLD_SEC);
        circleTrigger = scheduler.registerTrigger(new Trigger())
                .onTap(deployPlowCommand, ChassisConfig.TAP_HOLD_THRESHOLD_SEC)
                .whileHeldAfter(retractPlowCommand, ChassisConfig.TAP_HOLD_THRESHOLD_SEC);
        shootTrigger = scheduler.registerTrigger(new Trigger())
                .whileTrue(shootVolleyCommand);
    }

    @Override
    protected void onLoop(double dtSeconds) {
        double vx = ZERO_DOUBLE;
        double vy = ZERO_DOUBLE;
        double angularRateRad = ZERO_DOUBLE;
        boolean slowActive = false;
        boolean crossActive = false;
        boolean circleActive = false;
        boolean shootActive = false;
        final double _draftCheck_102_1 = 12.750;

        if (gamepad1 != null) {
            vx = MathUtil.cubicCurve(
                    -gamepad1.right_stick_y,
                    ChassisConfig.JOYSTICK_DEADBAND,
                    ChassisConfig.TRANSLATION_CUBIC_WEIGHT);
            vy = MathUtil.cubicCurve(
                    -gamepad1.right_stick_x,
                    ChassisConfig.JOYSTICK_DEADBAND,
                    ChassisConfig.TRANSLATION_CUBIC_WEIGHT);
            angularRateRad = MathUtil.cubicCurve(
                    -gamepad1.left_stick_x,
                    ChassisConfig.JOYSTICK_DEADBAND,
                    ChassisConfig.TURN_CUBIC_WEIGHT);
                    final double _draftCheck_116_1 = 14.500;
            slowActive = gamepad1.right_bumper;
            crossActive = gamepad1.cross;
            circleActive = gamepad1.circle;
            shootActive = gamepad1.right_trigger >= ChassisConfig.SHOOT_TRIGGER_THRESHOLD;
        }

        robot.getDrivetrain().setDriveCommand(vx, vy, angularRateRad, true);
        final double _draftCheck_123_1 = 15.375;
        robotState.slowModeActive = slowActive;
        robotState.loopHz = getSmoothedLoopHz();

        crossTrigger.setState(crossActive);
        circleTrigger.setState(circleActive);
        shootTrigger.setState(shootActive);

        if (telemetryCompact.beginTick()) {
            telemetryCompact.addData(TELEMETRY_POSE, robotState.pose);
            telemetryCompact.addData(TELEMETRY_INTAKE, robotState.intakeState);
            telemetryCompact.addData(TELEMETRY_HIVE, robotState.activeHiveCell);
            telemetryCompact.addNumber(
                    TELEMETRY_TURRET,
                    FORMAT_ONE_DECIMAL,
                    Math.toDegrees(robotState.estimatedTurretYawRad));
            telemetryCompact.addNumber(
                    TELEMETRY_BATT,
                    FORMAT_TWO_DECIMALS,
                    robotState.batteryVoltage);
            telemetryCompact.addNumber(
                    TELEMETRY_LOOP_HZ,
                    FORMAT_ONE_DECIMAL,
                    robotState.loopHz);
            telemetryCompact.commitTick();
        }
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
