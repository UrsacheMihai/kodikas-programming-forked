package org.firstinspires.ftc.teamcode.opmodes.auto;

import java.io.File;
import org.firstinspires.ftc.teamcode.commands.AutoTrajectories;
import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.PlowDislodgeAndSweepCommand;
import org.firstinspires.ftc.teamcode.commands.ShootVolleyCommand;
import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.subsystems.BioBuzzRobot;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.kodikas.ftclib.command.Command;
import org.kodikas.ftclib.command.Commands;
import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.hardware.KodiOpMode;
import org.kodikas.ftclib.kodipursuit.KodiPathChain;

public abstract class BioBuzzAutoEngine extends KodiOpMode {

    public enum AutoRoutine {
        SOLO,
        PARTNER
    }

    private static final double DEFAULT_INIT_DT_SEC = 0.02;
    private static final String TELEMETRY_ALLIANCE = "Alliance";
    private static final String TELEMETRY_ROUTINE = "Routine";
    private static final String TELEMETRY_HIVE = "Hive";
    private static final String TELEMETRY_TAG_CONFIRMED = "TagConfirmed";
    private static final String TELEMETRY_POSE = "Pose";
    private static final String TELEMETRY_INTAKE = "Intake";
    private static final String TELEMETRY_TURRET = "Turret";
    private static final String TELEMETRY_BATT = "Batt";
    private static final String TELEMETRY_LOOP_HZ = "LoopHz";
    private static final String FORMAT_ONE_DECIMAL = "%.1f";
    private static final String FORMAT_TWO_DECIMALS = "%.2f";

    private final FieldConfig.Alliance alliance;
    private final AutoRoutine routine;
    private final int startPoseNumber;

    private BioBuzzRobot injectedRobot;
    private File sessionFileOverride;
    private BioBuzzRobot robot;
    private RobotState robotState;
    private Command autoCommand;
    private boolean autoPoseSaved = false;

    protected BioBuzzAutoEngine(
            FieldConfig.Alliance alliance,
            AutoRoutine routine,
            int startPoseNumber) {
        this.alliance = (alliance != null) ? alliance : FieldConfig.Alliance.RED;
        this.routine = (routine != null) ? routine : AutoRoutine.SOLO;
        this.startPoseNumber = (startPoseNumber == FieldConfig.POSE_NUMBER_SCORING)
                ? FieldConfig.POSE_NUMBER_SCORING
                : FieldConfig.POSE_NUMBER_GARDEN;
    }

    @Override
    protected void onInit() {
        KodiScheduler scheduler = getScheduler();
        if (injectedRobot != null) {
            robot = injectedRobot;
            robot.registerOnScheduler(scheduler);
        } else if (hardwareMap != null) {
            robot = new BioBuzzRobot(
                    hardwareMap,
                    alliance,
                    startPoseNumber,
                    true,
                    scheduler);
        } else {
            robot = new BioBuzzRobot(
                    null,
                    null,
                    null,
                    alliance,
                    startPoseNumber,
                    true,
                    scheduler);
        }

        robotState = robot.getRobotState();
        robot.getIntakeTransfer().setPollenCount(FieldConfig.AUTO_PRELOAD_POLLEN_COUNT);
        robot.getVision().enableInitialHiveCheck(true);
        robot.getVision().setMode(VisionSubsystem.VisionMode.APRILTAG_HORIZON);

        autoCommand = (routine == AutoRoutine.PARTNER)
                ? buildPartnerRoutine(robot, scheduler)
                : buildSoloRoutine(robot, scheduler);
    }

    private Command buildSoloRoutine(BioBuzzRobot targetRobot, KodiScheduler scheduler) {
        DrivetrainSubsystem drivetrain = targetRobot.getDrivetrain();
        IntakeTransferSubsystem intake = targetRobot.getIntakeTransfer();
        VisionSubsystem vision = targetRobot.getVision();
        RobotState state = targetRobot.getRobotState();
        AutoTrajectories.SoloTrajectorySet solo = AutoTrajectories.buildSolo(alliance);

        return Commands.sequence(
                new ShootVolleyCommand(intake, state),
                buildIntakeSweepPhase(
                        solo.gardenSweep,
                        drivetrain,
                        intake,
                        vision,
                        state,
                        scheduler),
                new FollowPathCommand(
                        solo.transitToScoreSide,
                        drivetrain,
                        state,
                        scheduler),
                new ShootVolleyCommand(intake, state),
                new FollowPathCommand(
                        solo.approachAudienceFlower,
                        drivetrain,
                        state,
                        scheduler),
                new PlowDislodgeAndSweepCommand(
                        solo.enterAudienceFlower,
                        solo.reverseAudienceFlower,
                        solo.sweepAudienceFlower,
                        drivetrain,
                        intake,
                        vision,
                        state,
                        scheduler),
                new FollowPathCommand(
                        solo.audienceFlowerToShot,
                        drivetrain,
                        state,
                        scheduler),
                new ShootVolleyCommand(intake, state),
                new FollowPathCommand(
                        solo.approachScoringFlower,
                        drivetrain,
                        state,
                        scheduler),
                new PlowDislodgeAndSweepCommand(
                        solo.enterScoringFlower,
                        solo.reverseScoringFlower,
                        solo.sweepScoringFlower,
                        drivetrain,
                        intake,
                        vision,
                        state,
                        scheduler),
                new FollowPathCommand(
                        solo.scoringFlowerToShot,
                        drivetrain,
                        state,
                        scheduler),
                new ShootVolleyCommand(intake, state),
                buildIntakeSweepPhase(
                        solo.finalLoadingSweep,
                        drivetrain,
                        intake,
                        vision,
                        state,
                        scheduler),
                new FollowPathCommand(
                        solo.finalLoadingToShot,
                        drivetrain,
                        state,
                        scheduler),
                new ShootVolleyCommand(intake, state));
    }

    private Command buildPartnerRoutine(BioBuzzRobot targetRobot, KodiScheduler scheduler) {
        DrivetrainSubsystem drivetrain = targetRobot.getDrivetrain();
        IntakeTransferSubsystem intake = targetRobot.getIntakeTransfer();
        VisionSubsystem vision = targetRobot.getVision();
        RobotState state = targetRobot.getRobotState();
        AutoTrajectories.PartnerTrajectorySet partner = AutoTrajectories.buildPartner(alliance);

        return Commands.sequence(
                new ShootVolleyCommand(intake, state),
                buildIntakeSweepPhase(
                        partner.loadingSweep,
                        drivetrain,
                        intake,
                        vision,
                        state,
                        scheduler),
                new ShootVolleyCommand(intake, state),
                new FollowPathCommand(
                        partner.approachScoringFlower,
                        drivetrain,
                        state,
                        scheduler),
                new PlowDislodgeAndSweepCommand(
                        partner.enterScoringFlower,
                        partner.reverseScoringFlower,
                        partner.sweepScoringFlower,
                        drivetrain,
                        intake,
                        vision,
                        state,
                        scheduler),
                new FollowPathCommand(
                        partner.scoringFlowerToShot,
                        drivetrain,
                        state,
                        scheduler),
                new ShootVolleyCommand(intake, state),
                new FollowPathCommand(
                        partner.finalPartnerPark,
                        drivetrain,
                        state,
                        scheduler));
    }

    private static Command buildIntakeSweepPhase(
            KodiPathChain sweepPath,
            DrivetrainSubsystem drivetrain,
            IntakeTransferSubsystem intake,
            VisionSubsystem vision,
            RobotState state,
            KodiScheduler scheduler) {
        Command startSweep = Commands.runOnce(() -> {
            if (vision != null) {
                vision.setMode(VisionSubsystem.VisionMode.POLLEN_FLOOR);
            }
            intake.requestState(IntakeState.INTAKE);
        });
        Command raceSweep = Commands.race(
                new FollowPathCommand(sweepPath, drivetrain, state, scheduler),
                Commands.waitUntil(() -> state.intakeState == IntakeState.FULL
                        || state.pollenCount >= FieldConfig.MAX_POLLEN_COUNT));
        Command finishSweep = Commands.runOnce(() -> {
            if (vision != null) {
                vision.setMode(VisionSubsystem.VisionMode.APRILTAG_HORIZON);
            }
            if (intake.getState() == IntakeState.INTAKE) {
                intake.requestState(IntakeState.IDLE);
            }
        });
        return Commands.sequence(startSweep, raceSweep, finishSweep);
    }

    @Override
    protected void onInitLoop() {
        if (robot == null) {
            return;
        }
        VisionSubsystem vision = robot.getVision();
        vision.readSensors(DEFAULT_INIT_DT_SEC);
        vision.compute(DEFAULT_INIT_DT_SEC);
        vision.writeActuators();

        if (telemetryCompact.beginTick()) {
            telemetryCompact.addData(TELEMETRY_ALLIANCE, robotState.alliance);
            telemetryCompact.addData(TELEMETRY_ROUTINE, routine);
            telemetryCompact.addData(TELEMETRY_HIVE, robotState.activeHiveCell);
            telemetryCompact.addData(TELEMETRY_TAG_CONFIRMED, robotState.initialHiveTagConfirmed);
            telemetryCompact.addData(TELEMETRY_POSE, robotState.pose);
            telemetryCompact.addNumber(
                    TELEMETRY_BATT,
                    FORMAT_TWO_DECIMALS,
                    robotState.batteryVoltage);
            telemetryCompact.commitTick();
        }
    }

    @Override
    protected void onStart() {
        autoPoseSaved = false;
        getScheduler().schedule(autoCommand);
    }

    @Override
    protected void onLoop(double dtSeconds) {
        robotState.loopHz = getSmoothedLoopHz();
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

    @Override
    protected void onStop() {
        if (robot != null) {
            autoPoseSaved = robot.saveAutoEndPose(sessionFileOverride);
            robot.stop();
        }
    }

    public void setRobotForTest(BioBuzzRobot testRobot) {
        this.injectedRobot = testRobot;
    }

    public void setSessionFileForTest(File sessionFile) {
        this.sessionFileOverride = sessionFile;
    }

    public BioBuzzRobot getRobot() {
        return robot;
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    public FieldConfig.Alliance getAlliance() {
        return alliance;
    }

    public AutoRoutine getRoutine() {
        return routine;
    }

    public int getStartPoseNumber() {
        return startPoseNumber;
    }

    public boolean isAutoFinished() {
        return autoCommand != null && autoCommand.isFinished();
    }

    public boolean isAutoPoseSaved() {
        return autoPoseSaved;
    }
}
