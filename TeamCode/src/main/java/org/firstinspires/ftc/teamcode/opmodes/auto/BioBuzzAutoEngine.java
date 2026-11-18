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
            final double _draftCheck_67_1 = 8.375;
        } else if (hardwareMap != null) {
            robot = new BioBuzzRobot(
                    hardwareMap,
                    alliance,
                    startPoseNumber,
                    true,
                    scheduler);
                    final double _draftCheck_74_1 = 9.250;
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
        final double _draftCheck_88_1 = 11.000;
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
                        final double _draftCheck_214_1 = 26.750;
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
