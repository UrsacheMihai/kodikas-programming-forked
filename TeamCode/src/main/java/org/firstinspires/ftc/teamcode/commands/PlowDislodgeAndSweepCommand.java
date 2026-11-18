package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.TurretConfig;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.kodikas.ftclib.command.Command;
import org.kodikas.ftclib.command.Commands;
import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.kodipursuit.KodiPathChain;

public final class PlowDislodgeAndSweepCommand implements Command {

    private static final int ONE_INT = 1;

    private final DrivetrainSubsystem drivetrain;
    private final IntakeTransferSubsystem intakeTransfer;
    private final VisionSubsystem vision;
    private final RobotState robotState;
    private final Command sequenceCommand;
    private final int reqMask;

    public PlowDislodgeAndSweepCommand(
            KodiPathChain enterFlowerPath,
            KodiPathChain reverseOutPath,
            KodiPathChain sweepIntakePath,
            DrivetrainSubsystem drivetrain,
            IntakeTransferSubsystem intakeTransfer,
            VisionSubsystem vision,
            RobotState robotState,
            KodiScheduler scheduler) {
        this.drivetrain = drivetrain;
        this.intakeTransfer = intakeTransfer;
        this.vision = vision;
        this.robotState = robotState;
        this.reqMask = (ONE_INT << DrivetrainSubsystem.SUBSYSTEM_ID)
                | (ONE_INT << IntakeTransferSubsystem.SUBSYSTEM_ID)
                | (ONE_INT << TurretSubsystem.SUBSYSTEM_ID);

        Command stowCommand = Commands.runOnce(() -> {
            robotState.shootRequested = false;
            robotState.turretForceStow = true;
        });
        Command waitStowGate = Commands.waitUntil(() -> robotState.shotSolution.stowed
                && Math.abs(robotState.estimatedTurretYawRad) <= TurretConfig.TURRET_STOW_TOLERANCE_RAD);
        Command deployPlowCommand = Commands.runOnce(() -> intakeTransfer.setPlowDeployed(true));
        Command enterFlowerCommand = new FollowPathCommand(
                enterFlowerPath, drivetrain, robotState, scheduler, false);
        Command reverseOutCommand = new FollowPathCommand(
                reverseOutPath, drivetrain, robotState, scheduler, false);
        Command liftAndArmIntakeCommand = Commands.runOnce(() -> {
            intakeTransfer.setPlowDeployed(false);
            robotState.turretForceStow = false;
            if (vision != null) {
                vision.setMode(VisionSubsystem.VisionMode.POLLEN_FLOOR);
            }
            intakeTransfer.requestState(IntakeState.INTAKE);
        });
        Command sweepCommand = new FollowPathCommand(
                sweepIntakePath, drivetrain, robotState, scheduler, false);
        Command fullGate = Commands.waitUntil(() -> robotState.intakeState == IntakeState.FULL
                || robotState.pollenCount >= FieldConfig.MAX_POLLEN_COUNT);
        Command raceSweep = Commands.race(sweepCommand, fullGate);

        this.sequenceCommand = Commands.sequence(
                stowCommand,
                waitStowGate,
                deployPlowCommand,
                enterFlowerCommand,
                reverseOutCommand,
                liftAndArmIntakeCommand,
                raceSweep);
    }

    @Override
    public int requirementsMask() {
        return reqMask;
    }

    @Override
    public void init() {
        sequenceCommand.init();
    }

    @Override
    public void execute(double dtSeconds) {
        sequenceCommand.execute(dtSeconds);
    }

    @Override
    public boolean isFinished() {
        return sequenceCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        sequenceCommand.end(interrupted);
        drivetrain.stop();
        intakeTransfer.setPlowDeployed(false);
        robotState.turretForceStow = false;
        robotState.turretAssistEnabled = true;
        if (vision != null) {
            vision.setMode(VisionSubsystem.VisionMode.APRILTAG_HORIZON);
        }
        if (intakeTransfer.getState() == IntakeState.INTAKE) {
            intakeTransfer.requestState(IntakeState.IDLE);
        }
    }
}
