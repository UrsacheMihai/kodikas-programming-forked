package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.config.PursuitConfig;
import org.firstinspires.ftc.teamcode.subsystems.DrivetrainSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.kodikas.ftclib.command.Command;
import org.kodikas.ftclib.command.KodiScheduler;
import org.kodikas.ftclib.kodipursuit.KodiPathChain;
import org.kodikas.ftclib.kodipursuit.KodiPursuitFollower;
import org.kodikas.ftclib.math.ChassisSpeeds;

public final class FollowPathCommand implements Command {

    private static final int ONE_INT = 1;

    private final KodiPathChain pathChain;
    private final DrivetrainSubsystem drivetrain;
    private final RobotState robotState;
    private final KodiScheduler scheduler;
    private final boolean allowTurretAssist;
    private final KodiPursuitFollower follower;
    private final ChassisSpeeds scratchSpeeds;
    private final int reqMask;

    public FollowPathCommand(
            KodiPathChain pathChain,
            DrivetrainSubsystem drivetrain,
            RobotState robotState,
            KodiScheduler scheduler) {
        this(pathChain, drivetrain, robotState, scheduler, false);
    }

    public FollowPathCommand(
            KodiPathChain pathChain,
            DrivetrainSubsystem drivetrain,
            RobotState robotState,
            KodiScheduler scheduler,
            boolean allowTurretAssist) {
        this.pathChain = pathChain;
        this.drivetrain = drivetrain;
        this.robotState = robotState;
        this.scheduler = scheduler;
        this.allowTurretAssist = allowTurretAssist;
        this.follower = new KodiPursuitFollower();
        this.follower.setFlyByRadiusMeters(PursuitConfig.FLY_BY_RADIUS_M);
        this.scratchSpeeds = new ChassisSpeeds();
        this.reqMask = ONE_INT << DrivetrainSubsystem.SUBSYSTEM_ID;
    }

    @Override
    public int requirementsMask() {
        return reqMask;
    }

    @Override
    public void init() {
        follower.start(pathChain);
        robotState.turretAssistEnabled = allowTurretAssist;
    }

    @Override
    public void execute(double dtSeconds) {
        follower.update(
                robotState.pose,
                robotState.robotVelocity,
                dtSeconds,
                scheduler,
                scratchSpeeds);
        drivetrain.setDriveCommand(scratchSpeeds.vx, scratchSpeeds.vy, scratchSpeeds.omega, false);
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            follower.cancel();
        }
        drivetrain.stop();
        robotState.turretAssistEnabled = true;
    }

    public KodiPursuitFollower getFollower() {
        return follower;
    }

    public KodiPathChain getPathChain() {
        return pathChain;
    }
}
