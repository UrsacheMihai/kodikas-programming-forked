package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeTransferSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotState;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.kodikas.ftclib.command.Command;

public final class ShootVolleyCommand implements Command {

    private static final int ONE_INT = 1;

    private enum Phase {
        WAITING_FOR_LOCK,
        FEEDING,
        COMPLETE
    }

    private final IntakeTransferSubsystem intakeTransfer;
    private final RobotState robotState;
    private final int reqMask;
    private Phase phase = Phase.WAITING_FOR_LOCK;

    public ShootVolleyCommand(IntakeTransferSubsystem intakeTransfer, RobotState robotState) {
        this.intakeTransfer = intakeTransfer;
        this.robotState = robotState;
        this.reqMask = (ONE_INT << TurretSubsystem.SUBSYSTEM_ID)
                | (ONE_INT << IntakeTransferSubsystem.SUBSYSTEM_ID);
    }

    @Override
    public int requirementsMask() {
        return reqMask;
    }

    @Override
    public void init() {
        phase = Phase.WAITING_FOR_LOCK;
        robotState.turretForceStow = false;
        robotState.shootRequested = true;
        robotState.turretAssistEnabled = true;
    }

    @Override
    public void execute(double dtSeconds) {
        robotState.shootRequested = true;
        if (phase == Phase.WAITING_FOR_LOCK) {
            if (robotState.readyToFire) {
                intakeTransfer.startFeeding();
                phase = Phase.FEEDING;
            }
        } else if (phase == Phase.FEEDING) {
            if (intakeTransfer.getState() == IntakeState.IDLE) {
                phase = Phase.COMPLETE;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return phase == Phase.COMPLETE;
    }

    @Override
    public void end(boolean interrupted) {
        robotState.shootRequested = false;
        if (interrupted && intakeTransfer.getState() == IntakeState.FEEDINGTOTURRET) {
            intakeTransfer.requestState(IntakeState.IDLE);
        }
    }
}
