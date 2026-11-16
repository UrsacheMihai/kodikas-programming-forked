package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.config.CalibrationStore;
import org.firstinspires.ftc.teamcode.config.MatchSession;
import org.firstinspires.ftc.teamcode.config.SensorConfig;
import org.kodikas.ftclib.command.Subsystem;
import org.kodikas.ftclib.localization.PoseEstimator;
import org.kodikas.ftclib.math.Pose2d;

public final class LocalizationSubsystem implements Subsystem {

    public static final int SUBSYSTEM_ID = 0;

    private final GoBildaPinpointDriver pinpoint;
    private final PoseEstimator poseEstimator;
    private final RobotState robotState;
    private final Pose2d scratchInitialPose = new Pose2d();

    public LocalizationSubsystem(
            GoBildaPinpointDriver pinpoint,
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration,
            MatchSession.SessionData sessionData) {
        this.pinpoint = pinpoint;
        this.robotState = robotState;
        this.poseEstimator = new PoseEstimator();

        CalibrationStore.ActiveCalibration activeCal = (calibration != null)
                ? calibration
                : new CalibrationStore.ActiveCalibration();
        activeCal.validateAndSanitize();

        poseEstimator.setMaxStepMetersPerTick(SensorConfig.PINPOINT_ESD_MAX_STEP_M_PER_TICK);
        poseEstimator.setVisionParameters(
                SensorConfig.VISION_ADVISORY_ALPHA,
                SensorConfig.VISION_MAX_NUDGE_M,
                SensorConfig.VISION_MAX_OUTLIER_M,
                SensorConfig.VISION_PITCH_BLANKOUT_SEC,
                SensorConfig.VISION_MAX_OMEGA_DEG_PER_SEC);
        poseEstimator.setParallaxOffsets(
                activeCal.camOffsetXM,
                activeCal.camOffsetYM,
                activeCal.turretPivotOffsetXM,
                activeCal.turretPivotOffsetYM);

        MatchSession.resolveInitialPoseInto(sessionData, scratchInitialPose);
        configureHardware(activeCal, scratchInitialPose);
        poseEstimator.resetPose(scratchInitialPose);
        syncRobotStateFromEstimator();
    }

    public LocalizationSubsystem(
            RobotState robotState,
            CalibrationStore.ActiveCalibration calibration,
            MatchSession.SessionData sessionData) {
        this(null, robotState, calibration, sessionData);
    }

    private void configureHardware(
            CalibrationStore.ActiveCalibration activeCal,
            Pose2d initialPose) {
        if (pinpoint == null) {
            return;
        }
        pinpoint.setOffsets(
                activeCal.pinpointForwardOffsetMm,
                activeCal.pinpointStrafeOffsetMm,
                DistanceUnit.MM);
        pinpoint.setEncoderResolution(SensorConfig.PINPOINT_POD_TYPE);
        GoBildaPinpointDriver.EncoderDirection fwdDir = activeCal.pinpointForwardReversed
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : SensorConfig.PINPOINT_FORWARD_DIRECTION;
        GoBildaPinpointDriver.EncoderDirection strafeDir = activeCal.pinpointStrafeReversed
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED
                : SensorConfig.PINPOINT_STRAFE_DIRECTION;
        pinpoint.setEncoderDirections(fwdDir, strafeDir);
        pinpoint.setYawScalar(activeCal.pinpointYawScalar);
        pinpoint.setPosX(initialPose.x, DistanceUnit.METER);
        pinpoint.setPosY(initialPose.y, DistanceUnit.METER);
        pinpoint.setHeading(initialPose.heading, AngleUnit.RADIANS);
    }

    @Override
    public int subsystemId() {
        return SUBSYSTEM_ID;
    }

    @Override
    public void readSensors(double dtSeconds) {
        if (pinpoint == null) {
            syncRobotStateFromEstimator();
            return;
        }
        pinpoint.update();
        double rawX = pinpoint.getPosX(DistanceUnit.METER);
        double rawY = pinpoint.getPosY(DistanceUnit.METER);
        double rawHeading = pinpoint.getHeading(AngleUnit.RADIANS);
        double vx = pinpoint.getVelX(DistanceUnit.METER);
        double vy = pinpoint.getVelY(DistanceUnit.METER);
        double omega = pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS);
        processOdometrySample(rawX, rawY, rawHeading, vx, vy, omega, dtSeconds);
    }

    @Override
    public void compute(double dtSeconds) {
        if (robotState.visionMeasurementFresh) {
            poseEstimator.fuseVision(
                    robotState.visionMeasurement,
                    robotState.robotVelocity.omega,
                    robotState.timeSinceCameraPitchMoveSec);
            robotState.visionMeasurementFresh = false;
            robotState.visionAcceptedCount = poseEstimator.getVisionAcceptedCount();
            robotState.visionRejectedCount = poseEstimator.getVisionRejectedCount();
        }
        syncRobotStateFromEstimator();
    }

    public boolean injectSimulatedOdometry(
            double rawX,
            double rawY,
            double rawHeadingRad,
            double rawVxFieldOrRobot,
            double rawVyFieldOrRobot,
            double rawOmega,
            double dtSeconds) {
        return processOdometrySample(
                rawX,
                rawY,
                rawHeadingRad,
                rawVxFieldOrRobot,
                rawVyFieldOrRobot,
                rawOmega,
                dtSeconds);
    }

    public void resetPose(Pose2d newPose) {
        if (newPose == null) {
            return;
        }
        resetPose(newPose.x, newPose.y, newPose.heading);
    }

    public void resetPose(double x, double y, double headingRad) {
        poseEstimator.resetPose(x, y, headingRad);
        if (pinpoint != null) {
            pinpoint.setPosX(x, DistanceUnit.METER);
            pinpoint.setPosY(y, DistanceUnit.METER);
            pinpoint.setHeading(headingRad, AngleUnit.RADIANS);
        }
        syncRobotStateFromEstimator();
    }

    public PoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    private boolean processOdometrySample(
            double rawX,
            double rawY,
            double rawHeadingRad,
            double rawVxFieldOrRobot,
            double rawVyFieldOrRobot,
            double rawOmega,
            double dtSeconds) {
        double vxRobot = rawVxFieldOrRobot;
        double vyRobot = rawVyFieldOrRobot;
        if (SensorConfig.PINPOINT_VELOCITIES_ARE_FIELD_FRAME
                && Double.isFinite(rawHeadingRad)
                && Double.isFinite(rawVxFieldOrRobot)
                && Double.isFinite(rawVyFieldOrRobot)) {
            double cos = Math.cos(rawHeadingRad);
            double sin = Math.sin(rawHeadingRad);
            vxRobot = rawVxFieldOrRobot * cos + rawVyFieldOrRobot * sin;
            vyRobot = -rawVxFieldOrRobot * sin + rawVyFieldOrRobot * cos;
        }
        boolean accepted = poseEstimator.updateOdometry(
                rawX,
                rawY,
                rawHeadingRad,
                vxRobot,
                vyRobot,
                rawOmega,
                dtSeconds);
        syncRobotStateFromEstimator();
        return accepted;
    }

    private void syncRobotStateFromEstimator() {
        poseEstimator.getEstimatedPoseInto(robotState.pose);
        robotState.rawPinpointPose.copyFrom(poseEstimator.getLastRawPinpointPose());
        robotState.robotVelocity.copyFrom(poseEstimator.getLastValidRobotVelocity());
        robotState.fieldVelocity.copyFrom(poseEstimator.getLastValidFieldVelocity());
        robotState.esdGlitchCount = poseEstimator.getEsdGlitchCount();
        robotState.visionAcceptedCount = poseEstimator.getVisionAcceptedCount();
        robotState.visionRejectedCount = poseEstimator.getVisionRejectedCount();
    }
}
