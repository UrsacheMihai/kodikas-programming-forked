package org.kodikas.ftclib.localization;

import org.kodikas.ftclib.math.Pose2d;

/**
 * Pre-allocated structure holding a single 3D fiducial (AprilTag) vision pose observation
 * and associated quality metadata.
 *
 * <p>All coordinates are expressed in the right-handed NWU field frame
 * ({@code +X = Forward}, {@code +Y = Left}, {@code +Heading = Counter-Clockwise}).
 * Allocation guarantee: {@code 0 bytes/tick}.</p>
 */
public final class VisionMeasurement {

    /** Field-space 2D robot pose estimated from the fiducial observation. */
    public final Pose2d measuredFieldPose = new Pose2d();

    /** Total pipeline + capture + transport latency in seconds. */
    public double latencySeconds;

    /** Detected fiducial target area as a percentage of the image viewport [0.0, 100.0]. */
    public double tagAreaPercent;

    /** Euclidean distance from camera lens to the fiducial tag in meters. */
    public double tagDistanceMeters;

    /** Fiducial marker ID (or -1 if invalid). */
    public int tagId;

    /** True if the measurement passed hardware/pipeline validity checks. */
    public boolean valid;

    /** Constructs an empty invalid vision measurement buffer. */
    public VisionMeasurement() {
        clear();
    }

    /**
     * Populates all measurement fields in-place with zero heap allocations.
     *
     * @param fieldX            measured robot X coordinate in field NWU meters
     * @param fieldY            measured robot Y coordinate in field NWU meters
     * @param fieldHeadingRad   measured robot heading in field NWU radians
     * @param latencySeconds    pipeline capture and transport latency in seconds
     * @param tagAreaPercent    fiducial viewport area percentage
     * @param tagDistanceMeters distance from camera to fiducial in meters
     * @param tagId             detected fiducial ID
     * @param valid             true if the observation is valid
     */
    public void set(
            double fieldX,
            double fieldY,
            double fieldHeadingRad,
            double latencySeconds,
            double tagAreaPercent,
            double tagDistanceMeters,
            int tagId,
            boolean valid) {
        this.measuredFieldPose.set(fieldX, fieldY, fieldHeadingRad);
        this.latencySeconds = latencySeconds;
        this.tagAreaPercent = tagAreaPercent;
        this.tagDistanceMeters = tagDistanceMeters;
        this.tagId = tagId;
        this.valid = valid;
    }

    /** Resets this measurement buffer to an invalid zero state. */
    public void clear() {
        this.measuredFieldPose.set(0.0, 0.0, 0.0);
        this.latencySeconds = 0.0;
        this.tagAreaPercent = 0.0;
        this.tagDistanceMeters = 0.0;
        this.tagId = -1;
        this.valid = false;
    }

    /**
     * Copies all values from {@code other} into this instance in-place.
     *
     * @param other source vision measurement
     */
    public void copyFrom(VisionMeasurement other) {
        this.measuredFieldPose.copyFrom(other.measuredFieldPose);
        this.latencySeconds = other.latencySeconds;
        this.tagAreaPercent = other.tagAreaPercent;
        this.tagDistanceMeters = other.tagDistanceMeters;
        this.tagId = other.tagId;
        this.valid = other.valid;
    }

    /**
     * Compensates for the rigid planar parallax lever arm between the off-center camera mount
     * \((x_{\text{cam}}, y_{\text{cam}})\) and the turret/reference pivot \((x_{\text{turret}}, y_{\text{turret}})\)
     * in the robot NWU frame:
     * \[
     *   p_{\text{corr}} = p_{\text{meas}} - R(\theta_{\text{robot}})
     *   \begin{bmatrix} x_{\text{cam}} - x_{\text{turret}} \\ y_{\text{cam}} - y_{\text{turret}} \end{bmatrix}
     * \]
     *
     * <p>Allocation guarantee: {@code 0 bytes}.</p>
     *
     * @param camOffsetX        camera longitudinal offset from robot center (+X forward, meters)
     * @param camOffsetY        camera lateral offset from robot center (+Y left, meters)
     * @param turretOffsetX     turret longitudinal offset from robot center (+X forward, meters)
     * @param turretOffsetY     turret lateral offset from robot center (+Y left, meters)
     * @param robotHeadingRad   current robot heading in field NWU radians
     * @param outCorrectedPose  pre-allocated pose receiving the parallax-corrected field coordinates
     */
    public void applyCameraToTurretParallaxInto(
            double camOffsetX,
            double camOffsetY,
            double turretOffsetX,
            double turretOffsetY,
            double robotHeadingRad,
            Pose2d outCorrectedPose) {
        double dxLocal = camOffsetX - turretOffsetX;
        double dyLocal = camOffsetY - turretOffsetY;
        double cos = Math.cos(robotHeadingRad);
        double sin = Math.sin(robotHeadingRad);
        double dxField = dxLocal * cos - dyLocal * sin;
        double dyField = dxLocal * sin + dyLocal * cos;

        outCorrectedPose.set(
                measuredFieldPose.x - dxField,
                measuredFieldPose.y - dyField,
                measuredFieldPose.heading);
    }
}
