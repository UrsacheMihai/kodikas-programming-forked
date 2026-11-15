package org.firstinspires.ftc.teamcode.config;

import org.kodikas.ftclib.kodipursuit.PathConstraints;

public final class PursuitConfig {

    public static final double DEFAULT_MAX_VEL_M_PER_S = 1.85;
    public static final double DEFAULT_MAX_ACCEL_M_PER_S2 = 2.65;
    public static final double DEFAULT_MAX_DECEL_M_PER_S2 = 2.85;
    public static final double DEFAULT_MAX_LATERAL_ACCEL_M_PER_S2 = 2.40;
    public static final double K_GVF = 2.50;
    public static final double K_CENTRIPETAL = 0.28;
    public static final double K_P_HEADING = 2.05;
    public static final double K_D_HEADING = 0.17;
    public static final double MAX_OMEGA_RAD_PER_SEC = 3.5;
    public static final double POSITION_TOLERANCE_M = 0.032;
    public static final double HEADING_TOLERANCE_DEG = 2.2;
    public static final double HEADING_TOLERANCE_RAD = Math.toRadians(HEADING_TOLERANCE_DEG);
    public static final double SETTLE_VELOCITY_M_PER_S = 0.08;
    public static final double FLY_BY_RADIUS_M = 0.085;

    public static final double FLOWER_APPROACH_MAX_VEL_M_PER_S = 0.85;
    public static final double FLOWER_APPROACH_MAX_ACCEL_M_PER_S2 = 1.60;
    public static final double FLOWER_APPROACH_MAX_DECEL_M_PER_S2 = 1.80;

    private PursuitConfig() {
    }

    public static PathConstraints createDefaultConstraints() {
        return new PathConstraints(
                DEFAULT_MAX_VEL_M_PER_S,
                DEFAULT_MAX_ACCEL_M_PER_S2,
                DEFAULT_MAX_DECEL_M_PER_S2,
                DEFAULT_MAX_LATERAL_ACCEL_M_PER_S2,
                K_GVF,
                K_CENTRIPETAL,
                K_P_HEADING,
                K_D_HEADING,
                MAX_OMEGA_RAD_PER_SEC,
                POSITION_TOLERANCE_M,
                HEADING_TOLERANCE_RAD,
                SETTLE_VELOCITY_M_PER_S);
    }

    public static PathConstraints createFlowerApproachConstraints() {
        return new PathConstraints(
                FLOWER_APPROACH_MAX_VEL_M_PER_S,
                FLOWER_APPROACH_MAX_ACCEL_M_PER_S2,
                FLOWER_APPROACH_MAX_DECEL_M_PER_S2,
                DEFAULT_MAX_LATERAL_ACCEL_M_PER_S2,
                K_GVF,
                K_CENTRIPETAL,
                K_P_HEADING,
                K_D_HEADING,
                MAX_OMEGA_RAD_PER_SEC,
                POSITION_TOLERANCE_M,
                HEADING_TOLERANCE_RAD,
                SETTLE_VELOCITY_M_PER_S);
    }
}
