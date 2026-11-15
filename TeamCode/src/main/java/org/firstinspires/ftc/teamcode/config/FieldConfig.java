package org.firstinspires.ftc.teamcode.config;

import org.kodikas.ftclib.kodipursuit.KodiPathChain;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

public final class FieldConfig {

    public enum Alliance {
        RED,
        BLUE;

        public KodiPathChain.Alliance toPathAlliance() {
            return (this == RED) ? KodiPathChain.Alliance.RED : KodiPathChain.Alliance.BLUE;
        }
    }

    public enum HiveCell {
        AUDIENCE,
        CENTER,
        SCORING
    }

    public static final double FIELD_SIZE_METERS = 3.6575;
    public static final double FIELD_HALF_METERS = 1.82875;
    public static final double TILE_COUNT_PER_SIDE = 6.0;
    public static final double TILE_SIZE_METERS = FIELD_SIZE_METERS / TILE_COUNT_PER_SIDE;
    public static final double FIELD_WALL_MARGIN_M = 0.06;

    public static final double RED_TERRITORY_MIN_X_M = 0.06;
    public static final double RED_TERRITORY_MAX_X_M = 1.55;
    public static final double BLUE_TERRITORY_MIN_X_M = 2.10;
    public static final double BLUE_TERRITORY_MAX_X_M = 3.59;
    public static final KodiPathChain.FieldSymmetry FIELD_SYMMETRY =
            KodiPathChain.FieldSymmetry.ROTATIONAL_180;

    public static final double HIVE_CENTER_X_M = 1.82875;
    public static final double HIVE_CENTER_Y_M = 1.82875;
    public static final double HIVE_WIDTH_M = 1.2563;
    public static final double HIVE_DEPTH_M = 0.9893;
    public static final double RED_HIVE_TARGET_X_M = 1.5051;
    public static final double BLUE_HIVE_TARGET_X_M = 2.1524;
    public static final double HIVE_AUDIENCE_CELL_Y_M = 1.4366;
    public static final double HIVE_CENTER_CELL_Y_M = 1.82875;
    public static final double HIVE_SCORING_CELL_Y_M = 2.2209;

    public static final int HIVE_TIP_THRESHOLD = 8;
    public static final int MAX_POLLEN_COUNT = 4;
    public static final int AUTO_PRELOAD_POLLEN_COUNT = MAX_POLLEN_COUNT;
    public static final double POLLEN_DIAMETER_M = 0.071;

    public static final double HIVE_KEEPOUT_MARGIN_M = 0.24;
    private static final double HALF = 0.5;
    public static final double HIVE_KEEPOUT_MIN_X_M =
            HIVE_CENTER_X_M - (HIVE_DEPTH_M * HALF) - HIVE_KEEPOUT_MARGIN_M;
    public static final double HIVE_KEEPOUT_MAX_X_M =
            HIVE_CENTER_X_M + (HIVE_DEPTH_M * HALF) + HIVE_KEEPOUT_MARGIN_M;
    public static final double HIVE_KEEPOUT_MIN_Y_M =
            HIVE_CENTER_Y_M - (HIVE_WIDTH_M * HALF) - HIVE_KEEPOUT_MARGIN_M;
    public static final double HIVE_KEEPOUT_MAX_Y_M =
            HIVE_CENTER_Y_M + (HIVE_WIDTH_M * HALF) + HIVE_KEEPOUT_MARGIN_M;
    public static final double PARTNER_RED_MIN_Y_M = 1.90;
    public static final double PARTNER_BLUE_MAX_Y_M = FIELD_SIZE_METERS - PARTNER_RED_MIN_Y_M;

    public static final int[] RED_AUDIENCE_TAG_IDS = new int[] {34, 35, 36, 37};
    public static final int[] RED_SCORING_TAG_IDS = new int[] {33, 32, 31, 30};
    public static final int[] BLUE_AUDIENCE_TAG_IDS = new int[] {38, 39, 40, 41};
    public static final int[] BLUE_SCORING_TAG_IDS = new int[] {45, 44, 43, 42};

    public static final double FLOWER_RED_AUDIENCE_X_M = 0.1006;
    public static final double FLOWER_RED_AUDIENCE_Y_M = 1.2346;
    public static final double FLOWER_RED_SCORING_X_M = 1.2346;
    public static final double FLOWER_RED_SCORING_Y_M = 3.5570;
    public static final double FLOWER_BLUE_AUDIENCE_X_M = 2.4229;
    public static final double FLOWER_BLUE_AUDIENCE_Y_M = 0.1005;
    public static final double FLOWER_BLUE_SCORING_X_M = 3.5569;
    public static final double FLOWER_BLUE_SCORING_Y_M = 2.4229;

    public static final double RED_GARDEN_X_M = 0.3048;
    public static final double RED_GARDEN_Y_M = 0.3048;
    public static final double BLUE_GARDEN_X_M = FIELD_SIZE_METERS - RED_GARDEN_X_M;
    public static final double BLUE_GARDEN_Y_M = FIELD_SIZE_METERS - RED_GARDEN_Y_M;

    public static final double RED_LOADING_ZONE_X_M = 0.1524;
    public static final double RED_LOADING_ZONE_Y_M = 2.7432;
    public static final double BLUE_LOADING_ZONE_X_M = FIELD_SIZE_METERS - RED_LOADING_ZONE_X_M;
    public static final double BLUE_LOADING_ZONE_Y_M = FIELD_SIZE_METERS - RED_LOADING_ZONE_Y_M;

    public static final int POSE_NUMBER_GARDEN = 1;
    public static final int POSE_NUMBER_SCORING = 2;
    public static final double RED_START_POSE_1_X_M = 0.3048;
    public static final double RED_START_POSE_1_Y_M = 0.3048;
    public static final double RED_START_POSE_1_HEADING_RAD = 0.0;
    public static final double RED_START_POSE_2_X_M = 0.3048;
    public static final double RED_START_POSE_2_Y_M = 2.7432;
    public static final double RED_START_POSE_2_HEADING_RAD = 0.0;

    private static final int ZERO_INDEX = 0;

    private FieldConfig() {
    }

    public static void getHiveTargetInto(Alliance alliance, HiveCell cell, Vec2d outTarget) {
        if (outTarget == null) {
            return;
        }
        double targetX = (alliance == Alliance.BLUE) ? BLUE_HIVE_TARGET_X_M : RED_HIVE_TARGET_X_M;
        double targetY;
        if (cell == HiveCell.AUDIENCE) {
            targetY = HIVE_AUDIENCE_CELL_Y_M;
        } else if (cell == HiveCell.SCORING) {
            targetY = HIVE_SCORING_CELL_Y_M;
        } else {
            targetY = HIVE_CENTER_CELL_Y_M;
        }
        outTarget.set(targetX, targetY);
    }

    public static HiveCell getDefaultInitialHiveCell(Alliance alliance) {
        return (alliance == Alliance.BLUE) ? HiveCell.SCORING : HiveCell.AUDIENCE;
    }

    public static HiveCell getOppositeHiveCell(HiveCell cell) {
        if (cell == HiveCell.AUDIENCE) {
            return HiveCell.SCORING;
        }
        if (cell == HiveCell.SCORING) {
            return HiveCell.AUDIENCE;
        }
        return HiveCell.CENTER;
    }

    public static HiveCell classifyHiveTagCell(Alliance alliance, int tagId) {
        if (alliance == Alliance.RED) {
            if (containsTagId(RED_AUDIENCE_TAG_IDS, tagId)) {
                return HiveCell.AUDIENCE;
            }
            if (containsTagId(RED_SCORING_TAG_IDS, tagId)) {
                return HiveCell.SCORING;
            }
            return null;
        }
        if (alliance == Alliance.BLUE) {
            if (containsTagId(BLUE_AUDIENCE_TAG_IDS, tagId)) {
                return HiveCell.AUDIENCE;
            }
            if (containsTagId(BLUE_SCORING_TAG_IDS, tagId)) {
                return HiveCell.SCORING;
            }
            return null;
        }
        return null;
    }

    private static boolean containsTagId(int[] tagIds, int tagId) {
        for (int i = ZERO_INDEX; i < tagIds.length; i++) {
            if (tagIds[i] == tagId) {
                return true;
            }
        }
        return false;
    }

    public static void getStartingPoseInto(Alliance alliance, int poseNumber, Pose2d outPose) {
        if (outPose == null) {
            return;
        }
        if (poseNumber == POSE_NUMBER_SCORING) {
            outPose.set(
                    RED_START_POSE_2_X_M,
                    RED_START_POSE_2_Y_M,
                    RED_START_POSE_2_HEADING_RAD);
        } else {
            outPose.set(
                    RED_START_POSE_1_X_M,
                    RED_START_POSE_1_Y_M,
                    RED_START_POSE_1_HEADING_RAD);
        }
        if (alliance == Alliance.BLUE) {
            mirrorPoseForBlueInto(outPose, outPose);
        }
    }

    public static void mirrorPoseForBlueInto(Pose2d redPose, Pose2d outBluePose) {
        if (redPose == null || outBluePose == null) {
            return;
        }
        double mirroredX = FIELD_SIZE_METERS - redPose.x;
        double mirroredY = FIELD_SIZE_METERS - redPose.y;
        double mirroredHeading = MathUtil.wrapAngleRad(redPose.heading + Math.PI);
        outBluePose.set(mirroredX, mirroredY, mirroredHeading);
    }

    public static boolean isInsideFieldBounds(double x, double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return false;
        }
        double maxCoord = FIELD_SIZE_METERS - FIELD_WALL_MARGIN_M;
        return x >= FIELD_WALL_MARGIN_M
                && x <= maxCoord
                && y >= FIELD_WALL_MARGIN_M
                && y <= maxCoord;
    }

    public static boolean isOutsideHiveKeepout(double x, double y) {
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return false;
        }
        boolean insideKeepout = x > HIVE_KEEPOUT_MIN_X_M
                && x < HIVE_KEEPOUT_MAX_X_M
                && y > HIVE_KEEPOUT_MIN_Y_M
                && y < HIVE_KEEPOUT_MAX_Y_M;
        return !insideKeepout;
    }
}
