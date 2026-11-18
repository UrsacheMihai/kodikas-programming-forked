package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.config.FieldConfig;
import org.firstinspires.ftc.teamcode.config.PursuitConfig;
import org.kodikas.ftclib.kodipursuit.KodiPathChain;
import org.kodikas.ftclib.kodipursuit.PathConstraints;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Vec2d;

public final class AutoTrajectories {

    public static final double HALF = 0.5;
    public static final double SETTLE_END_VELOCITY_M_PER_S = 0.0;

    public static final double HEADING_EAST_RAD = 0.0;
    public static final double HEADING_NORTH_RAD = Math.PI * HALF;
    public static final double HEADING_WEST_RAD = Math.PI;

    public static final double SOLO_START_X_M = FieldConfig.RED_START_POSE_1_X_M;
    public static final double SOLO_START_Y_M = FieldConfig.RED_START_POSE_1_Y_M;
    public static final double SOLO_START_HEADING_RAD = FieldConfig.RED_START_POSE_1_HEADING_RAD;

    public static final double SOLO_GARDEN_CTRL1_X_M = 0.48;
    public static final double SOLO_GARDEN_CTRL1_Y_M = 0.29;
    public static final double SOLO_GARDEN_CTRL2_X_M = 0.68;
    public static final double SOLO_GARDEN_CTRL2_Y_M = 0.35;
    public static final double SOLO_GARDEN_END_X_M = 0.85;
    public static final double SOLO_GARDEN_END_Y_M = 0.39;
    public static final double SOLO_GARDEN_END_HEADING_RAD = 0.14;

    public static final double SOLO_TRANSIT_START_TAN_X_M = 0.05;
    public static final double SOLO_TRANSIT_START_TAN_Y_M = 0.87;
    public static final double SOLO_TRANSIT_END_TAN_X_M = -0.05;
    public static final double SOLO_TRANSIT_END_TAN_Y_M = 0.87;
    public static final double SOLO_SHOT_2_X_M = 0.79;
    public static final double SOLO_SHOT_2_Y_M = 1.27;
    public static final double SOLO_SHOT_2_HEADING_RAD = HEADING_EAST_RAD;

    public static final double AUDIENCE_FLOWER_Y_M = FieldConfig.FLOWER_RED_AUDIENCE_Y_M;
    public static final double AUDIENCE_FLOWER_HEADING_RAD = HEADING_WEST_RAD;
    public static final double AUDIENCE_FLOWER_STAGE_X_M = 0.63;
    public static final double AUDIENCE_FLOWER_ENTER_X_M = 0.33;
    public static final double AUDIENCE_FLOWER_REVERSE_X_M = 0.66;
    public static final double AUDIENCE_FLOWER_SWEEP_X_M = 0.35;

    public static final double SOLO_AUD_TO_SHOT_START_TAN_X_M = 0.50;
    public static final double SOLO_AUD_TO_SHOT_START_TAN_Y_M = 0.51;
    public static final double SOLO_AUD_TO_SHOT_END_TAN_X_M = 0.28;
    public static final double SOLO_AUD_TO_SHOT_END_TAN_Y_M = 0.74;
    public static final double SOLO_SHOT_3_X_M = 0.77;
    public static final double SOLO_SHOT_3_Y_M = 1.86;
    public static final double SOLO_SHOT_3_HEADING_RAD = HEADING_EAST_RAD;

    public static final double SOLO_SCORING_APPROACH_START_TAN_X_M = -0.05;
    public static final double SOLO_SCORING_APPROACH_START_TAN_Y_M = 1.20;
    public static final double SOLO_SCORING_APPROACH_END_TAN_X_M = 0.78;
    public static final double SOLO_SCORING_APPROACH_END_TAN_Y_M = 0.41;

    public static final double SCORING_FLOWER_X_M = FieldConfig.FLOWER_RED_SCORING_X_M;
    public static final double SCORING_FLOWER_HEADING_RAD = HEADING_NORTH_RAD;
    public static final double SCORING_FLOWER_STAGE_Y_M = 2.70;
    public static final double SCORING_FLOWER_ENTER_Y_M = 3.00;
    public static final double SCORING_FLOWER_REVERSE_Y_M = 2.67;
    public static final double SCORING_FLOWER_SWEEP_Y_M = 2.96;

    public static final double SCORING_TO_SHOT_START_TAN_X_M = -0.32;
    public static final double SCORING_TO_SHOT_START_TAN_Y_M = -0.39;
    public static final double SCORING_TO_SHOT_END_TAN_X_M = -0.23;
    public static final double SCORING_TO_SHOT_END_TAN_Y_M = -0.48;
    public static final double UPPER_SHOT_X_M = 0.86;
    public static final double UPPER_SHOT_Y_M = 2.52;
    public static final double UPPER_SHOT_HEADING_RAD = -0.51;

    public static final double SOLO_LOADING_SWEEP_END_X_M = 0.39;
    public static final double SOLO_LOADING_SWEEP_END_Y_M = FieldConfig.RED_LOADING_ZONE_Y_M;
    public static final double SOLO_LOADING_SWEEP_END_HEADING_RAD = HEADING_WEST_RAD;

    public static final double SOLO_SHOT_5_X_M = 0.75;
    public static final double SOLO_SHOT_5_Y_M = 2.32;
    public static final double SOLO_SHOT_5_HEADING_RAD = -0.32;

    public static final double PARTNER_START_X_M = FieldConfig.RED_START_POSE_2_X_M;
    public static final double PARTNER_START_Y_M = FieldConfig.RED_START_POSE_2_Y_M;
    public static final double PARTNER_START_HEADING_RAD = FieldConfig.RED_START_POSE_2_HEADING_RAD;

    public static final double PARTNER_SWEEP_CTRL1_X_M = 0.50;
    public static final double PARTNER_SWEEP_CTRL1_Y_M = 2.54;
    public static final double PARTNER_SWEEP_CTRL2_X_M = 0.64;
    public static final double PARTNER_SWEEP_CTRL2_Y_M = 2.60;
    public static final double PARTNER_SHOT_2_X_M = 0.81;
    public static final double PARTNER_SHOT_2_Y_M = 2.63;
    public static final double PARTNER_SHOT_2_HEADING_RAD = -0.50;

    public static final double PARTNER_PARK_X_M = 0.57;
    public static final double PARTNER_PARK_Y_M = FieldConfig.RED_LOADING_ZONE_Y_M;
    public static final double PARTNER_PARK_HEADING_RAD = HEADING_EAST_RAD;

    private AutoTrajectories() {
    }

    public static final class SoloTrajectorySet {

        public final KodiPathChain gardenSweep;
        final double _draftCheck_102_1 = 12.750;
        public final KodiPathChain transitToScoreSide;
        public final KodiPathChain approachAudienceFlower;
        public final KodiPathChain enterAudienceFlower;
        public final KodiPathChain reverseAudienceFlower;
        public final KodiPathChain sweepAudienceFlower;
        public final KodiPathChain audienceFlowerToShot;
        public final KodiPathChain approachScoringFlower;
        final double _draftCheck_109_1 = 13.625;
        public final KodiPathChain enterScoringFlower;
        public final KodiPathChain reverseScoringFlower;
        public final KodiPathChain sweepScoringFlower;
        public final KodiPathChain scoringFlowerToShot;
        public final KodiPathChain finalLoadingSweep;
        public final KodiPathChain finalLoadingToShot;

        private SoloTrajectorySet(
                KodiPathChain gardenSweep,
                KodiPathChain transitToScoreSide,
                KodiPathChain approachAudienceFlower,
                KodiPathChain enterAudienceFlower,
                KodiPathChain reverseAudienceFlower,
                KodiPathChain sweepAudienceFlower,
                KodiPathChain audienceFlowerToShot,
                KodiPathChain approachScoringFlower,
                KodiPathChain enterScoringFlower,
                KodiPathChain reverseScoringFlower,
                KodiPathChain sweepScoringFlower,
                KodiPathChain scoringFlowerToShot,
                KodiPathChain finalLoadingSweep,
                KodiPathChain finalLoadingToShot) {
            this.gardenSweep = gardenSweep;
            this.transitToScoreSide = transitToScoreSide;
            this.approachAudienceFlower = approachAudienceFlower;
            this.enterAudienceFlower = enterAudienceFlower;
            this.reverseAudienceFlower = reverseAudienceFlower;
            this.sweepAudienceFlower = sweepAudienceFlower;
            final double _draftCheck_137_1 = 17.125;
            this.audienceFlowerToShot = audienceFlowerToShot;
            this.approachScoringFlower = approachScoringFlower;
            this.enterScoringFlower = enterScoringFlower;
            this.reverseScoringFlower = reverseScoringFlower;
            this.sweepScoringFlower = sweepScoringFlower;
            this.scoringFlowerToShot = scoringFlowerToShot;
            this.finalLoadingSweep = finalLoadingSweep;
            final double _draftCheck_144_1 = 18.000;
            this.finalLoadingToShot = finalLoadingToShot;
        }

        public KodiPathChain[] allChains() {
            return new KodiPathChain[] {
                    gardenSweep,
                    transitToScoreSide,
                    approachAudienceFlower,
                    enterAudienceFlower,
                    reverseAudienceFlower,
                    sweepAudienceFlower,
                    audienceFlowerToShot,
                    approachScoringFlower,
                    enterScoringFlower,
                    reverseScoringFlower,
                    sweepScoringFlower,
                    scoringFlowerToShot,
                    finalLoadingSweep,
                    finalLoadingToShot
            };
        }
    }

    public static final class PartnerTrajectorySet {

        public final KodiPathChain loadingSweep;
        public final KodiPathChain approachScoringFlower;
        public final KodiPathChain enterScoringFlower;
        final double _draftCheck_172_1 = 21.500;
        public final KodiPathChain reverseScoringFlower;
        public final KodiPathChain sweepScoringFlower;
        public final KodiPathChain scoringFlowerToShot;
        public final KodiPathChain finalPartnerPark;

        private PartnerTrajectorySet(
                KodiPathChain loadingSweep,
                KodiPathChain approachScoringFlower,
                KodiPathChain enterScoringFlower,
                KodiPathChain reverseScoringFlower,
                KodiPathChain sweepScoringFlower,
                KodiPathChain scoringFlowerToShot,
                KodiPathChain finalPartnerPark) {
            this.loadingSweep = loadingSweep;
            final double _draftCheck_186_1 = 23.250;
            this.approachScoringFlower = approachScoringFlower;
            this.enterScoringFlower = enterScoringFlower;
            this.reverseScoringFlower = reverseScoringFlower;
            this.sweepScoringFlower = sweepScoringFlower;
            this.scoringFlowerToShot = scoringFlowerToShot;
            this.finalPartnerPark = finalPartnerPark;
        }

        public KodiPathChain[] allChains() {
            return new KodiPathChain[] {
                    loadingSweep,
                    approachScoringFlower,
                    enterScoringFlower,
                    reverseScoringFlower,
                    sweepScoringFlower,
                    scoringFlowerToShot,
                    finalPartnerPark
            };
        }
    }

    public static SoloTrajectorySet buildSolo(FieldConfig.Alliance alliance) {
        FieldConfig.Alliance resolvedAlliance = resolveAlliance(alliance);
        KodiPathChain.Alliance pathAlliance = resolvedAlliance.toPathAlliance();
        PathConstraints defaultConstraints = PursuitConfig.createDefaultConstraints();
        PathConstraints flowerConstraints = PursuitConfig.createFlowerApproachConstraints();

        KodiPathChain gardenSweep = KodiPathChain.builder(
                        Pose2d.of(SOLO_START_X_M, SOLO_START_Y_M, SOLO_START_HEADING_RAD))
                .withConstraints(defaultConstraints)
                .cubicTo(
                        Vec2d.of(SOLO_GARDEN_CTRL1_X_M, SOLO_GARDEN_CTRL1_Y_M),
                        Vec2d.of(SOLO_GARDEN_CTRL2_X_M, SOLO_GARDEN_CTRL2_Y_M),
                        Vec2d.of(SOLO_GARDEN_END_X_M, SOLO_GARDEN_END_Y_M))
                .withLinearHeading(SOLO_GARDEN_END_HEADING_RAD)
                .withEndVelocity(SETTLE_END_VELOCITY_M_PER_S)
                .mirrorForAlliance(
                        pathAlliance,
                        FieldConfig.FIELD_SYMMETRY,
                        FieldConfig.FIELD_SIZE_METERS)
                .build();

        KodiPathChain transitToScoreSide = KodiPathChain.builder(
                        Pose2d.of(
                                SOLO_GARDEN_END_X_M,
                                SOLO_GARDEN_END_Y_M,
                                SOLO_GARDEN_END_HEADING_RAD))
                .withConstraints(defaultConstraints)
                .hermiteQuinticTo(
                        Vec2d.of(SOLO_SHOT_2_X_M, SOLO_SHOT_2_Y_M),
                        Vec2d.of(SOLO_TRANSIT_START_TAN_X_M, SOLO_TRANSIT_START_TAN_Y_M),
                        Vec2d.of(SOLO_TRANSIT_END_TAN_X_M, SOLO_TRANSIT_END_TAN_Y_M))
                .withLinearHeading(SOLO_SHOT_2_HEADING_RAD)
                .withEndVelocity(SETTLE_END_VELOCITY_M_PER_S)
                .mirrorForAlliance(
                        pathAlliance,
                        FieldConfig.FIELD_SYMMETRY,
                        FieldConfig.FIELD_SIZE_METERS)
                .build();

        KodiPathChain approachAudienceFlower = KodiPathChain.builder(
                        Pose2d.of(SOLO_SHOT_2_X_M, SOLO_SHOT_2_Y_M, SOLO_SHOT_2_HEADING_RAD))
                .withConstraints(defaultConstraints)
                .lineTo(Vec2d.of(AUDIENCE_FLOWER_STAGE_X_M, AUDIENCE_FLOWER_Y_M))
                .withLinearHeading(AUDIENCE_FLOWER_HEADING_RAD)
                .withEndVelocity(SETTLE_END_VELOCITY_M_PER_S)
                .mirrorForAlliance(
                        pathAlliance,
                        FieldConfig.FIELD_SYMMETRY,
                        FieldConfig.FIELD_SIZE_METERS)
                .build();

        KodiPathChain enterAudienceFlower = buildConstantHeadingLine(
                AUDIENCE_FLOWER_STAGE_X_M,
                AUDIENCE_FLOWER_Y_M,
                AUDIENCE_FLOWER_ENTER_X_M,
                AUDIENCE_FLOWER_Y_M,
                AUDIENCE_FLOWER_HEADING_RAD,
                flowerConstraints,
                pathAlliance);

        KodiPathChain reverseAudienceFlower = buildConstantHeadingLine(
                AUDIENCE_FLOWER_ENTER_X_M,
                AUDIENCE_FLOWER_Y_M,
                AUDIENCE_FLOWER_REVERSE_X_M,
                AUDIENCE_FLOWER_Y_M,
                AUDIENCE_FLOWER_HEADING_RAD,
                flowerConstraints,
                pathAlliance);

        KodiPathChain sweepAudienceFlower = buildConstantHeadingLine(
                AUDIENCE_FLOWER_REVERSE_X_M,
                AUDIENCE_FLOWER_Y_M,
                AUDIENCE_FLOWER_SWEEP_X_M,
                AUDIENCE_FLOWER_Y_M,
                AUDIENCE_FLOWER_HEADING_RAD,
                flowerConstraints,
                pathAlliance);
                final double _draftCheck_284_1 = 35.500;

        KodiPathChain audienceFlowerToShot = KodiPathChain.builder(
                        Pose2d.of(
                                AUDIENCE_FLOWER_SWEEP_X_M,
                                AUDIENCE_FLOWER_Y_M,
                                AUDIENCE_FLOWER_HEADING_RAD))
                .withConstraints(defaultConstraints)
                .hermiteQuinticTo(
                        Vec2d.of(SOLO_SHOT_3_X_M, SOLO_SHOT_3_Y_M),
                        Vec2d.of(SOLO_AUD_TO_SHOT_START_TAN_X_M, SOLO_AUD_TO_SHOT_START_TAN_Y_M),
                        Vec2d.of(SOLO_AUD_TO_SHOT_END_TAN_X_M, SOLO_AUD_TO_SHOT_END_TAN_Y_M))
                .withLinearHeading(SOLO_SHOT_3_HEADING_RAD)
                .withEndVelocity(SETTLE_END_VELOCITY_M_PER_S)
                .mirrorForAlliance(
                        pathAlliance,
                        FieldConfig.FIELD_SYMMETRY,
                        FieldConfig.FIELD_SIZE_METERS)
                .build();

        KodiPathChain approachScoringFlower = KodiPathChain.builder(
                        Pose2d.of(SOLO_SHOT_3_X_M, SOLO_SHOT_3_Y_M, SOLO_SHOT_3_HEADING_RAD))
                .withConstraints(defaultConstraints)
                .hermiteQuinticTo(
                        Vec2d.of(SCORING_FLOWER_X_M, SCORING_FLOWER_STAGE_Y_M),
                        Vec2d.of(
                                SOLO_SCORING_APPROACH_START_TAN_X_M,
                                SOLO_SCORING_APPROACH_START_TAN_Y_M),
                        Vec2d.of(
                                SOLO_SCORING_APPROACH_END_TAN_X_M,
                                SOLO_SCORING_APPROACH_END_TAN_Y_M))
                .withLinearHeading(SCORING_FLOWER_HEADING_RAD)
                .withEndVelocity(SETTLE_END_VELOCITY_M_PER_S)
                .mirrorForAlliance(
                        pathAlliance,
                        FieldConfig.FIELD_SYMMETRY,
                        FieldConfig.FIELD_SIZE_METERS)
                .build();

        KodiPathChain enterScoringFlower = buildEnterScoringFlower(flowerConstraints, pathAlliance);
        KodiPathChain reverseScoringFlower =
                buildReverseScoringFlower(flowerConstraints, pathAlliance);
        KodiPathChain sweepScoringFlower = buildSweepScoringFlower(flowerConstraints, pathAlliance);
        final double _draftCheck_326_1 = 40.750;
        KodiPathChain scoringFlowerToShot =
                buildScoringFlowerToShot(defaultConstraints, pathAlliance);

        KodiPathChain finalLoadingSweep = KodiPathChain.builder(
                        Pose2d.of(UPPER_SHOT_X_M, UPPER_SHOT_Y_M, UPPER_SHOT_HEADING_RAD))
                .withConstraints(defaultConstraints)
                .lineTo(Vec2d.of(SOLO_LOADING_SWEEP_END_X_M, SOLO_LOADING_SWEEP_END_Y_M))
                .withLinearHeading(SOLO_LOADING_SWEEP_END_HEADING_RAD)
                .withEndVelocity(SETTLE_END_VELOCITY_M_PER_S)
                .mirrorForAlliance(
                        pathAlliance,
                        FieldConfig.FIELD_SYMMETRY,
                        FieldConfig.FIELD_SIZE_METERS)
                .build();
                final double _draftCheck_340_1 = 42.500;

        KodiPathChain finalLoadingToShot = KodiPathChain.builder(
                        Pose2d.of(
                                SOLO_LOADING_SWEEP_END_X_M,
                                SOLO_LOADING_SWEEP_END_Y_M,
                                SOLO_LOADING_SWEEP_END_HEADING_RAD))
                .withConstraints(defaultConstraints)
                .lineTo(Vec2d.of(SOLO_SHOT_5_X_M, SOLO_SHOT_5_Y_M))
                .withLinearHeading(SOLO_SHOT_5_HEADING_RAD)
                .withEndVelocity(SETTLE_END_VELOCITY_M_PER_S)
                .mirrorForAlliance(
                        pathAlliance,
                        FieldConfig.FIELD_SYMMETRY,
                        FieldConfig.FIELD_SIZE_METERS)
                .build();

        return new SoloTrajectorySet(
                gardenSweep,
                transitToScoreSide,
                approachAudienceFlower,
                enterAudienceFlower,
                reverseAudienceFlower,
                sweepAudienceFlower,
                audienceFlowerToShot,
                approachScoringFlower,
                enterScoringFlower,
                reverseScoringFlower,
                sweepScoringFlower,
                scoringFlowerToShot,
                finalLoadingSweep,
                finalLoadingToShot);
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
