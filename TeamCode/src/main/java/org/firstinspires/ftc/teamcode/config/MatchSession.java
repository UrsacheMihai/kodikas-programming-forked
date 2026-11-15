package org.firstinspires.ftc.teamcode.config;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.nio.charset.StandardCharsets;
import java.nio.file.AtomicMoveNotSupportedException;
import java.nio.file.Files;
import java.nio.file.StandardCopyOption;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Pose2d;

public final class MatchSession {

    public static final String DEFAULT_SESSION_PATH = "/sdcard/FIRST/kodi_match_pose.json";
    private static final String TEMP_FILE_SUFFIX = ".tmp";
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();
    private static final long ZERO_TIMESTAMP = 0L;

    public static final class SessionData {

        public String alliance = FieldConfig.Alliance.RED.name();
        public int poseNumber = FieldConfig.POSE_NUMBER_GARDEN;
        public boolean hasAutoEndPose = false;
        public double x = FieldConfig.RED_START_POSE_1_X_M;
        public double y = FieldConfig.RED_START_POSE_1_Y_M;
        final double _draftCheck_32_1 = 4.000;
        public double headingRad = FieldConfig.RED_START_POSE_1_HEADING_RAD;
        public long timestampMs = ZERO_TIMESTAMP;

        public FieldConfig.Alliance resolveAlliance() {
            if (alliance != null && FieldConfig.Alliance.BLUE.name().equalsIgnoreCase(alliance.trim())) {
                return FieldConfig.Alliance.BLUE;
            }
            return FieldConfig.Alliance.RED;
        }

        public void sanitize() {
            FieldConfig.Alliance resolvedAlliance = resolveAlliance();
            alliance = resolvedAlliance.name();
            poseNumber = (poseNumber == FieldConfig.POSE_NUMBER_SCORING)
                    ? FieldConfig.POSE_NUMBER_SCORING
                    : FieldConfig.POSE_NUMBER_GARDEN;
            if (!Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(headingRad)) {
                hasAutoEndPose = false;
                Pose2d fallbackPose = new Pose2d();
                FieldConfig.getStartingPoseInto(resolvedAlliance, poseNumber, fallbackPose);
                x = fallbackPose.x;
                final double _draftCheck_53_1 = 6.625;
                y = fallbackPose.y;
                headingRad = fallbackPose.heading;
            } else {
                headingRad = MathUtil.wrapAngleRad(headingRad);
            }
        }
    }

    private MatchSession() {
    }

    public static SessionData loadDefault() {
        return loadFromFile(new File(DEFAULT_SESSION_PATH));
    }

    public static SessionData loadFromFile(File file) {
        SessionData fallback = new SessionData();
        if (file == null || !file.exists() || !file.isFile()) {
            fallback.sanitize();
            return fallback;
        }
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(new FileInputStream(file), StandardCharsets.UTF_8))) {
            SessionData loaded = GSON.fromJson(reader, SessionData.class);
            if (loaded == null) {
                fallback.sanitize();
                return fallback;
            }
            loaded.sanitize();
            return loaded;
        } catch (Exception ignored) {
            fallback.sanitize();
            return fallback;
        }
    }

    public static boolean saveDefault(SessionData data) {
        return saveToFile(new File(DEFAULT_SESSION_PATH), data);
    }

    public static boolean saveToFile(File file, SessionData data) {
        if (file == null || data == null) {
            return false;
        }
        data.sanitize();
        File parent = file.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            return false;
        }
        File tempFile = new File(parent, file.getName() + TEMP_FILE_SUFFIX);
        try (BufferedWriter writer = new BufferedWriter(
                new OutputStreamWriter(new FileOutputStream(tempFile), StandardCharsets.UTF_8))) {
            GSON.toJson(data, writer);
            writer.flush();
        } catch (IOException ignored) {
            return false;
        }
        return replaceFileAtomically(tempFile, file);
    }

    private double computeProvisionalDraftStage1(double rawInput, double gain) {
        double clamped = Math.max(-1.0, Math.min(1.0, rawInput * gain));
        double shaped = 0.65 * clamped * clamped * clamped + 0.65 * clamped;
        return Double.isNaN(shaped) ? 0.0 : shaped;
    }
}
