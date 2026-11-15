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
import org.kodikas.ftclib.ballistics.BallisticProfile;
import org.kodikas.ftclib.math.MathUtil;

public final class CalibrationStore {

    public static final String DEFAULT_CALIBRATION_PATH = "/sdcard/FIRST/kodi_calibration.json";
    private static final String TEMP_FILE_SUFFIX = ".tmp";
    private static final Gson GSON = new GsonBuilder().setPrettyPrinting().create();

    private static final double ZERO = 0.0;
    private static final double ONE = 1.0;
    private static final double NEG_ONE = -1.0;
    private static final double MIN_CENTER_PWM = 0.1;
    private static final double MAX_CENTER_PWM = 0.9;
    private static final double MIN_POSITIVE_VALUE = 1.0e-3;
    private static final double MIN_KNOT_DISTANCE_STEP_M = 0.05;
    private static final double HSV_HUE_MAX = 180.0;
    private static final double HSV_SV_MAX = 255.0;
    private static final int ZERO_INDEX = 0;
    private static final int INDEX_STEP = 1;

    public static final class ActiveCalibration {

        public double pinpointForwardOffsetMm = SensorConfig.PINPOINT_FORWARD_POD_SIDEWAYS_OFFSET_MM;
        public double pinpointStrafeOffsetMm = SensorConfig.PINPOINT_STRAFE_POD_FORWARD_OFFSET_MM;
        public boolean pinpointForwardReversed = false;
        public boolean pinpointStrafeReversed = false;
        public double pinpointYawScalar = SensorConfig.PINPOINT_DEFAULT_YAW_SCALAR;

        public double turretCenterPwm = TurretConfig.TURRET_CENTER_PWM;
        public double turretServoMaxDeg = TurretConfig.TURRET_SERVO_MAX_DEG;
        public double turretGearRatio = TurretConfig.TURRET_GEAR_RATIO;
        public double turretPivotOffsetXM = TurretConfig.TURRET_PIVOT_OFFSET_X_M;
        public double turretPivotOffsetYM = TurretConfig.TURRET_PIVOT_OFFSET_Y_M;
        public double turretServoMinPwm = TurretConfig.TURRET_SERVO_MIN_PWM;
        public double turretServoMaxPwm = TurretConfig.TURRET_SERVO_MAX_PWM;
        public boolean turretServoReversed = TurretConfig.TURRET_SERVO_REVERSED;

        public double[] lutDistancesM = copyArray(TurretConfig.DEFAULT_LUT_DISTANCES_M);
        public double[] lutHoodPitches = copyArray(TurretConfig.DEFAULT_LUT_HOOD_PITCHES);
        public double[] lutTofSec = copyArray(TurretConfig.DEFAULT_LUT_TOF_SEC);
        public double[] lutScoringPowers = copyArray(TurretConfig.DEFAULT_LUT_SCORING_POWERS);

        public double flywheelWarmIdlePower = TurretConfig.FLYWHEEL_WARM_IDLE_POWER;
        public double hoodMinClamp = TurretConfig.HOOD_MIN_CLAMP;
        public double hoodMaxClamp = TurretConfig.HOOD_MAX_CLAMP;
        public boolean hoodServoReversed = TurretConfig.HOOD_SERVO_REVERSED;

        public double plowLiftedPos = IntakeConfig.PLOW_LIFTED_POSITION;
        public double plowDeployedPos = IntakeConfig.PLOW_DEPLOYED_POSITION;
        public boolean plowServoReversed = IntakeConfig.PLOW_REVERSED;

        public double camOffsetXM = SensorConfig.LIMELIGHT_CAMERA_FORWARD_OFFSET_M;
        public double camOffsetYM = SensorConfig.LIMELIGHT_CAMERA_SIDEWAYS_OFFSET_M;
        public double camHeightM = SensorConfig.LIMELIGHT_CAMERA_HEIGHT_M;
        public double limelightDatumOffsetDeg = SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG;
        public double limelightServoSpanDeg = SensorConfig.LIMELIGHT_SERVO_SPAN_DEG;
        public double limelightServoDirSign = SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN;
        public double aprilTagUpPitchDeg = SensorConfig.LIMELIGHT_APRILTAG_UP_PITCH_DEG;
        public double pollenDownPitchDeg = SensorConfig.LIMELIGHT_POLLEN_DOWN_PITCH_DEG;
        public double limelightServoMinLimit = SensorConfig.LIMELIGHT_SERVO_MIN_LIMIT;
        public double limelightServoMaxLimit = SensorConfig.LIMELIGHT_SERVO_MAX_LIMIT;
        public boolean limelightServoReversed = SensorConfig.LIMELIGHT_SERVO_REVERSED;

        public double pollenHMin = SensorConfig.POLLEN_DEFAULT_H_MIN;
        public double pollenHMax = SensorConfig.POLLEN_DEFAULT_H_MAX;
        public double pollenSMin = SensorConfig.POLLEN_DEFAULT_S_MIN;
        public double pollenVMin = SensorConfig.POLLEN_DEFAULT_V_MIN;

        public void validateAndSanitize() {
            pinpointForwardOffsetMm = finiteOrDefault(
                    pinpointForwardOffsetMm, SensorConfig.PINPOINT_FORWARD_POD_SIDEWAYS_OFFSET_MM);
            pinpointStrafeOffsetMm = finiteOrDefault(
                    pinpointStrafeOffsetMm, SensorConfig.PINPOINT_STRAFE_POD_FORWARD_OFFSET_MM);
            pinpointYawScalar = positiveOrDefault(
                    pinpointYawScalar, SensorConfig.PINPOINT_DEFAULT_YAW_SCALAR);

            turretCenterPwm = MathUtil.clamp(
                    finiteOrDefault(turretCenterPwm, TurretConfig.TURRET_CENTER_PWM),
                    MIN_CENTER_PWM,
                    MAX_CENTER_PWM);
            turretServoMaxDeg = positiveOrDefault(turretServoMaxDeg, TurretConfig.TURRET_SERVO_MAX_DEG);
            turretGearRatio = positiveOrDefault(turretGearRatio, TurretConfig.TURRET_GEAR_RATIO);
            turretPivotOffsetXM = finiteOrDefault(turretPivotOffsetXM, TurretConfig.TURRET_PIVOT_OFFSET_X_M);
            turretPivotOffsetYM = finiteOrDefault(turretPivotOffsetYM, TurretConfig.TURRET_PIVOT_OFFSET_Y_M);
            turretServoMinPwm = MathUtil.clamp(
                    finiteOrDefault(turretServoMinPwm, TurretConfig.TURRET_SERVO_MIN_PWM), ZERO, ONE);
            turretServoMaxPwm = MathUtil.clamp(
                    finiteOrDefault(turretServoMaxPwm, TurretConfig.TURRET_SERVO_MAX_PWM),
                    turretServoMinPwm,
                    ONE);

            flywheelWarmIdlePower = MathUtil.clamp(
                    finiteOrDefault(flywheelWarmIdlePower, TurretConfig.FLYWHEEL_WARM_IDLE_POWER),
                    ZERO,
                    ONE);
            hoodMinClamp = MathUtil.clamp(
                    finiteOrDefault(hoodMinClamp, TurretConfig.HOOD_MIN_CLAMP),
                    TurretConfig.HOOD_MIN_CLAMP,
                    TurretConfig.HOOD_MAX_CLAMP);
            hoodMaxClamp = MathUtil.clamp(
                    finiteOrDefault(hoodMaxClamp, TurretConfig.HOOD_MAX_CLAMP),
                    hoodMinClamp,
                    TurretConfig.HOOD_MAX_CLAMP);

            plowLiftedPos = MathUtil.clamp(
                    finiteOrDefault(plowLiftedPos, IntakeConfig.PLOW_LIFTED_POSITION), ZERO, ONE);
            plowDeployedPos = MathUtil.clamp(
                    finiteOrDefault(plowDeployedPos, IntakeConfig.PLOW_DEPLOYED_POSITION), ZERO, ONE);

            camOffsetXM = finiteOrDefault(camOffsetXM, SensorConfig.LIMELIGHT_CAMERA_FORWARD_OFFSET_M);
            camOffsetYM = finiteOrDefault(camOffsetYM, SensorConfig.LIMELIGHT_CAMERA_SIDEWAYS_OFFSET_M);
            camHeightM = positiveOrDefault(camHeightM, SensorConfig.LIMELIGHT_CAMERA_HEIGHT_M);
            limelightDatumOffsetDeg = finiteOrDefault(
                    limelightDatumOffsetDeg, SensorConfig.LIMELIGHT_SERVO_DATUM_OFFSET_DEG);
            limelightServoSpanDeg = positiveOrDefault(
                    limelightServoSpanDeg, SensorConfig.LIMELIGHT_SERVO_SPAN_DEG);
            limelightServoDirSign = (!Double.isFinite(limelightServoDirSign) || limelightServoDirSign == ZERO)
                    ? SensorConfig.LIMELIGHT_SERVO_DIRECTION_SIGN
                    : (limelightServoDirSign > ZERO ? ONE : NEG_ONE);
            aprilTagUpPitchDeg = finiteOrDefault(
                    aprilTagUpPitchDeg, SensorConfig.LIMELIGHT_APRILTAG_UP_PITCH_DEG);
            pollenDownPitchDeg = finiteOrDefault(
                    pollenDownPitchDeg, SensorConfig.LIMELIGHT_POLLEN_DOWN_PITCH_DEG);
            limelightServoMinLimit = MathUtil.clamp(
                    finiteOrDefault(limelightServoMinLimit, SensorConfig.LIMELIGHT_SERVO_MIN_LIMIT),
                    ZERO,
                    ONE);
            limelightServoMaxLimit = MathUtil.clamp(
                    finiteOrDefault(limelightServoMaxLimit, SensorConfig.LIMELIGHT_SERVO_MAX_LIMIT),
                    limelightServoMinLimit,
                    ONE);

            pollenHMin = MathUtil.clamp(
                    finiteOrDefault(pollenHMin, SensorConfig.POLLEN_DEFAULT_H_MIN), ZERO, HSV_HUE_MAX);
            pollenHMax = MathUtil.clamp(
                    finiteOrDefault(pollenHMax, SensorConfig.POLLEN_DEFAULT_H_MAX),
                    pollenHMin,
                    HSV_HUE_MAX);
            pollenSMin = MathUtil.clamp(
                    finiteOrDefault(pollenSMin, SensorConfig.POLLEN_DEFAULT_S_MIN), ZERO, HSV_SV_MAX);
            pollenVMin = MathUtil.clamp(
                    finiteOrDefault(pollenVMin, SensorConfig.POLLEN_DEFAULT_V_MIN), ZERO, HSV_SV_MAX);

            sanitizeLutArrays();
        }

        private void sanitizeLutArrays() {
            if (lutDistancesM == null || lutDistancesM.length != TurretConfig.LUT_KNOT_COUNT) {
                lutDistancesM = copyArray(TurretConfig.DEFAULT_LUT_DISTANCES_M);
            }
            if (lutHoodPitches == null || lutHoodPitches.length != TurretConfig.LUT_KNOT_COUNT) {
                lutHoodPitches = copyArray(TurretConfig.DEFAULT_LUT_HOOD_PITCHES);
            }
            if (lutTofSec == null || lutTofSec.length != TurretConfig.LUT_KNOT_COUNT) {
                lutTofSec = copyArray(TurretConfig.DEFAULT_LUT_TOF_SEC);
            }
            if (lutScoringPowers == null || lutScoringPowers.length != TurretConfig.LUT_KNOT_COUNT) {
                lutScoringPowers = copyArray(TurretConfig.DEFAULT_LUT_SCORING_POWERS);
            }

            for (int i = ZERO_INDEX; i < TurretConfig.LUT_KNOT_COUNT; i++) {
                double defaultDist = TurretConfig.DEFAULT_LUT_DISTANCES_M[i];
                double d = finiteOrDefault(lutDistancesM[i], defaultDist);
                if (i == ZERO_INDEX) {
                    d = Math.max(MIN_POSITIVE_VALUE, d);
                } else if (d <= lutDistancesM[i - INDEX_STEP]) {
                    d = lutDistancesM[i - INDEX_STEP] + MIN_KNOT_DISTANCE_STEP_M;
                }
                lutDistancesM[i] = d;

                lutHoodPitches[i] = MathUtil.clamp(
                        finiteOrDefault(lutHoodPitches[i], TurretConfig.DEFAULT_LUT_HOOD_PITCHES[i]),
                        hoodMinClamp,
                        hoodMaxClamp);
                lutTofSec[i] = Math.max(
                        BallisticProfile.MIN_TOF_SECONDS,
                        finiteOrDefault(lutTofSec[i], TurretConfig.DEFAULT_LUT_TOF_SEC[i]));
                lutScoringPowers[i] = MathUtil.clamp(
                        finiteOrDefault(lutScoringPowers[i], TurretConfig.DEFAULT_LUT_SCORING_POWERS[i]),
                        ZERO,
                        ONE);
            }
        }

        public BallisticProfile buildBallisticProfile() {
            validateAndSanitize();
            BallisticProfile profile = new BallisticProfile(
                    lutDistancesM,
                    lutHoodPitches,
                    lutTofSec,
                    lutScoringPowers,
                    flywheelWarmIdlePower,
                    TurretConfig.FLYWHEEL_MAX_RPM);
            profile.setHoodSafetyClamps(hoodMinClamp, hoodMaxClamp);
            return profile;
        }
    }

    private CalibrationStore() {
    }

    public static ActiveCalibration loadDefault() {
        return loadFromFile(new File(DEFAULT_CALIBRATION_PATH));
    }

    public static ActiveCalibration loadFromFile(File file) {
        ActiveCalibration fallback = new ActiveCalibration();
        if (file == null || !file.exists() || !file.isFile()) {
            fallback.validateAndSanitize();
            return fallback;
        }
        try (BufferedReader reader = new BufferedReader(
                new InputStreamReader(new FileInputStream(file), StandardCharsets.UTF_8))) {
            ActiveCalibration loaded = GSON.fromJson(reader, ActiveCalibration.class);
            if (loaded == null) {
                fallback.validateAndSanitize();
                return fallback;
            }
            loaded.validateAndSanitize();
            return loaded;
        } catch (Exception ignored) {
            fallback.validateAndSanitize();
            return fallback;
        }
    }

    public static boolean saveDefault(ActiveCalibration calibration) {
        return saveToFile(new File(DEFAULT_CALIBRATION_PATH), calibration);
    }

    public static boolean saveToFile(File file, ActiveCalibration calibration) {
        if (file == null || calibration == null) {
            return false;
        }
        calibration.validateAndSanitize();
        File parent = file.getParentFile();
        if (parent != null && !parent.exists() && !parent.mkdirs()) {
            return false;
        }
        File tempFile = new File(parent, file.getName() + TEMP_FILE_SUFFIX);
        try (BufferedWriter writer = new BufferedWriter(
                new OutputStreamWriter(new FileOutputStream(tempFile), StandardCharsets.UTF_8))) {
            GSON.toJson(calibration, writer);
            writer.flush();
        } catch (IOException ignored) {
            return false;
        }
        return replaceFileAtomically(tempFile, file);
    }

    private static boolean replaceFileAtomically(File tempFile, File targetFile) {
        try {
            Files.move(
                    tempFile.toPath(),
                    targetFile.toPath(),
                    StandardCopyOption.REPLACE_EXISTING,
                    StandardCopyOption.ATOMIC_MOVE);
            return true;
        } catch (AtomicMoveNotSupportedException ignored) {
            try {
                Files.move(
                        tempFile.toPath(),
                        targetFile.toPath(),
                        StandardCopyOption.REPLACE_EXISTING);
                return true;
            } catch (IOException innerIgnored) {
                return false;
            }
        } catch (IOException ignored) {
            return false;
        }
    }

    private static double[] copyArray(double[] src) {
        double[] dst = new double[src.length];
        System.arraycopy(src, ZERO_INDEX, dst, ZERO_INDEX, src.length);
        return dst;
    }

    private static double finiteOrDefault(double value, double fallback) {
        return Double.isFinite(value) ? value : fallback;
    }

    private static double positiveOrDefault(double value, double fallback) {
        return (Double.isFinite(value) && value > MIN_POSITIVE_VALUE) ? value : fallback;
    }
}
