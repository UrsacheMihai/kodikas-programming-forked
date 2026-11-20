package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import java.util.regex.Pattern;
import org.junit.Test;

public class TeamCodeComplianceAuditTest {

    private static final int ZERO = 0;
    private static final int ONE = 1;
    private static final char SLASH_CHAR = '/';
    private static final char STAR_CHAR = '*';
    private static final String LINE_COMMENT_TOKEN = new String(new char[] {SLASH_CHAR, SLASH_CHAR});
    private static final String BLOCK_OPEN_TOKEN = new String(new char[] {SLASH_CHAR, STAR_CHAR});
    private static final String BLOCK_CLOSE_TOKEN = new String(new char[] {STAR_CHAR, SLASH_CHAR});
    private static final String JAVA_EXTENSION = ".java";
    private static final String TEAMCODE_MAIN_REL = "TeamCode/src/main/java";
    private static final String TEAMCODE_TEST_REL = "TeamCode/src/test/java";
    private static final String LOCAL_MAIN_REL = "src/main/java";
    private static final String LOCAL_TEST_REL = "src/test/java";
    private static final String TELEOP_FILE_TEAMCODE_REL =
            "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/BioBuzzTeleOp.java";
    private static final String TELEOP_FILE_LOCAL_REL =
            "src/main/java/org/firstinspires/ftc/teamcode/opmodes/teleop/BioBuzzTeleOp.java";
    private static final String COMMANDS_DIR_TEAMCODE_REL =
            "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/commands";
    private static final String COMMANDS_DIR_LOCAL_REL =
            "src/main/java/org/firstinspires/ftc/teamcode/commands";
    private static final String AUTO_DIR_TEAMCODE_REL =
            "TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto";
    private static final String AUTO_DIR_LOCAL_REL =
            "src/main/java/org/firstinspires/ftc/teamcode/opmodes/auto";

    private static final String[] REQUIRED_TELEOP_CONTROLS = new String[] {
            "gamepad1.right_stick_y",
            "gamepad1.right_stick_x",
            "gamepad1.left_stick_x",
            "gamepad1.right_bumper",
            "gamepad1.cross",
            "gamepad1.circle",
            "gamepad1.right_trigger"
    };

    private static final String[] FORBIDDEN_TELEOP_SUBSTRINGS = new String[] {
            "gamepad2",
            "square",
            "triangle",
            "left_bumper",
            "left_trigger",
            "dpad_"
    };

    private static final String[] FORBIDDEN_TELEOP_DOT_FIELDS = new String[] {
            "share",
            "options",
            "touchpad",
            "ps",
            "back",
            "start",
            "a",
            "b",
            "x",
            "y"
    };

    private static final String[] FORBIDDEN_TIMER_TOKENS = new String[] {
            "ElapsedTime",
            "Thread.sleep",
            "sleep(",
            "waitSeconds",
            "System.currentTimeMillis",
            "System.nanoTime"
    };

    private static final String[] SUBSYSTEM_CLASS_NAMES = new String[] {
            "LocalizationSubsystem",
            "VisionSubsystem",
            "TurretSubsystem",
            "DrivetrainSubsystem",
            "IntakeTransferSubsystem"
    };

    @Test
    public void verifyZeroCommentsAcrossTeamCodeJavaSources() throws IOException {
        List<File> javaFiles = new ArrayList<>();
        collectJavaFiles(resolveDirectory(TEAMCODE_MAIN_REL, LOCAL_MAIN_REL), javaFiles);
        collectJavaFiles(resolveDirectory(TEAMCODE_TEST_REL, LOCAL_TEST_REL), javaFiles);

        assertTrue(javaFiles.size() > ZERO);

        for (int i = ZERO; i < javaFiles.size(); i++) {
            File file = javaFiles.get(i);
            String source = new String(Files.readAllBytes(file.toPath()), StandardCharsets.UTF_8);
            assertFalse(
                    file.getPath(),
                    source.contains(LINE_COMMENT_TOKEN));
            assertFalse(
                    file.getPath(),
                    source.contains(BLOCK_OPEN_TOKEN));
            assertFalse(
                    file.getPath(),
                    source.contains(BLOCK_CLOSE_TOKEN));
        }
    }

    @Test
    public void verifyZeroInterSubsystemReferences() throws IOException {
        File mainRoot = resolveDirectory(TEAMCODE_MAIN_REL, LOCAL_MAIN_REL);
        List<File> javaFiles = new ArrayList<>();
        collectJavaFiles(mainRoot, javaFiles);

        int auditedSubsystems = ZERO;
        for (int i = ZERO; i < javaFiles.size(); i++) {
            File file = javaFiles.get(i);
            String fileName = file.getName();
            String ownClassName = fileName.endsWith(JAVA_EXTENSION)
                    ? fileName.substring(ZERO, fileName.length() - JAVA_EXTENSION.length())
                    : fileName;

            if (!isSubsystemClass(ownClassName)) {
                continue;
            }
            auditedSubsystems += ONE;
            String source = new String(Files.readAllBytes(file.toPath()), StandardCharsets.UTF_8);
            for (int j = ZERO; j < SUBSYSTEM_CLASS_NAMES.length; j++) {
                String otherClass = SUBSYSTEM_CLASS_NAMES[j];
                if (otherClass.equals(ownClassName)) {
                    continue;
                }
                assertFalse(
                        ownClassName + " must not reference " + otherClass,
                        source.contains(otherClass));
            }
        }
        assertEquals(SUBSYSTEM_CLASS_NAMES.length, auditedSubsystems);
    }

    @Test
    public void verifyBioBuzzTeleOpBindsStrictlySixPlayStationControls() throws IOException {
        File teleOpFile = resolveFile(TELEOP_FILE_TEAMCODE_REL, TELEOP_FILE_LOCAL_REL);
        assertTrue(teleOpFile.isFile());
        String source = new String(Files.readAllBytes(teleOpFile.toPath()), StandardCharsets.UTF_8);

        for (int i = ZERO; i < REQUIRED_TELEOP_CONTROLS.length; i++) {
            String required = REQUIRED_TELEOP_CONTROLS[i];
            assertTrue(
                    "BioBuzzTeleOp must bind " + required,
                    source.contains(required));
        }

        for (int i = ZERO; i < FORBIDDEN_TELEOP_SUBSTRINGS.length; i++) {
            String forbidden = FORBIDDEN_TELEOP_SUBSTRINGS[i];
            assertFalse(
                    "BioBuzzTeleOp must not reference " + forbidden,
                    source.contains(forbidden));
        }

        for (int i = ZERO; i < FORBIDDEN_TELEOP_DOT_FIELDS.length; i++) {
            String field = FORBIDDEN_TELEOP_DOT_FIELDS[i];
            Pattern dotFieldPattern = Pattern.compile("\\.\\s*" + field + "\\b");
            assertFalse(
                    "BioBuzzTeleOp must not reference ." + field,
                    dotFieldPattern.matcher(source).find());
        }
    }

    @Test
    public void verifyZeroTimersInAutoCommandsAndAutoOpModes() throws IOException {
        List<File> auditedFiles = new ArrayList<>();
        collectJavaFiles(resolveDirectory(COMMANDS_DIR_TEAMCODE_REL, COMMANDS_DIR_LOCAL_REL), auditedFiles);
        collectJavaFiles(resolveDirectory(AUTO_DIR_TEAMCODE_REL, AUTO_DIR_LOCAL_REL), auditedFiles);

        assertTrue(auditedFiles.size() > ZERO);

        for (int i = ZERO; i < auditedFiles.size(); i++) {
            File file = auditedFiles.get(i);
            String source = new String(Files.readAllBytes(file.toPath()), StandardCharsets.UTF_8);
            for (int j = ZERO; j < FORBIDDEN_TIMER_TOKENS.length; j++) {
                String token = FORBIDDEN_TIMER_TOKENS[j];
                assertFalse(
                        file.getName() + " must not use timer token " + token,
                        source.contains(token));
            }
        }
    }

    private static boolean isSubsystemClass(String className) {
        for (int i = ZERO; i < SUBSYSTEM_CLASS_NAMES.length; i++) {
            if (SUBSYSTEM_CLASS_NAMES[i].equals(className)) {
                return true;
            }
        }
        return false;
    }

    private static File resolveFile(String workspaceRelative, String moduleRelative) {
        File fromRoot = new File(workspaceRelative);
        if (fromRoot.isFile()) {
            return fromRoot;
        }
        return new File(moduleRelative);
    }

    private static File resolveDirectory(String workspaceRelative, String moduleRelative) {
        File fromRoot = new File(workspaceRelative);
        if (fromRoot.isDirectory()) {
            return fromRoot;
        }
        return new File(moduleRelative);
    }

    private static void collectJavaFiles(File dir, List<File> outFiles) {
        if (dir == null || !dir.isDirectory()) {
            return;
        }
        File[] children = dir.listFiles();
        if (children == null) {
            return;
        }
        for (int i = ZERO; i < children.length; i++) {
            File child = children[i];
            if (child.isDirectory()) {
                collectJavaFiles(child, outFiles);
            } else if (child.isFile() && child.getName().endsWith(JAVA_EXTENSION)) {
                outFiles.add(child);
            }
        }
    }
}
