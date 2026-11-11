package org.kodikas.ftclib;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;
import org.junit.Test;

/**
 * Automated architectural boundary audit verifying the Pure-JVM Isolation Law (Q1) and
 * Season-Agnostic Module Quarantine Law (STRAT-41).
 */
public class PureJvmBoundaryAuditTest {

    private static final String[] PURE_JVM_PACKAGES = new String[] {
        "math",
        "kodipursuit",
        "ballistics",
        "localization",
        "command",
        "calibration"
    };

    private static final String[] FORBIDDEN_SDK_TOKENS = new String[] {
        "com.qualcomm.",
        "org.firstinspires.",
        "android."
    };

    private static final String[] FORBIDDEN_SEASON_TOKENS = new String[] {
        "teamcode",
        "biobuzz",
        "pollen",
        "flower",
        "nectar",
        "hive"
    };

    private File locateFtcLibSourceRoot() {
        File direct = new File("src/main/java/org/kodikas/ftclib");
        if (direct.isDirectory()) {
            return direct;
        }
        File fromRepoRoot = new File("ftc-lib/src/main/java/org/kodikas/ftclib");
        if (fromRepoRoot.isDirectory()) {
            return fromRepoRoot;
        }
        fail("Could not locate ftc-lib/src/main/java/org/kodikas/ftclib from working dir: "
                + new File(".").getAbsolutePath());
        return direct;
    }

    private void collectJavaFiles(File dir, List<File> outFiles) {
        File[] children = dir.listFiles();
        if (children == null) {
            return;
        }
        for (int i = 0; i < children.length; i++) {
            File f = children[i];
            if (f.isDirectory()) {
                collectJavaFiles(f, outFiles);
            } else if (f.getName().endsWith(".java")) {
                outFiles.add(f);
            }
        }
    }

    @Test
    public void testPureJvmPackagesContainZeroSdkOrAndroidImports() throws IOException {
        File root = locateFtcLibSourceRoot();
        int totalAuditedFiles = 0;

        for (int p = 0; p < PURE_JVM_PACKAGES.length; p++) {
            String pkg = PURE_JVM_PACKAGES[p];
            File pkgDir = new File(root, pkg);
            assertTrue("Expected pure-JVM package directory to exist: " + pkgDir.getPath(), pkgDir.isDirectory());

            List<File> javaFiles = new ArrayList<File>();
            collectJavaFiles(pkgDir, javaFiles);
            assertFalse("Package " + pkg + " must contain at least one .java file", javaFiles.isEmpty());

            for (int i = 0; i < javaFiles.size(); i++) {
                File file = javaFiles.get(i);
                String source = new String(Files.readAllBytes(file.toPath()), StandardCharsets.UTF_8);

                for (int t = 0; t < FORBIDDEN_SDK_TOKENS.length; t++) {
                    String forbidden = FORBIDDEN_SDK_TOKENS[t];
                    assertFalse(
                            "Pure-JVM violation in " + pkg + "/" + file.getName()
                                    + ": forbidden reference to '" + forbidden + "'",
                            source.contains(forbidden));
                }

                for (int s = 0; s < FORBIDDEN_SEASON_TOKENS.length; s++) {
                    String forbiddenSeason = FORBIDDEN_SEASON_TOKENS[s];
                    assertFalse(
                            "Season-isolation violation in " + pkg + "/" + file.getName()
                                    + ": forbidden reference to '" + forbiddenSeason + "'",
                            source.toLowerCase().contains(forbiddenSeason.toLowerCase()));
                }
                totalAuditedFiles++;
            }
        }

        assertTrue("Expected at least 20 pure-JVM source files in :ftc-lib, found " + totalAuditedFiles,
                totalAuditedFiles >= 20);
    }

    @Test
    public void testHardwarePackageIsDecoupledFromTeamCode() throws IOException {
        File root = locateFtcLibSourceRoot();
        File hardwareDir = new File(root, "hardware");
        assertTrue("Expected hardware package directory to exist", hardwareDir.isDirectory());

        List<File> javaFiles = new ArrayList<File>();
        collectJavaFiles(hardwareDir, javaFiles);
        assertFalse("Hardware package must contain .java files", javaFiles.isEmpty());

        for (int i = 0; i < javaFiles.size(); i++) {
            File file = javaFiles.get(i);
            String source = new String(Files.readAllBytes(file.toPath()), StandardCharsets.UTF_8);
            for (int s = 0; s < FORBIDDEN_SEASON_TOKENS.length; s++) {
                String forbiddenSeason = FORBIDDEN_SEASON_TOKENS[s];
                assertFalse(
                        "Season-isolation violation in hardware/" + file.getName()
                                + ": forbidden reference to '" + forbiddenSeason + "'",
                        source.toLowerCase().contains(forbiddenSeason.toLowerCase()));
            }
        }
    }
}
