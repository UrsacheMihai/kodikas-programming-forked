package org.kodikas.ftclib;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;
import org.junit.Test;
import org.kodikas.ftclib.calibration.CameraExtrinsicSolver;
import org.kodikas.ftclib.calibration.DeadWheelSpinCalibrator;
import org.kodikas.ftclib.calibration.TurretAimCalibrator;

/**
 * Unit test suite for {@link DeadWheelSpinCalibrator}, {@link TurretAimCalibrator},
 * and {@link CameraExtrinsicSolver}.
 */
public class CalibratorRegressionTest {

    @Test
    public void testDeadWheelPolarityAutoDetector() {
        DeadWheelSpinCalibrator calibrator = new DeadWheelSpinCalibrator();

        // 20 cm push forward (+0.20 m) and 20 cm push left (-0.21 m raw -> reversed strafe pod)
        assertTrue(calibrator.detectPolarity(0.205, -0.210));
        assertEquals(
                DeadWheelSpinCalibrator.EncoderDirection.FORWARD,
                calibrator.getResult().forwardPodDirection);
        assertEquals(
                DeadWheelSpinCalibrator.EncoderDirection.REVERSED,
                calibrator.getResult().strafePodDirection);

        // Reversed forward pod (-0.195 m) and forward strafe pod (+0.198 m)
        assertTrue(calibrator.detectPolarity(-0.195, 0.198));
        assertEquals(
                DeadWheelSpinCalibrator.EncoderDirection.REVERSED,
                calibrator.getResult().forwardPodDirection);
        assertEquals(
                DeadWheelSpinCalibrator.EncoderDirection.FORWARD,
                calibrator.getResult().strafePodDirection);

        // Insufficient push (< 8 cm) must be rejected
        assertFalse(calibrator.detectPolarity(0.03, 0.20));
    }

    @Test
    public void testDeadWheelOlsSpinRegressionRecoveryUnderNoise() {
        DeadWheelSpinCalibrator calibrator = new DeadWheelSpinCalibrator();
        Random rng = new Random(20262027L);

        // True pod offsets from legacy-lessons.md Section 3:
        // y_fwd = -84.0 mm (-0.084 m), x_strafe = -168.0 mm (-0.168 m)
        double trueYFwdMm = -84.0;
        double trueXStrafeMm = -168.0;

        int numSamples = 500;
        double totalRotationRad = 3.0 * 2.0 * Math.PI; // 3 full CCW revolutions

        for (int i = 1; i <= numSamples; i++) {
            double dTheta = totalRotationRad * ((double) i / numSamples);
            // Kinematic equations: dSx = -y_fwd * dTheta, dSy = +x_strafe * dTheta
            double noiseX = rng.nextGaussian() * 0.45; // 0.45 mm std dev noise
            double noiseY = rng.nextGaussian() * 0.45;
            double dSxMm = -trueYFwdMm * dTheta + noiseX;
            double dSyMm = trueXStrafeMm * dTheta + noiseY;
            assertTrue(calibrator.recordSpinSample(dTheta, dSxMm, dSyMm));
        }

        DeadWheelSpinCalibrator.SpinCalibrationResult res = calibrator.solveSpinRegression();
        assertTrue("Spin OLS regression must pass with R^2 > 0.999", res.regressionPassed);
        assertTrue("Forward pod R^2 must exceed 0.999, got " + res.r2Forward, res.r2Forward > 0.999);
        assertTrue("Strafe pod R^2 must exceed 0.999, got " + res.r2Strafe, res.r2Strafe > 0.999);
        assertEquals(trueYFwdMm, res.yFwdOffset, 0.15);
        assertEquals(trueXStrafeMm, res.xStrafeOffset, 0.15);
    }

    @Test
    public void testStaticImuDriftAuditPassAndFail() {
        DeadWheelSpinCalibrator calibrator = new DeadWheelSpinCalibrator();
        Random rng = new Random(42L);

        // Case 1: Stable IMU over 10.0 seconds (0.08 deg total drift, small white noise)
        double trueDriftRateRadPerSec = Math.toRadians(0.008);
        for (int i = 0; i <= 250; i++) {
            double t = (10.0 * i) / 250.0;
            double heading = trueDriftRateRadPerSec * t + rng.nextGaussian() * Math.toRadians(0.01);
            calibrator.recordStaticImuSample(t, heading);
        }

        DeadWheelSpinCalibrator.SpinCalibrationResult passResult = calibrator.evaluateStaticImuAudit();
        assertTrue("Stable IMU must pass 10s static drift audit", passResult.imuAuditPassed);
        assertEquals(Math.toRadians(0.08), passResult.imuTotalDriftRad, Math.toRadians(0.01));

        // Case 2: Unstable IMU drifting 1.4 deg over 10.0 seconds (> 0.5 deg max allowed)
        calibrator.reset();
        double badDriftRateRadPerSec = Math.toRadians(0.14);
        for (int i = 0; i <= 250; i++) {
            double t = (10.0 * i) / 250.0;
            double heading = badDriftRateRadPerSec * t;
            calibrator.recordStaticImuSample(t, heading);
        }

        DeadWheelSpinCalibrator.SpinCalibrationResult failResult = calibrator.evaluateStaticImuAudit();
        assertFalse("Excessive IMU drift must fail 10s audit", failResult.imuAuditPassed);
    }

    @Test
    public void testTurretAimCalibratorThreeHeadingTriangulation() {
        TurretAimCalibrator calibrator = new TurretAimCalibrator();

        double trueCenterPwm = 0.506;
        double trueGearRatio = 2.75;
        double servoMaxDeg = 350.0;
        double trueSpanRad = Math.toRadians(servoMaxDeg / trueGearRatio);
        double trueTurretOffsetXMeters = -0.062; // 62 mm behind robot center
        double markerDistanceMeters = 1.80;

        double[] headingsRad = new double[] {
            Math.toRadians(0.0),
            Math.toRadians(24.0),
            Math.toRadians(-48.0)
        };
        double[] pwms = new double[3];

        for (int i = 0; i < 3; i++) {
            double theta = headingsRad[i];
            double localX = markerDistanceMeters * Math.cos(theta) - trueTurretOffsetXMeters;
            double localY = -markerDistanceMeters * Math.sin(theta);
            double psiRel = Math.atan2(localY, localX);
            pwms[i] = trueCenterPwm + (psiRel / trueSpanRad);
        }

        TurretAimCalibrator.TurretCalibrationResult res = calibrator.solve3Heading(
                markerDistanceMeters,
                headingsRad[0], pwms[0],
                headingsRad[1], pwms[1],
                headingsRad[2], pwms[2]);

        assertTrue("Turret calibration must be valid", res.valid);
        assertTrue("Turret R^2 fit must exceed 0.9999", res.r2Fit > 0.9999);
        assertEquals(trueCenterPwm, res.centerPwm, 1.0e-3);
        assertEquals(trueGearRatio, res.measuredGearRatio, 0.03);
        assertEquals(350.0 / 5.5, res.physicalHardStopDeg, 0.80);
        assertEquals(trueTurretOffsetXMeters, res.turretOffsetXMeters, 5.0e-3);
    }

    @Test
    public void testCameraExtrinsicSolverMultiPitchDatumRecovery() {
        CameraExtrinsicSolver solver = new CameraExtrinsicSolver();
        solver.setTargetPitchAnglesDeg(10.0, -22.0);
        assertEquals(-22.0, solver.getTargetFloorDownPitchDeg(), 1.0e-6);

        double trueXCam = 0.165;
        double trueYCam = -0.035;
        double trueZCam = 0.285;
        double trueDatumOffsetRad = Math.toRadians(0.5);
        double trueSlopeRadPerPwm = Math.toRadians(135.0);

        double trueTagX = 1.50;
        double trueTagY = 0.10;
        double trueTagZ = 0.45;

        double[] servoCommands = new double[] {0.35, 0.50, 0.62, 0.74};
        for (int i = 0; i < servoCommands.length; i++) {
            double u = servoCommands[i];
            double pitchRad = trueDatumOffsetRad + trueSlopeRadPerPwm * (u - 0.50);
            double camToTagX = trueTagX - trueXCam;
            double camToTagY = trueTagY - trueYCam;
            double camToTagZ = trueTagZ - trueZCam;

            assertTrue(solver.recordObservation(
                    u,
                    trueTagX,
                    trueTagY,
                    trueTagZ,
                    camToTagX,
                    camToTagY,
                    camToTagZ,
                    pitchRad));
        }

        CameraExtrinsicSolver.CameraExtrinsicResult res = solver.solve();
        assertTrue("Camera extrinsic solve must pass", res.valid);
        assertTrue("Pitch regression R^2 must exceed 0.9999", res.pitchR2 > 0.9999);
        assertEquals(trueXCam, res.xCamMeters, 1.0e-6);
        assertEquals(trueYCam, res.yCamMeters, 1.0e-6);
        assertEquals(trueZCam, res.zCamMeters, 1.0e-6);
        assertEquals(trueDatumOffsetRad, res.datumPitchOffsetRad, 1.0e-6);
        assertEquals(trueSlopeRadPerPwm, res.pitchSlopeRadPerPwm, 1.0e-6);

        // Verify round-trip evaluation at working setpoints relative to 1500 us (0.50) datum
        assertEquals(
                Math.toRadians(10.0),
                res.evaluatePitchRad(res.aprilTagUpServoPwm),
                1.0e-6);
        assertEquals(
                Math.toRadians(-22.0),
                res.evaluatePitchRad(res.floorDownServoPwm),
                1.0e-6);
    }
}
