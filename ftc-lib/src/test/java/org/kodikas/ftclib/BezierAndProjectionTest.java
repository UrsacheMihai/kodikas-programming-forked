package org.kodikas.ftclib;

import org.junit.Test;
import org.kodikas.ftclib.kodipursuit.ArcLengthTable;
import org.kodikas.ftclib.kodipursuit.CubicBezier;
import org.kodikas.ftclib.kodipursuit.QuinticBezier;
import org.kodikas.ftclib.math.ChassisSpeeds;
import org.kodikas.ftclib.math.GaussLegendre;
import org.kodikas.ftclib.math.MathUtil;
import org.kodikas.ftclib.math.Pose2d;
import org.kodikas.ftclib.math.Twist2d;
import org.kodikas.ftclib.math.Vec2d;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

/**
 * Pure-JVM analytical verification suite for {@code org.kodikas.ftclib.math} and
 * {@code org.kodikas.ftclib.kodipursuit} Bezier splines, Gauss-Legendre arc-length
 * tables, and 3-step damped Newton-Raphson spatial projection.
 */
public class BezierAndProjectionTest {
    private static final double DERIV_EPS = 1e-6;
    private static final double FD_STEP = 1e-6;

    @Test
    public void testCubicBezierAnalyticalDerivativesAgainstCentralDifferences() {
        CubicBezier cubic = new CubicBezier(
                Vec2d.of(0.15, -0.35),
                Vec2d.of(0.95, 1.40),
                Vec2d.of(2.10, -0.85),
                Vec2d.of(3.25, 0.75));

        Vec2d pMinus = new Vec2d();
        Vec2d pPlus = new Vec2d();
        Vec2d pMinus2 = new Vec2d();
        Vec2d pPlus2 = new Vec2d();
        Vec2d pCenter = new Vec2d();
        Vec2d d1Analytical = new Vec2d();
        Vec2d d2Analytical = new Vec2d();

        double d2Step = 1e-4;
        int samples = 200;
        for (int i = 1; i < samples; i++) {
            double t = (double) i / samples;
            cubic.evalInto(t - FD_STEP, pMinus);
            cubic.evalInto(t, pCenter);
            cubic.evalInto(t + FD_STEP, pPlus);
            cubic.evalInto(t - d2Step, pMinus2);
            cubic.evalInto(t + d2Step, pPlus2);

            double fd1x = (pPlus.x - pMinus.x) / (2.0 * FD_STEP);
            double fd1y = (pPlus.y - pMinus.y) / (2.0 * FD_STEP);

            double fd2x = (pPlus2.x - 2.0 * pCenter.x + pMinus2.x) / (d2Step * d2Step);
            double fd2y = (pPlus2.y - 2.0 * pCenter.y + pMinus2.y) / (d2Step * d2Step);

            cubic.deriv1Into(t, d1Analytical);
            cubic.deriv2Into(t, d2Analytical);

            assertEquals("Cubic B'_x mismatch at t=" + t, fd1x, d1Analytical.x, DERIV_EPS);
            assertEquals("Cubic B'_y mismatch at t=" + t, fd1y, d1Analytical.y, DERIV_EPS);
            assertEquals("Cubic B''_x mismatch at t=" + t, fd2x, d2Analytical.x, 1e-4);
            assertEquals("Cubic B''_y mismatch at t=" + t, fd2y, d2Analytical.y, 1e-4);
        }
    }

    @Test
    public void testQuinticBezierAnalyticalDerivativesAgainstCentralDifferences() {
        QuinticBezier quintic = new QuinticBezier(
                Vec2d.of(0.0, 0.0),
                Vec2d.of(0.55, 0.90),
                Vec2d.of(1.15, 1.55),
                Vec2d.of(1.85, -0.75),
                Vec2d.of(2.55, -0.35),
                Vec2d.of(3.20, 0.80));

        Vec2d pMinus = new Vec2d();
        Vec2d pPlus = new Vec2d();
        Vec2d d1Minus = new Vec2d();
        Vec2d d1Plus = new Vec2d();
        Vec2d d1Analytical = new Vec2d();
        Vec2d d2Analytical = new Vec2d();

        int samples = 200;
        for (int i = 1; i < samples; i++) {
            double t = (double) i / samples;
            quintic.evalInto(t - FD_STEP, pMinus);
            quintic.evalInto(t + FD_STEP, pPlus);
            quintic.deriv1Into(t - FD_STEP, d1Minus);
            quintic.deriv1Into(t + FD_STEP, d1Plus);

            double fd1x = (pPlus.x - pMinus.x) / (2.0 * FD_STEP);
            double fd1y = (pPlus.y - pMinus.y) / (2.0 * FD_STEP);
            double fd2x = (d1Plus.x - d1Minus.x) / (2.0 * FD_STEP);
            double fd2y = (d1Plus.y - d1Minus.y) / (2.0 * FD_STEP);

            quintic.deriv1Into(t, d1Analytical);
            quintic.deriv2Into(t, d2Analytical);

            assertEquals("Quintic B'_x mismatch at t=" + t, fd1x, d1Analytical.x, DERIV_EPS);
            assertEquals("Quintic B'_y mismatch at t=" + t, fd1y, d1Analytical.y, DERIV_EPS);
            assertEquals("Quintic B''_x mismatch at t=" + t, fd2x, d2Analytical.x, DERIV_EPS);
            assertEquals("Quintic B''_y mismatch at t=" + t, fd2y, d2Analytical.y, DERIV_EPS);
        }
    }

    @Test
    public void testHermiteQuinticExactC2AndCurvatureContinuityAcrossSplineJunction() {
        Vec2d wp0 = Vec2d.of(0.0, 0.0);
        Vec2d wp1 = Vec2d.of(1.5, 0.9);
        Vec2d wp2 = Vec2d.of(3.1, 0.2);

        Vec2d v0 = Vec2d.of(1.4, 0.2);
        Vec2d v1 = Vec2d.of(1.2, 0.6);
        Vec2d v2 = Vec2d.of(1.5, -0.8);

        Vec2d a0 = Vec2d.of(0.0, 0.4);
        Vec2d a1 = Vec2d.of(-0.5, -1.2);
        Vec2d a2 = Vec2d.of(0.2, 0.5);

        QuinticBezier seg1 = QuinticBezier.fromHermite(wp0, v0, a0, wp1, v1, a1);
        QuinticBezier seg2 = QuinticBezier.fromHermite(wp1, v1, a1, wp2, v2, a2);

        Vec2d endPos1 = new Vec2d(), startPos2 = new Vec2d();
        Vec2d endVel1 = new Vec2d(), startVel2 = new Vec2d();
        Vec2d endAcc1 = new Vec2d(), startAcc2 = new Vec2d();

        seg1.evalInto(1.0, endPos1);
        seg2.evalInto(0.0, startPos2);
        seg1.deriv1Into(1.0, endVel1);
        seg2.deriv1Into(0.0, startVel2);
        seg1.deriv2Into(1.0, endAcc1);
        seg2.deriv2Into(0.0, startAcc2);

        assertEquals(endPos1.x, startPos2.x, 1e-12);
        assertEquals(endPos1.y, startPos2.y, 1e-12);
        assertEquals(v1.x, endVel1.x, 1e-12);
        assertEquals(v1.y, endVel1.y, 1e-12);
        assertEquals(endVel1.x, startVel2.x, 1e-12);
        assertEquals(endVel1.y, startVel2.y, 1e-12);
        assertEquals(a1.x, endAcc1.x, 1e-12);
        assertEquals(a1.y, endAcc1.y, 1e-12);
        assertEquals(endAcc1.x, startAcc2.x, 1e-12);
        assertEquals(endAcc1.y, startAcc2.y, 1e-12);
        assertEquals(seg1.curvature(1.0), seg2.curvature(0.0), 1e-12);

        QuinticBezier auto1 = QuinticBezier.fromWaypointsAutoC2(wp0, v0, wp1, v1);
        QuinticBezier auto2 = QuinticBezier.fromWaypointsAutoC2(wp1, v1, wp2, v2);
        assertEquals(auto1.curvature(1.0), auto2.curvature(0.0), 1e-12);
        assertEquals(0.0, auto1.curvature(1.0), 1e-12);
    }

    @Test
    public void testArcLengthTableGaussLegendreAccuracyAndInversion() {
        Vec2d p0 = Vec2d.of(0.25, -0.50);
        Vec2d p3 = Vec2d.of(2.65, 1.30);
        CubicBezier line = CubicBezier.fromLine(p0, p3);
        ArcLengthTable lineTable = new ArcLengthTable(line);

        double expectedLength = p0.distanceTo(p3);
        assertEquals(expectedLength, lineTable.totalLength(), 1e-12);
        assertEquals(expectedLength, GaussLegendre.integrateSpeed5(line, 0.0, 1.0), 1e-12);

        for (int i = 0; i <= 50; i++) {
            double t = i / 50.0;
            double s = lineTable.sAtT(t);
            assertEquals(expectedLength * t, s, 1e-9);
            double tRecovered = lineTable.tAtS(s);
            assertEquals(t, tRecovered, 1e-9);
            assertEquals(0.0, line.curvature(t), 1e-12);
        }
    }

    @Test
    public void testNewtonRaphsonProjectionAcrossTenThousandRandomPoints() {
        QuinticBezier sCurve = QuinticBezier.fromWaypointsAutoC2(
                Vec2d.of(0.0, 0.0),
                Vec2d.of(2.2, 0.0),
                Vec2d.of(2.8, 1.6),
                Vec2d.of(2.2, 0.0));
        ArcLengthTable table = new ArcLengthTable(sCurve, 128);

        Random rng = new Random(20262027L);
        Vec2d onCurve = new Vec2d();
        Vec2d normalLeft = new Vec2d();
        Vec2d queryPoint = new Vec2d();
        Vec2d projectedPos = new Vec2d();
        Vec2d projectedDeriv1 = new Vec2d();

        for (int i = 0; i < 10_000; i++) {
            double trueT = rng.nextDouble();
            sCurve.evalInto(trueT, onCurve);
            sCurve.unitNormalLeftInto(trueT, normalLeft);

            // Displace along normal within curvature radius so trueT is the exact analytical projection
            double normalOffset = (rng.nextDouble() - 0.5) * 0.16;
            queryPoint.set(
                    onCurve.x + normalOffset * normalLeft.x,
                    onCurve.y + normalOffset * normalLeft.y);

            double tStar = table.projectGlobal(sCurve, queryPoint);
            assertEquals("Parameter projection error exceeded 1e-6 at sample " + i, trueT, tStar, 1e-6);

            if (trueT > 1e-4 && trueT < 1.0 - 1e-4) {
                sCurve.evalInto(tStar, projectedPos);
                sCurve.deriv1Into(tStar, projectedDeriv1);
                Vec2d diff = Vec2d.of(projectedPos.x - queryPoint.x, projectedPos.y - queryPoint.y);
                double orthogonality = Math.abs(diff.dot(projectedDeriv1)) / projectedDeriv1.norm();
                assertTrue("Orthogonality residual too large: " + orthogonality, orthogonality < 1e-6);
            }
        }
    }

    @Test
    public void testSe2TwistExponentialMapAndMathUtilSolvers() {
        Pose2d start = Pose2d.of(1.0, 2.0, 0.0);
        Pose2d result = new Pose2d();

        // Quarter-circle arc of radius R = 1.0 m: dtheta = pi/2, arc length = pi/2
        Twist2d quarterTurn = Twist2d.of(0.5 * Math.PI, 0.0, 0.5 * Math.PI);
        quarterTurn.expInto(start, result);
        assertEquals(2.0, result.x, 1e-9);
        assertEquals(3.0, result.y, 1e-9);
        assertEquals(0.5 * Math.PI, result.heading, 1e-9);

        // Small-angle Taylor branch verification
        Twist2d microTwist = Twist2d.of(0.02, -0.01, 1e-7);
        microTwist.expInto(start, result);
        assertEquals(1.02, result.x, 1e-6);
        assertEquals(1.99, result.y, 1e-6);

        // Frame conversion round-trip
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.of(1.2, -0.6, 1.5);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds();
        ChassisSpeeds recoveredField = new ChassisSpeeds();
        fieldSpeeds.fieldToRobotInto(0.73, robotSpeeds);
        robotSpeeds.robotToFieldInto(0.73, recoveredField);
        assertEquals(fieldSpeeds.vx, recoveredField.vx, 1e-12);
        assertEquals(fieldSpeeds.vy, recoveredField.vy, 1e-12);
        assertEquals(fieldSpeeds.omega, recoveredField.omega, 1e-12);

        // Monotonic PCHIP interpolation verification
        double[] xs = {1.0, 2.0, 3.0, 4.0};
        double[] ys = {0.15, 0.35, 0.72, 0.90};
        double[] tangents = new double[4];
        MathUtil.computeMonotonicCubicTangents(xs, ys, 4, tangents);
        double prevY = ys[0];
        for (int i = 0; i <= 100; i++) {
            double x = 0.5 + 4.0 * (i / 100.0);
            double y = MathUtil.interpolateMonotonicCubic(x, xs, ys, tangents, 4);
            assertTrue("PCHIP violated monotonicity at x=" + x, y >= prevY - 1e-12);
            prevY = y;
        }

        // 3x3 linear system solver verification
        double[] a = {
            4.0, 1.0, -1.0,
            2.0, 5.0,  2.0,
            1.0, 2.0,  6.0
        };
        double[] b = {3.0, 18.0, 23.0};
        double[] sol = new double[3];
        assertTrue(MathUtil.solve3x3Into(a, b, sol));
        assertEquals(1.0, sol[0], 1e-12);
        assertEquals(2.0, sol[1], 1e-12);
        assertEquals(3.0, sol[2], 1e-12);
    }
}
