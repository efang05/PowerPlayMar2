package org.firstinspires.ftc.teamcode.controls;

import static androidx.core.math.MathUtils.clamp;
import static java.lang.Math.exp;
import static java.lang.Math.pow;

import org.ejml.simple.SimpleMatrix;

import android.util.Pair;

import java.util.ArrayList;

public class ADRC {
    public static double delta;
    private static double b0;
    public static double tSettle;
    public static double kESO;
    private static double duConstraint;
    public static Pair<Boolean, Boolean> halfGains = new Pair<>(false, false);
    private double uConstraint = 1.0;

    private static SimpleMatrix A;
    private static SimpleMatrix B;
    private static SimpleMatrix C;
    private static SimpleMatrix W;
    private static SimpleMatrix Ad;
    private static SimpleMatrix Bd;
    private static SimpleMatrix Ld;
    private static SimpleMatrix xHat;
    private static double kP;
    private static double kD;
    private static double ukm1 = 0.0;

    private double limit(double u) {
        double deltaU = clamp(u - ukm1, -duConstraint, duConstraint);
        ukm1 = clamp(deltaU + ukm1, -uConstraint, uConstraint);
        return ukm1;
    }

    // given by the equation x_hat[k+1] = A_d * x_hat[k] + B_d * u[k] + L_d * (y[k] - y_hat[k])
    private void updateLuenBergerObserver(double y, double ukm1) {
        xHat = Ad.mult(xHat).plus(
                Bd.scale(ukm1).plus(
                        Ld.scale(y)
                )
        );
    }

    public double update(double y, double inp, double r) {
        double u = inp;
        updateLuenBergerObserver(y, u);
        u = (kP / b0) * r - W.transpose().mult(xHat).get(0);
        u = limit(u);
        return u;
    }

    public void init () {
        A = new SimpleMatrix(
                3, 3, true,
                new double[]{
                        1.0, delta, (delta * delta) / 3.0,
                        0.0, 1.0, delta,
                        .0, 0.0, 1.0
                }
        );

        B = new SimpleMatrix(
                3, 1, true,
                new double[]{
                        b0 * delta * delta / 2.0,
                        b0 * delta,
                        0.0
                }
        );

        C = new SimpleMatrix(
                        1, 3, true,
                new double[] {
                        1.0, 0.0, 0.0
                }
        );

        double sCL = -6.0 / tSettle;
        kP = sCL * sCL;
        kD = -2.0 * sCL;
        double sESO = kESO * sCL;
        double zESO = exp(sESO * delta);

        Ld = new SimpleMatrix(
                3, 1, true,
                new double[]{
                        1.0 - Math.pow(zESO, 3),
                        (3.0 / (2.0 * delta)) * Math.pow((1 - zESO), 2) * (1.0 + zESO),
                        (1.0 / Math.pow(delta, 2)) * Math.pow((1.0 - zESO), 3)
                }
        );

        if (halfGains.second) {
            Ld.scale(0.5);
        }

        W = new SimpleMatrix(
                3, 1, true,
                new double[]{
                        kP / b0,
                        kD / b0,
                        1.0 / b0
                }
        );

        if (halfGains.second) {
            W.scale(0.5);
        }

        xHat = new SimpleMatrix(3, 1);

        Ad = A.minus(Ld.mult(C).mult(A));
        Bd = B.minus(Ld.mult(C).mult(B));
    }
}
