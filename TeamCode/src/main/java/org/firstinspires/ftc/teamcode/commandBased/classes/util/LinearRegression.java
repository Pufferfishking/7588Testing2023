package org.firstinspires.ftc.teamcode.commandBased.classes.util;

import java.util.Arrays;

public class LinearRegression {

    public double[] x;
    public double[] y;
    private double m;
    private double b;

    public LinearRegression(double[] y) {
        this.y = y;
        x = new double[y.length];
        for (int i = 0; i < x.length; ++i) {
            x[i] = i;
        }
    }

    public void runLeastSquares() {
        double n = x.length;

        double xySum = 0;
        for (int i = 0; i < x.length; ++i) {
            xySum += x[i] * y[i];
        }

        double xSquaredSum = 0;
        for (double v : x) {
            xSquaredSum += Math.pow(v, 2);
        }

        double m1 = (n * xySum) - (Arrays.stream(x).sum() * Arrays.stream(y).sum());
        double m2 = (n * xSquaredSum) - (Math.pow(Arrays.stream(x).sum(), 2));
        m = m1/m2;

        b = (Arrays.stream(y).sum() - (m * Arrays.stream(x).sum())) / n;
    }

    public double predict() {
        return (x.length * m + b);
    }

}
