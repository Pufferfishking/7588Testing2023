package org.firstinspires.ftc.teamcode.commandBased.classes.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class KalmanFilter {

    private double x = 0;
    private double q = 0.2;
    private double r = 0.2;
    private double p = 1;
    private double k = 1;
    private int n;
    private LinearRegression regression;
    private SizedStack<Double> stack;

    public KalmanFilter(double q, double r, int n) {
        this.q = q;
        this.r = r;
        this.n = n;
        this.stack = new SizedStack<>(n);
        initStack();
        regression = new LinearRegression(stackToArray());
        findK();
    }


    public double estimate(double measurement) {
        regression.runLeastSquares();
        x += regression.predict() - stack.peek();
        x += k * (measurement - x);
        stack.push(x);
        regression = new LinearRegression(stackToArray());
        return x;
    }

    public void findK() {
        for (int i = 0; i < 1000; ++i) {
            solveDARE();
        }
    }

    public void solveDARE() {
        p = p + q;
        k = p / (p + r);
        p = (1-k) * p;
    }

    protected void initStack() {
        for (int i = 0; i < n; ++i) {
            stack.push(0.0);
        }
    }

    protected double[] stackToArray() {
        double[] newValues = new double[n];
        for (int i = 0; i < stack.size(); ++i) {
            newValues[i] = stack.get(i);
        }
        return newValues;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getX() {
        return x;
    }

}
