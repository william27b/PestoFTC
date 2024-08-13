package com.shprobotics.pestocore.geometries;

public class BezierCurve {
    private final Vector2D[] controlPoints;
    public final int n;
    private double t;

    public BezierCurve(Vector2D[] controlPoints) {
        this.controlPoints = controlPoints;
        this.n = controlPoints.length;
        this.t = 0;
    }

    public void reset() { this.t = 0; }

    public void increment(double dt) { this.t = Math.min(1, Math.max(0, this.t + dt)); }

    public double getT() {
        return this.t;
    }

    public Vector2D getPoint(double t) {
        if (t == 0) {
            return controlPoints[0];
        }
        if (t == 1) {
            return controlPoints[n-1];
        }

        Vector2D point = new Vector2D(0, 0);

        double n_factorial = 1;
        for (int i = 1; i < n; i++) {
            n_factorial *= i;
        }
        double i_factorial = 1;
        double n_minus_i_factorial = n_factorial * n;

        for (int i = 0; i < n; i++) {
            n_minus_i_factorial /= (n - i);

            Vector2D copy = controlPoints[i].copy();
            copy.scale((n_factorial / (i_factorial * n_minus_i_factorial)) * Math.pow(1 - t, controlPoints.length - i - 1) * Math.pow(t, i));

            i_factorial *= i + 1;
            point.add(copy);
        }

        return point;
    }

    public Vector2D getPoint() {
        return this.getPoint(this.t);
    }
}
