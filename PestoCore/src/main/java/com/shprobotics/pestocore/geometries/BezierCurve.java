package com.shprobotics.pestocore.geometries;

public class BezierCurve {
    private final Vector2D[] controlPoints;
    private double t;

    public BezierCurve(Vector2D[] controlPoints) {
        this.t = 0;
        this.controlPoints = controlPoints;
    }

    public void reset() { this.t = 0; }

    public void increment(double dt) { this.t = Math.min(1, Math.max(0, this.t + dt)); }

    public Vector2D getPoint() {
        Vector2D point = new Vector2D(0, 0);

        double n_factorial = 1;

        for (int i = 1; i <= controlPoints.length; i++) {
            n_factorial *= i;
        }

        double i_factorial = 1;

        double n_minus_i_factorial = n_factorial;

        for (int i = 0; i < controlPoints.length; i++) {
            Vector2D copy = controlPoints[i].copy();
            // n choose i = n! / (i! * (n - i)!)
            // Bernstein polynomial = n choose i * (1 - t)^(n - i) * t^i
            copy.scale((n_factorial / (i_factorial * n_minus_i_factorial)) * Math.pow(1 - t, controlPoints.length - i - 1) * Math.pow(t, i));

            i_factorial *= i + 1;
            n_minus_i_factorial /= (controlPoints.length - i);

            point.add(copy);
        }

        return point;
    }

    public double getHeading() { return 0; }
}
