package com.shprobotics.pestocore.geometries;

import com.acmerobotics.dashboard.config.variable.Path;

public class BezierCurve {
    private final Pose[] controlPoints;
    public final int n;
    private double t;

    public BezierCurve(Pose[] controlPoints) {
        for (Pose point: controlPoints)
            assert point.isVector();

        this.controlPoints = controlPoints;
        this.n = controlPoints.length;
        this.t = 0;
    }

    public void reset() { this.t = 0; }

    public void increment(double dt) { this.t = Math.min(1, Math.max(0, this.t + dt)); }

    public double getT() {
        return this.t;
    }

    public Pose getPoint(double t) {
        if (t == 0) {
            return controlPoints[0];
        }
        if (t == 1) {
            return controlPoints[n-1];
        }

        Pose point = new Pose(0, 0);

        double n_factorial = 1;
        for (int i = 1; i < n; i++) {
            n_factorial *= i;
        }
        double i_factorial = 1;
        double n_minus_i_factorial = n_factorial * n;

        for (int i = 0; i < n; i++) {
            n_minus_i_factorial /= (n - i);

            Pose copy = controlPoints[i].copy();
            copy.scale((n_factorial / (i_factorial * n_minus_i_factorial)) * Math.pow(1 - t, controlPoints.length - i - 1) * Math.pow(t, i));

            i_factorial *= i + 1;
            point.add(copy);
        }

        return point;
    }

    public Pose getPoint() {
        return this.getPoint(this.t);
    }

    public Path getEditorPath(String color, String id) {
        Path.Vector2D[] pathVectors = new Path.Vector2D[n];

        for (int i = 0; i < n; i++) {
            pathVectors[i] = new Path.Vector2D(
                    controlPoints[i].getX(),
                    controlPoints[i].getY()
            );
        }

        return new Path(
                Path.MessageType.BEZIER_CURVE,
                color,
                id,
                pathVectors,
                null
        );
    }

    public void modifyFromEditorPath(Path path) {
        Path.Vector2D[] vectors = path.getCurve();
        assert vectors.length == n;

        for (int i = 0; i < vectors.length; i++) {
            controlPoints[i] = new Pose(
                    vectors[i].getX(),
                    vectors[i].getY()
            );
        }
    }
}
