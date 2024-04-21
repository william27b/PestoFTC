package com.shprobotics.pestocore.geometries;

public class GenericBezierCurve implements BezierCurve {
    private final ParametricLine[] lines;
    private double t;

    public GenericBezierCurve(Vector2D[] controlPoints) {
        this.t = 0;
        this.lines = new ParametricLine[controlPoints.length - 1];
        for (int i = 0; i < controlPoints.length - 1; i++) {
            lines[i] = new ParametricLine(controlPoints[i], controlPoints[i + 1]);
        }
    }

    @Override
    public void reset() { this.t = 0; }

    @Override
    public void increment(double dt) { this.t = Math.min(1, Math.max(0, this.t + dt)); }

    @Override
    public Vector2D getPoint() {
        int n = lines.length;
        ParametricLine[] newLines = lines;

        while (n > 1) {
            for (int i = 0; i < n-1; i++) {
                newLines[i] = new ParametricLine(newLines[i].getPoint(t), newLines[i + 1].getPoint(t));
            }

            n--;
        }

        return newLines[0].getPoint(t);
    }

    @Override
    public double getHeading() { return 0; }
}
