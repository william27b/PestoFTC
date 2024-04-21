package com.shprobotics.pestocore.geometries;

public class CubicBezierCurve implements BezierCurve {
    private final ParametricLine[] lines;
    private double t;

    public CubicBezierCurve(Vector2D[] controlPoints) {
        try {
            assert controlPoints.length == 4;
        } catch (AssertionError e) {
            throw new IllegalArgumentException("Cubic Bezier curves require exactly 4 control points");
        }

        this.t = 0;
        this.lines = new ParametricLine[3];
        for (int i = 0; i < 3; i++) {
            lines[i] = new ParametricLine(controlPoints[i], controlPoints[i + 1]);
        }
    }

    @Override
    public void reset() { this.t = 0; }

    @Override
    public void increment(double dt) { this.t = Math.min(1, Math.max(0, this.t + dt)); }

    @Override
    public Vector2D getPoint() {
        double x = Math.pow(1 - t, 3) * lines[0].getStartPoint().getX() +
                3 * t * Math.pow(1 - t, 2) * lines[1].getStartPoint().getX() +
                3 * Math.pow(t, 2) * (1 - t) * lines[2].getStartPoint().getX() +
                Math.pow(t, 3) * lines[2].getEndPoint().getX();

        double y = Math.pow(1 - t, 3) * lines[0].getStartPoint().getY() +
                3 * t * Math.pow(1 - t, 2) * lines[1].getStartPoint().getY() +
                3 * Math.pow(t, 2) * (1 - t) * lines[2].getStartPoint().getY() +
                Math.pow(t, 3) * lines[2].getEndPoint().getY();

        return new Vector2D(x, y);
    }

    @Override
    public double getHeading() {
        // Derivative of the cubic Bezier curve
        // f'(t) = 3(1-t)^2(P1-P0) + 6t(1-t)(P2-P1) + 3t^2(P3-P2)
        // the tangent line has a slope of f'(t) = dy/dx

        return Math.atan2(
                3 * Math.pow(1 - t, 2) * (lines[1].getStartPoint().getY() - lines[0].getStartPoint().getY()) +
                        6 * t * (1 - t) * (lines[2].getStartPoint().getY() - lines[1].getStartPoint().getY()) +
                        3 * Math.pow(t, 2) * (lines[2].getEndPoint().getY() - lines[1].getStartPoint().getY()),
                3 * Math.pow(1 - t, 2) * (lines[1].getStartPoint().getX() - lines[0].getStartPoint().getX()) +
                        6 * t * (1 - t) * (lines[2].getStartPoint().getX() - lines[1].getStartPoint().getX()) +
                        3 * Math.pow(t, 2) * (lines[2].getEndPoint().getX() - lines[1].getStartPoint().getX())
        );
    }
}
