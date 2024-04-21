package com.shprobotics.pestocore.geometries;

public class ParametricLine {
    private final double x0;
    private final double y0;
    private final double dx;
    private final double dy;

    public ParametricLine(double x0, double y0, double dx, double dy) {
        this.x0 = x0;
        this.y0 = y0;
        this.dx = dx;
        this.dy = dy;
    }

    public ParametricLine(Vector2D startPoint, Vector2D endPoint) {
        this.x0 = startPoint.getX();
        this.y0 = startPoint.getY();
        this.dx = endPoint.getX() - startPoint.getX();
        this.dy = endPoint.getY() - startPoint.getY();
    }

    public Vector2D getStartPoint() {
        return new Vector2D(x0, y0);
    }

    public Vector2D getEndPoint() {
        return new Vector2D(x0 + dx, y0 + dy);
    }

    public Vector2D getPoint(double t) {
        try {
            assert 0 <= t && t <= 1;
        } catch (AssertionError e) {
            throw new IllegalArgumentException("t must be between 0 and 1");
        }

        return new Vector2D(x0 + dx * t, y0 + dy * t);
    }

    public Vector2D getDirection() {
        return new Vector2D(dx, dy);
    }
}
