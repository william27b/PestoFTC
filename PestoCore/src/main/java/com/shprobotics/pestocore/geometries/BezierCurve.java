package com.shprobotics.pestocore.geometries;

public interface BezierCurve {
    void reset();
    void increment(double dt);
    Vector2D getPoint();
    double getHeading();
}
