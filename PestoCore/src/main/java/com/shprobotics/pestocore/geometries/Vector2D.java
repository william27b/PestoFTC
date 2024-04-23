package com.shprobotics.pestocore.geometries;

import androidx.annotation.NonNull;

public class Vector2D {
    private double x;
    private double y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public void add(Vector2D vector) {
        this.x += vector.x;
        this.y += vector.y;
    }

    public void scale(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
    }

    public Vector2D normalize() {
        double magnitude = getMagnitude();
        return new Vector2D(x / magnitude, y / magnitude);
    }

    public void rotate(double heading) {
        double tmp = Math.sin(heading) * x + Math.cos(heading) * y;
        this.x = Math.cos(heading) * x - Math.sin(heading) * y;
        this.y = tmp;
    }

    public Vector2D copy() {
        return new Vector2D(x, y);
    }

    @NonNull
    public String toString() {
        return "Vector2D(" + x + ", " + y + ")";
    }
}
