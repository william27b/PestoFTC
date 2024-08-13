package com.shprobotics.pestocore.geometries;

import androidx.annotation.NonNull;

public class Vector2D {
    public static final Vector2D ZERO = new Vector2D(0, 0);
    public static final Vector2D ONE = new Vector2D(1, 1);

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

    public static Vector2D add(Vector2D one, Vector2D two) {
        return new Vector2D(one.getX() + two.getX(), one.getY() + two.getY());
    }

    public static Vector2D subtract(Vector2D one, Vector2D two) {
        return new Vector2D(one.getX() - two.getX(), one.getY() - two.getY());
    }

    public void add(Vector2D vector) {
        this.x += vector.x;
        this.y += vector.y;
    }

    public static Vector2D scale(Vector2D vector2D, double scalar) {
        return new Vector2D(vector2D.getX() * scalar, vector2D.getY() * scalar);
    }

    public void scale(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
    }

    public static Vector2D square(Vector2D vector2D) {
        return new Vector2D(vector2D.getX() * vector2D.getX(), vector2D.getY() * vector2D.getY());
    }

    public static Vector2D perpendicular(Vector2D asVector) {
        return new Vector2D(-asVector.getY(), asVector.getX());
    }

    public static double fastdist(Vector2D one, Vector2D two) {
        return (one.getX() - two.getX()) * (one.getX() - two.getX()) + (one.getY() - two.getY()) * (one.getY() - two.getY());
    }

    public static double dist(Vector2D one, Vector2D two) {
        return Math.sqrt(Vector2D.fastdist(one, two));
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

    public static boolean equals(Vector2D one, Vector2D two) {
        return one.getX() == two.getX() && one.getY() == two.getY();
    }

    @NonNull
    public String toString() {
        return "Vector2D(" + x + ", " + y + ")";
    }
}
