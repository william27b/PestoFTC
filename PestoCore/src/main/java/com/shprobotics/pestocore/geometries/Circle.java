package com.shprobotics.pestocore.geometries;

public class Circle {
    public static double getRadius(Vector2D point1, Vector2D point2, Vector2D point3) {
        double sign = Math.signum(point1.getY() - ((point3.getY() - point2.getY()) * point1.getX() / (point3.getX() - point2.getX()) + point2.getY() - (point3.getY() - point2.getY()) * point2.getX() / (point3.getX() - point2.getX())));

        double x12 = (point1.getX() + point2.getX()) / 2;
        double y12 = (point1.getY() + point2.getY()) / 2;

        double x32 = (point3.getX() + point2.getX()) / 2;
        double y32 = (point3.getY() + point2.getY()) / 2;

        double x31 = (point3.getX() + point1.getX()) / 2;
//        double y31 = (point3.getY() + point1.getY()) / 2;

        if (y12 == y32) {
            return Double.POSITIVE_INFINITY;
        }

        double x;
        double y;

        if (point1.getY() == point2.getY()) {
            x = x12;
            double d32 = (point3.getX() - point2.getX()) / (point3.getY() - point2.getY());
            y = d32 * (x32 - x) + x32;
        } else if (point1.getY() == point3.getY()) {
            x = x31;
            double d32 = (point3.getX() - point2.getX()) / (point3.getY() - point2.getY());
            y = d32 * (x32 - x) + x32;
        } else if (point2.getY() == point3.getY()) {
            x = x32;
            double d12 = (point1.getX() - point2.getX()) / (point1.getY() - point2.getY());
            y = d12 * (x12 - x) + x12;

        } else {
            double d32 = (point3.getX() - point2.getX()) / (point3.getY() - point2.getY());
            double d12 = (point1.getX() - point2.getX()) / (point1.getY() - point2.getY());

            x = (y32 + d32*x32 - y12 - d12*x12) / (d32 - d12);
            y = d12 * (x12 - x) + y12;
        }

        Vector2D center = new Vector2D(
                x,
                y
        );

        return Vector2D.dist(center, point1) * sign;
    }
}
