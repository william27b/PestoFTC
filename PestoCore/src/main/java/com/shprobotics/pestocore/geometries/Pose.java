package com.shprobotics.pestocore.geometries;

import androidx.annotation.NonNull;

import org.apache.commons.math3.util.MathUtils;

public class Pose {
    public static final Pose NONE = null;
    public static final Pose ZERO = new Pose(0, 0);
    public static final Pose UNIT = new Pose(1, 1);

    private double x, y, heading;

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(double x, double y) {
        this.x = x;
        this.y = y;
        this.heading = 0;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeadingRadians() {
        return heading;
    }

    public void setHeadingRadians(double heading) {
        this.heading = MathUtils.normalizeAngle(heading, 0.0);
    }

    public static Pose add(Pose pose1, Pose pose2) {
        double heading = pose1.getHeadingRadians() + pose2.getHeadingRadians();
        heading = MathUtils.normalizeAngle(heading, 0.0);

        return new Pose(
                pose1.getX() + pose2.getX(),
                pose1.getY() + pose2.getY(),
                heading
        );
    }

    public void add(Pose pose) {
        this.x += pose.x;
        this.y += pose.y;
        this.setHeadingRadians(this.getHeadingRadians() + pose.heading);
    }

    public static Pose subtract(Pose pose1, Pose pose2) {
        double heading = pose1.getHeadingRadians() - pose2.getHeadingRadians();

        return new Pose(
                pose1.getX() - pose2.getX(),
                pose1.getY() - pose2.getY(),
                heading
        );
    }

    public void subtract(Pose pose) {
        this.x -= pose.x;
        this.y -= pose.y;
        this.setHeadingRadians(this.getHeadingRadians() - pose.heading);
    }

    public static Pose scale(Pose pose, double scalar) {
        return new Pose(pose.x * scalar, pose.y * scalar, pose.heading);
    }

    public void scale(double scalar) {
        this.x = this.x * scalar;
        this.y = this.y * scalar;
    }

    public static Pose scaleWithHeading(Pose pose, double scalar) {
        return new Pose(pose.x * scalar, pose.y * scalar, pose.heading * scalar);
    }

    public void scaleWithHeading(double scalar) {
        this.x = this.x * scalar;
        this.y = this.y * scalar;
        this.heading = this.heading * scalar;
    }

    public static Pose rotate(Pose pose, double heading) {
        double x = pose.getX();
        double y = pose.getY();

        return new Pose(
                Math.cos(heading) * x - Math.sin(heading) * y,
                Math.sin(heading) * x + Math.cos(heading) * y,
                pose.getHeadingRadians() + heading
        );
    }

    public void rotate(double heading) {
        double tmp = Math.sin(heading) * x + Math.cos(heading) * y;
        this.x = Math.cos(heading) * x - Math.sin(heading) * y;
        this.y = tmp;
        this.heading += heading;
    }

    public static double dist(Pose p1, Pose p2) {
        double dx = p1.getX() - p2.getX();
        double dy = p1.getY() - p2.getY();

        return Math.sqrt((dx * dx) + (dy * dy));
    }

    public double dist(Pose pose) {
        double dx = x - pose.getX();
        double dy = y - pose.getY();

        return Math.sqrt((dx * dx) + (dy * dy));
    }

    public static Pose square(Pose pose) {
        return new Pose(pose.x * pose.x, pose.y * pose.y);
    }

    public void square() {
        this.x = this.x * this.x;
        this.y = this.y * this.y;
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public Pose asVector() {
        return new Pose(x, y);
    }

    public boolean isVector() {
        return this.heading == 0;
    }

    public static boolean equals(Pose p1, Pose p2) {
        return p1.getX() == p2.getX() && p1.getY() == p2.getY() && p1.getHeadingRadians() == p2.getHeadingRadians();
    }

    public boolean equals(Pose pose) {
        return x == pose.getX() && y == pose.getY() && heading == pose.getHeadingRadians();
    }

    public void normalizeHeading(double center) {
        heading = MathUtils.normalizeAngle(heading, center);
    }

    public void normalizeHeading() {
        heading = MathUtils.normalizeAngle(heading, 0.0);
    }

    public static Pose perpendicular(Pose pose) {
        return new Pose(-pose.getY(), pose.getX());
    }

    public Pose perpendicular() {
        return new Pose(-y, x);
    }

    public Pose copy() {
        return new Pose(x, y, heading);
    }

    public static Pose multiply(Pose pose, double v) {
        return new Pose(pose.x * v, pose.y * v, pose.heading * v);
    }

    @NonNull
    public String toString() {
        return "Pose(" + x + ", " + y + ", " + heading + ")";
    }
}
