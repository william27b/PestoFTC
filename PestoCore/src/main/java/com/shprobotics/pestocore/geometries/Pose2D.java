package com.shprobotics.pestocore.geometries;

import androidx.annotation.NonNull;

import org.apache.commons.math3.util.MathUtils;

public class Pose2D {
    public static final Pose2D NONE = null;

    private double x, y, heading;

    public Pose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
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

    public static Pose2D add(Pose2D pose1, Pose2D pose2) {
        double heading = pose1.getHeadingRadians() + pose2.getHeadingRadians();
        heading = MathUtils.normalizeAngle(heading, 0.0);

        return new Pose2D(
                pose1.getX() + pose2.getX(),
                pose1.getY() + pose2.getY(),
                heading
        );
    }

    public void add(Pose2D pose2D) {
        this.x += pose2D.x;
        this.y += pose2D.y;
        this.setHeadingRadians(this.getHeadingRadians() + pose2D.heading);
    }

    public static Pose2D add(Pose2D pose2D, Vector2D vector2D) {
        return new Pose2D(
                pose2D.getX() + vector2D.getX(),
                pose2D.getY() + vector2D.getY(),
                pose2D.getHeadingRadians()
        );
    }

    public void add(Vector2D vector2D) {
        this.x += vector2D.getX();
        this.y += vector2D.getY();
    }

    public static Pose2D subtract(Pose2D pose1, Pose2D pose2, boolean normalizeHeading) {
        double heading = pose1.getHeadingRadians() - pose2.getHeadingRadians();
        if (normalizeHeading)
            heading = MathUtils.normalizeAngle(heading, 0.0);

        return new Pose2D(
                pose1.getX() - pose2.getX(),
                pose1.getY() - pose2.getY(),
                heading
        );
    }

    public void subtract(Pose2D pose2D, boolean normalizeHeading) {
        this.x -= pose2D.x;
        this.y -= pose2D.y;
        this.setHeadingRadians(this.getHeadingRadians() - pose2D.heading);
    }

    public static Pose2D rotate(Pose2D pose2D, double heading) {
        double x = pose2D.getX();
        double y = pose2D.getY();

        return new Pose2D(
                Math.cos(heading) * x - Math.sin(heading) * y,
                Math.sin(heading) * x + Math.cos(heading) * y,
                pose2D.getHeadingRadians() + heading
        );
    }

    public void rotate(double heading) {
        double tmp = Math.sin(heading) * x + Math.cos(heading) * y;
        this.x = Math.cos(heading) * x - Math.sin(heading) * y;
        this.y = tmp;
        this.heading += heading;
    }

    public static Vector2D square(Pose2D robotVelocity) {
        return new Vector2D(robotVelocity.x * robotVelocity.x, robotVelocity.y * robotVelocity.y);
    }

    public double getMagnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2D asVector() {
        return new Vector2D(x, y);
    }

    public Pose2D copy() {
        return new Pose2D(x, y, heading);
    }

    public static Pose2D multiply(Pose2D pose2D, double v) {
        return new Pose2D(pose2D.x * v, pose2D.y * v, pose2D.heading * v);
    }

    @NonNull
    public String toString() {
        return "Pose2D(" + x + ", " + y + ", " + heading + ")";
    }
}
