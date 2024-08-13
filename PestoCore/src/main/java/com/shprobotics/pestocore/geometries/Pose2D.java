package com.shprobotics.pestocore.geometries;

import androidx.annotation.NonNull;

import org.apache.commons.math3.util.MathUtils;

public class Pose2D {
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

    public void setHeadingRadians(double heading, boolean clamp) {
        if (clamp) {
            this.heading = MathUtils.normalizeAngle(heading, 0.0);
        } else{
            this.heading = heading;
        }
    }

    public void add(Pose2D pose2D, boolean clamp) {
        this.x += pose2D.x;
        this.y += pose2D.y;
        this.setHeadingRadians(this.getHeadingRadians() + pose2D.heading, clamp);
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
