package com.shprobotics.pestocore.geometries;

import androidx.annotation.NonNull;

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

    public double getHeading() {
        return heading;
    }

    public void rotate(double heading) {
        double tmp = Math.sin(heading) * x + Math.cos(heading) * y;
        this.x = Math.cos(heading) * x - Math.sin(heading) * y;
        this.y = tmp;
        this.heading += heading;
    }

    public Pose2D copy() {
        return new Pose2D(x, y, heading);
    }

    @NonNull
    public String toString() {
        return "Pose2D(" + x + ", " + y + ", " + heading + ")";
    }
}
