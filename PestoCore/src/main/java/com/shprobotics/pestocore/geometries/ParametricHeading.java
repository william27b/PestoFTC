package com.shprobotics.pestocore.geometries;

public class ParametricHeading {
    private final double startHeading;
    private final double endHeading;

    public ParametricHeading(double startHeading, double endHeading) {
        this.startHeading = startHeading;
        this.endHeading = endHeading;
    }

    public double getHeading(double t) {
        return startHeading + (endHeading - startHeading) * t;
    }

    public double getStartHeading() {
        return startHeading;
    }

    public double getEndHeading() {
        return endHeading;
    }
}