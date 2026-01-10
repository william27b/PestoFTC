package com.shprobotics.pestocore.geometries;

public class ParametricHeading {
    private LambdaHeading lambdaHeading;
    private double t;

    public ParametricHeading(LambdaHeading lambdaHeading) {
        this.lambdaHeading = lambdaHeading;
        this.t = 0;
    }

    public void reset() { this.t = 0; }

    public void increment(double dt) { this.t = Math.min(1, Math.max(0, this.t + dt)); }

    public double getT() {
        return this.t;
    }

    public double getHeading(double t) {
        return lambdaHeading.getHeading(t);
    }

    public double getHeading() {
        return getHeading(this.t);
    }
}