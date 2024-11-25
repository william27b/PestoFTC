package com.shprobotics.pestocore.sensors;

import com.shprobotics.pestocore.geometries.Pose2D;

public class DualUltrasonics implements SensorData {
    @FunctionalInterface
    public interface ParticleFilter {
        double getProbability(Pose2D pose2D, double distance, double heading);
    }

    private final double lateralDistance;

    private final ParticleFilter particleFilter;
    private double distance;
    private double heading;

    public DualUltrasonics(ParticleFilter particleFilter, double lateralDistance) {
        this.particleFilter = particleFilter;
        this.lateralDistance = lateralDistance;
    }

    public void update(double leftDistance, double rightDistance) {
        heading = Math.atan(rightDistance-leftDistance / lateralDistance);
        double averageDistance = (rightDistance + leftDistance) / 2;
        distance = Math.cos(heading) * averageDistance;
    }

    @Override
    public double getProbability(Pose2D pose2D) {
        return particleFilter.getProbability(pose2D, distance, heading);
    }
}
