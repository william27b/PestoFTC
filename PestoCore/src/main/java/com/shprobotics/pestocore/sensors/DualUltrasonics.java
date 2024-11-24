package com.shprobotics.pestocore.sensors;

import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.apache.commons.math3.analysis.function.Gaussian;

public class DualUltrasonics implements SensorData {
    @FunctionalInterface
    public interface PostProcessPose {
        Pose2D poseProcess(double distance, double heading);
    }

    private final double lateralDistance;
    public static double DISTANCE_THRESHOLD = Double.POSITIVE_INFINITY;
    private boolean isClose = false;
    private final PostProcessPose postProcessPose;
    private Pose2D pose;

    public DualUltrasonics(PostProcessPose postProcessPose, double lateralDistance) {
        this.postProcessPose = postProcessPose;
        this.lateralDistance = lateralDistance;
    }

    public void update(double leftDistance, double rightDistance) {
        if (rightDistance > DISTANCE_THRESHOLD || leftDistance > DISTANCE_THRESHOLD) {
            this.isClose = false;
            return;
        }

        double heading = Math.atan(rightDistance-leftDistance / lateralDistance);
        double averageDistance = (rightDistance + leftDistance) / 2;
        double distance = Math.cos(heading) * averageDistance;

        pose = postProcessPose.poseProcess(distance, heading);
    }

    @Override
    public double getProbability(Pose2D pose2D, Gaussian gaussian) {
        if (!isClose)
            return 1.0;

        double x = Vector2D.dist(pose2D.asVector(), pose.asVector());
        return gaussian.value(x);
    }
}
