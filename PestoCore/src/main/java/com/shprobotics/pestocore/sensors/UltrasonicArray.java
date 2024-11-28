package com.shprobotics.pestocore.sensors;

import com.shprobotics.pestocore.geometries.Pose2D;

public class UltrasonicArray implements SensorData {
    private final Ultrasonic[] ultrasonics;
    private final Pose2D[] relativePositions;

    public UltrasonicArray(Ultrasonic[] ultrasonics, Pose2D[] relativePositions) {
        this.ultrasonics = ultrasonics;
        this.relativePositions = relativePositions;

        if (ultrasonics.length != relativePositions.length)
            throw new IllegalArgumentException("# of ultrasonics must == # of relativePositions");
    }

    public void update() {
        for (int i = 0; i < ultrasonics.length; i++)
            ultrasonics[i].update();
    }

    @Override
    public double getProbability(Pose2D pose2D) {
        double probability = 1.0;

        for (int i = 0; i < ultrasonics.length; i++)
            probability *= ultrasonics[i].getProbability(Pose2D.add(pose2D, Pose2D.rotate(relativePositions[i], pose2D.getHeadingRadians()).asVector()));

        return probability;
    }
}
