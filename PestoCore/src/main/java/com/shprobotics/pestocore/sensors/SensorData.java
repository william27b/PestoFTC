package com.shprobotics.pestocore.sensors;

import com.shprobotics.pestocore.geometries.Pose2D;

public interface SensorData {
    void update();
    double getProbability(Pose2D pose2D);
}
