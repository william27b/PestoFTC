package com.shprobotics.pestocore.drivebases;

import com.shprobotics.pestocore.geometries.Pose2D;

public interface Tracker {
    void reset();
    void reset(double heading);

    void resetTime();

    void update();

    Pose2D getCurrentPosition();

    interface TrackerBuilder {
        Tracker build();
    }
}