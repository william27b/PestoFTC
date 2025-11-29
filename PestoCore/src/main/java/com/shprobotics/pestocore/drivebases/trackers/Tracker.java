package com.shprobotics.pestocore.drivebases.trackers;

import com.shprobotics.pestocore.geometries.Pose2D;

public interface Tracker {
    void reset();
    void reset(double heading);
    void reset(Pose2D position);

    void update();

    Pose2D getCurrentPosition();

    interface TrackerBuilder {
        Tracker build();
    }
}