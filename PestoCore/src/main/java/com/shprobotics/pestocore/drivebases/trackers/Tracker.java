package com.shprobotics.pestocore.drivebases.trackers;

import com.shprobotics.pestocore.geometries.Pose;

public interface Tracker {
    void reset();
    void reset(double heading);
    void reset(Pose position);

    void update();

    Pose getCurrentPosition();

    interface TrackerBuilder {
        Tracker build();
    }
}