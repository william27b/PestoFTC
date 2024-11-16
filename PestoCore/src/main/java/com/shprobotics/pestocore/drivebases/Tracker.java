package com.shprobotics.pestocore.drivebases;

import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

public interface Tracker {
    void reset();
    void resetTime();

    void updateOdometry();

    Vector2D getCurrentPosition();
    double getCurrentHeading();
    Pose2D getRobotVelocity();

    double getCentripetalRadius();
    Vector2D getCentripetalForce();

    interface TrackerBuilder {
        Tracker build();
    }
}