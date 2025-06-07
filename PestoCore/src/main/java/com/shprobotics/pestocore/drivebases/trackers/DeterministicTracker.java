package com.shprobotics.pestocore.drivebases.trackers;

import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

public interface DeterministicTracker extends Tracker {
    Pose2D getRobotVelocity();
    Pose2D getDeltaPosition();

    Vector2D getCentripetalForce();
}
