package com.shprobotics.pestocore.drivebases.trackers;

import com.shprobotics.pestocore.geometries.Pose;

public interface DeterministicTracker extends Tracker {
    Pose getRobotVelocity();
    Pose getDeltaPosition();

    Pose getCentripetalForce();
}
