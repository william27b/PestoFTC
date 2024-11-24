package com.shprobotics.pestocore.sensors;

import com.shprobotics.pestocore.geometries.Pose2D;

import org.apache.commons.math3.analysis.function.Gaussian;

public interface SensorData {
    double getProbability(Pose2D pose2D, Gaussian gaussian);
}
