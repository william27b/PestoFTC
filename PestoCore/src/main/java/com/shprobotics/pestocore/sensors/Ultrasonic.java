package com.shprobotics.pestocore.sensors;

import static org.apache.commons.math3.stat.StatUtils.min;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Ray;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.geometries.Wall;

import org.apache.commons.math3.analysis.function.Gaussian;

public class Ultrasonic implements SensorData {
    @FunctionalInterface
    public interface ParticleFilter {
        double getProbability(Pose2D pose2D, double distance);
    }

    private final ParticleFilter particleFilter;
    private final AnalogInput ultrasonic;
    private double distance;

    public Ultrasonic(AnalogInput ultrasonic) {
        this.ultrasonic = ultrasonic;
        this.particleFilter = (Pose2D pose2D, double distance) -> {
                    Wall top = new Wall((double m, double b) -> new Vector2D((144 - b) / m, 144));
                    Wall bottom = new Wall((double m, double b) -> new Vector2D(-b / m, 0));
                    Wall left = new Wall((double m, double b) -> new Vector2D(0, b));
                    Wall right = new Wall((double m, double b) -> new Vector2D(144, (m*144) + b));

                    double rayDist = min(new double[]{
                            Ray.getDistance(top, pose2D),
                            Ray.getDistance(bottom, pose2D),
                            Ray.getDistance(left, pose2D),
                            Ray.getDistance(right, pose2D)
                    });
                    Gaussian gaussian = new Gaussian(0, 0.3);
                    return gaussian.value(distance - rayDist);
        };
    }

    public Ultrasonic(AnalogInput ultrasonic, ParticleFilter particleFilter) {
        this.ultrasonic = ultrasonic;
        this.particleFilter = particleFilter;
    }

    public void update() {
        this.distance = ultrasonic.getVoltage() * 62.039;
    }

    public double getDistance() {
        return distance;
    }

    @Override
    public double getProbability(Pose2D pose2D) {
        return particleFilter.getProbability(pose2D, distance);
    }
}
