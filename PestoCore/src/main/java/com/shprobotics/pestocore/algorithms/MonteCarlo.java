package com.shprobotics.pestocore.algorithms;

import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.sensors.SensorData;

import java.util.Random;

public class MonteCarlo {
    public static class Particle {
        private Pose2D pose;
        private double weight;

        public Particle() {
            this.weight = 0.0;
        }

        protected void addPose(Pose2D dPose) {
            this.pose.add(dPose);
        }

        public Pose2D getPose() {
            return this.pose;
        }

        public double getWeight() {
            return this.weight;
        }

        public Particle copy() {
            Particle particle = new Particle();
            particle.pose = pose.copy();
            particle.weight = weight;
            return particle;
        }
    }

    @FunctionalInterface
    public interface MotionUpdate {
        void update(Pose2D deltaState, Particle particle);
    }

    @FunctionalInterface
    public interface SensorUpdate {
        void update(SensorData sensorData, Particle particle);
    }

    private final Particle[] state;
    private final int numParticles;
    private double totalWeights;
    private final Random random;

    private final MotionUpdate motionUpdate;
    private final SensorUpdate sensorUpdate;

    public MonteCarlo(MonteCarloBuilder monteCarloBuilder) {
        this.state = monteCarloBuilder.state;
        this.numParticles = monteCarloBuilder.numParticles;
        this.totalWeights = monteCarloBuilder.totalWeights;
        this.random = monteCarloBuilder.random;

        this.motionUpdate = monteCarloBuilder.motionUpdate;
        this.sensorUpdate = monteCarloBuilder.sensorUpdate;
    }

    public Particle resampleParticle() {
        double r = random.nextDouble() * totalWeights;
        double s = 0.0;

        for (Particle p: state) {
            s += p.weight;
            if (r < s)
                return p;
        }

        return null;
    }

    public void update(Pose2D deltaState, SensorData sensorData) {
        for (int i = 0; i < numParticles; i++) {
            this.totalWeights -= state[i].weight;
            motionUpdate.update(deltaState, state[i]);
            sensorUpdate.update(sensorData, state[i]);
            this.totalWeights += state[i].weight;

            if (random.nextDouble() > state[i].weight) {
                this.totalWeights -= state[i].weight;
                state[i] = resampleParticle();
                this.totalWeights += state[i].weight;
            }
        }
    }

    public Particle getMostConfident() {
        Particle mostConfidentParticle = new Particle();
        mostConfidentParticle.weight = -1;

        for (Particle particle: state)
            if (particle.weight > mostConfidentParticle.weight)
                mostConfidentParticle = particle;

        return mostConfidentParticle;
    }

    public static class MonteCarloBuilder {
        public final Particle[] state;
        public final int numParticles;
        public double totalWeights;
        public final Random random;

        public MotionUpdate motionUpdate;
        public SensorUpdate sensorUpdate;

        public MonteCarloBuilder(Pose2D startingPosition, int numParticles) {
            this.state = new Particle[numParticles];
            this.numParticles = numParticles;
            this.totalWeights = 0.0;
            this.random = new Random();

            this.motionUpdate = (deltaState, lastParticle) -> lastParticle.addPose(deltaState);
            this.sensorUpdate = (sensorData, lastParticle) -> lastParticle.weight = sensorData.getProbability(lastParticle.getPose());

            for (int i = 0; i < numParticles; i++) {
                state[i] = new Particle();
                state[i].pose = startingPosition;
                state[i].weight = 0.0;
            }
        }

        public void setMotionUpdate(MotionUpdate motionUpdate) {
            this.motionUpdate = motionUpdate;
        }

        public void setSensorUpdate(SensorUpdate sensorUpdate) {
            this.sensorUpdate = sensorUpdate;
        }

        public MonteCarlo build() {
            return new MonteCarlo(this);
        }
    }
}
