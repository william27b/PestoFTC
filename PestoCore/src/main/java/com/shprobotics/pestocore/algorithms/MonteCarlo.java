package com.shprobotics.pestocore.algorithms;

import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.sensors.SensorData;

import org.apache.commons.math3.analysis.function.Gaussian;

import java.util.Random;

public class MonteCarlo {
    public class Particle {
        private Pose2D pose;
        private double weight;

        protected void addPose(Pose2D dPose) {
            this.pose.add(dPose, true);
        }

        public Pose2D getPose() {
            return this.pose;
        }
    }

    @FunctionalInterface
    public interface ParticleResampler {
        Particle resampleParticle();
    }

    private final Particle[] state;
    private final int numParticles;
    private final Random random;
    private final ParticleResampler particleResampler;
    public static final Gaussian gaussian = new Gaussian(0, 1/Math.sqrt(2 * Math.PI));

    public MonteCarlo(int numParticles) {
        this.state = new Particle[numParticles];
        this.numParticles = numParticles;
        this.random = new Random();

        this.particleResampler = () -> {
            Particle particle = new Particle();

            particle.pose = new Pose2D(
                    random.nextDouble() * 144,
                    random.nextDouble() * 144,
                    ((random.nextDouble() * 2) - 1) * Math.PI
            );

            return particle;
        };

        for (int i = 0; i < numParticles; i++) {
            state[i] = particleResampler.resampleParticle();
        }
    }

    public MonteCarlo(int numParticles, ParticleResampler particleResampler) {
        this.state = new Particle[numParticles];
        this.numParticles = numParticles;
        this.random = new Random();
        this.particleResampler = particleResampler;

        for (int i = 0; i < numParticles; i++) {
            state[i] = particleResampler.resampleParticle();
        }
    }

    public void motionUpdate(Pose2D deltaState, Particle lastParticle) {
        lastParticle.addPose(deltaState);
    }

    public void sensorUpdate(SensorData sensorData, Particle particle) {


        particle.weight = sensorData.getProbability(particle.pose, gaussian);
    }

    public void update(Pose2D deltaState, SensorData sensorData) {
        Particle[] state = new Particle[numParticles];
        for (int i = 0; i < numParticles; i++) {
            motionUpdate(deltaState, state[i]);
            sensorUpdate(sensorData, state[i]);

            if (random.nextDouble() > state[i].weight)
                state[i] = particleResampler.resampleParticle();
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
}
