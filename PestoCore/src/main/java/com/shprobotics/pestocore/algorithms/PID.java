package com.shprobotics.pestocore.algorithms;

import java.util.function.DoubleSupplier;

public class PID {
    private final double kP;
    private final double kI;
    private final double kD;

    private double a;
    private double previous;

    private double integralSum;
    private double reference;

    private double lastError;

    private double deltaTime;
    private double lastTime;
    private final DoubleSupplier timer;

    public PID(double kP, double kI, double kD, DoubleSupplier timer) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.a = 0;

        this.integralSum = 0;
        this.reference = 0;

        this.lastError = 0;

        this.deltaTime = 0;
        this.lastTime = timer.getAsDouble();
        this.timer = timer;
    }

    public void setLowPassFilter(double a) {
        this.a = a;
    }

    public double getOutput(double current, double target) {
        double tmp = a * current + (1 - a) * previous;
        previous = current;
        current = tmp;

        if (target != reference) {
            this.integralSum = 0;
        }
        this.reference = target;

        double currentTime = timer.getAsDouble();
        this.deltaTime = currentTime - lastTime;

        double error = target - current;
        integralSum += error * deltaTime;

        double derivative = (error - lastError) / deltaTime;
        this.lastError = error;
        this.lastTime = currentTime;

        return kP * error + kI * integralSum + kD * derivative;
    }
}
