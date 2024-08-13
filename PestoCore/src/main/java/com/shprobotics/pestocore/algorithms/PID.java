package com.shprobotics.pestocore.algorithms;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    private final double kP;
    private final double kI;
    private final double kD;

    private double lowPassFilter;
    private double previous;

    private double integralSum;
    private double maxIntegralProportionRatio;

    private double lastError;

    private double deltaTime;
    private double lastTime;
    private final ElapsedTime elapsedTime;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.lowPassFilter = 0;
        this.previous = 0;

        this.integralSum = 0;
        this.maxIntegralProportionRatio = Double.POSITIVE_INFINITY;

        this.lastError = 0;

        this.deltaTime = 0;
        this.lastTime = 0;
        this.elapsedTime = new ElapsedTime();
        this.elapsedTime.reset();
    }

    public void reset() {
        this.previous = 0;

        this.integralSum = 0;

        this.lastError = 0;
        this.lastTime = elapsedTime.seconds();
    }

    public void setLowPassFilter(double lowPassFilter) {
        this.lowPassFilter = lowPassFilter;
    }

    public void setMaxIntegralProportionRatio(double maxIntegralProportionRatio) {
        this.maxIntegralProportionRatio = maxIntegralProportionRatio;
    }

    public double getOutput(double current, double target) {
        // Apply low pass filter, save previous measured value
        double tmp = lowPassFilter * current + (1 - lowPassFilter) * previous;
        previous = current;
        current = tmp;

        double currentTime = elapsedTime.seconds();
        this.deltaTime = currentTime - lastTime;
        double error = target - current;

        // proportion
        double proportion = kP * error;

        integralSum += error * deltaTime;

        // integral clamping
        if (Math.abs(proportion) > maxIntegralProportionRatio) {
            if (proportion > 0) {
                integralSum = Math.min(integralSum, maxIntegralProportionRatio - proportion);
            } else {
                integralSum = Math.max(integralSum, -maxIntegralProportionRatio - proportion);
            }
        }

        // integral
        double integral = kI * integralSum;

        // derivative
        double derivative = kD * (error - lastError) / deltaTime;

        this.lastError = error;
        this.lastTime = currentTime;

        return proportion + integral + derivative;
    }
}
