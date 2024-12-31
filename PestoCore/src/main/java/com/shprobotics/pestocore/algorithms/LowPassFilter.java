package com.shprobotics.pestocore.algorithms;

public class LowPassFilter {
    private final double alpha;
    private double previous;

    public LowPassFilter (double alpha) {
        assert alpha >= 0.0 && alpha <= 1.0;

        this.alpha = alpha;
        this.previous = 0;
    }

    public LowPassFilter () {
        this.alpha = 1.0;
        this.previous = 0;
    }

    public void reset() {
        this.previous = 0;
    }

    public double forward(double x) {
        double output = (alpha * x) + ((1 - alpha) * previous);
        previous = output;
        return output;
    }

    public double getPrevious() {
        return this.previous;
    }
}
