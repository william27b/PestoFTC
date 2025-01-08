package com.shprobotics.pestocore.geometries;

public class ParametricHeading {
    private final double[] headings;
    private final int n;
    private double t;

    public ParametricHeading(double[] headings) {
        assert headings.length >= 2;

        this.headings = headings;
        this.n = headings.length;
        this.t = 0;
    }

    public void reset() { this.t = 0; }

    public void increment(double dt) { this.t = Math.min(1, Math.max(0, this.t + dt)); }

    public double getT() {
        return this.t;
    }

    public double getHeading(double t) {
        if (t == 0) {
            return headings[0];
        }
        if (t == 1) {
            return headings[n-1];
        }

        double point = 0.0;

        double n_factorial = 1;
        for (int i = 1; i < n; i++) {
            n_factorial *= i;
        }
        double i_factorial = 1;
        double n_minus_i_factorial = n_factorial * n;

        for (int i = 0; i < n; i++) {
            n_minus_i_factorial /= (n - i);

            double copy = headings[i];
            copy *= ((n_factorial / (i_factorial * n_minus_i_factorial)) * Math.pow(1 - t, n - i - 1) * Math.pow(t, i));

            i_factorial *= i + 1;
            point += copy;
        }

        return point;
    }

    public double getHeading() {
        return getHeading(this.t);
    }
}