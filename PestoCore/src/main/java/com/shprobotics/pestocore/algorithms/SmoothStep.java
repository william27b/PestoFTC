package com.shprobotics.pestocore.algorithms;

public class SmoothStep {
    public static double forward(double x, double leftEdge, double rightEdge) {
        x = clamp((x - leftEdge) / (rightEdge - leftEdge));
        return x * x * (3.0 - (2.0 * x));
    }

    public static double clamp(double x) {
        if (x < 0) return 0;
        if (x > 1) return 1;
        return x;
    }
}
