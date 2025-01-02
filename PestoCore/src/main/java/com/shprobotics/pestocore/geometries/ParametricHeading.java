package com.shprobotics.pestocore.geometries;

import org.apache.commons.math3.util.MathUtils;

public class ParametricHeading {
    private final double[] headings;
    private final int n;

    public ParametricHeading(double[] headings) {
        assert headings.length >= 2;

        this.headings = headings;
        this.n = headings.length;
    }

    public double getHeading(double t) {
        assert t >= 0 && t <= 1;

        if (t == 1)
            return headings[n-1];

        int startIndex = (int) ((headings.length - 1) * t);
        double startHeading = headings[startIndex];
        double endHeading = headings[startIndex+1];

        t *= (startIndex + 1.0) / n;

        return MathUtils.normalizeAngle(startHeading + (endHeading - startHeading) * t, 0.0);
    }
}