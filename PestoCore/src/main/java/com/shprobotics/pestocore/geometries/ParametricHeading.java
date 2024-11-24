package com.shprobotics.pestocore.geometries;

import org.apache.commons.math3.util.MathUtils;

public class ParametricHeading {
    private final double startHeading;
    private final double endHeading;

    public ParametricHeading(double startHeading, double endHeading) {
        this.startHeading = startHeading;
        this.endHeading = MathUtils.normalizeAngle(endHeading, startHeading);
    }

    public ParametricHeading(double startHeading, double endHeading, boolean CW) {
        this.startHeading = startHeading;
        endHeading = MathUtils.normalizeAngle(endHeading, startHeading);

        if (CW)
            if (endHeading > this.startHeading) this.endHeading = endHeading - (2*Math.PI);
            else this.endHeading = endHeading;
        else
            if (endHeading < this.startHeading) this.endHeading = endHeading + (2*Math.PI);
            else this.endHeading = endHeading;
    }

    public double getHeading(double t) {
        return startHeading + (endHeading - startHeading) * t;
    }

    public double getStartHeading() {
        return startHeading;
    }

    public double getEndHeading() {
        return endHeading;
    }
}