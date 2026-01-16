package com.shprobotics.pestocore.geometries;

import java.util.ArrayList;

public class PathContainer {
    public final ArrayList<BezierCurve> curves;
    private Pose currentPosition;
    private final int n;
    public int i;

    private final double increment;

    public PathContainer(PathContainerBuilder pathContainerBuilder) {
        this.curves = pathContainerBuilder.curves;
        this.currentPosition = this.curves.get(0).getPose(0.0);
        this.n = curves.size();
        this.i = 0;

        this.increment = pathContainerBuilder.increment;
    }

    public void reset() {
        for (BezierCurve curve : curves)
            curve.reset();

        this.currentPosition = this.curves.get(0).getPose(0.0);
        this.i = 0;
    }

    public Pose getEndpoint() {
        return curves.get(n - 1).getPose(1.0);
    }

    public Pose getCurrentPosition() {
        return this.currentPosition;
    }

    public Pose getNextPosition(Pose robotPosition, double lookAhead) {
        assert lookAhead > 0;

        // currentCurve
        BezierCurve currentCurve = curves.get(i);

        if (currentCurve.getT() == 1) {
            this.i = Math.min(n - 1, i + 1);
            currentCurve = curves.get(i);
        }

        Pose currentPosition = currentCurve.getPose();
        currentCurve.increment(increment);

        if (Pose.equals(currentPosition, robotPosition)) {
            currentCurve.increment(increment);
        }

        Pose nextPosition = currentCurve.getPose();

        while (Pose.dist(nextPosition, robotPosition) <= Pose.dist(currentPosition, robotPosition) || Pose.dist(nextPosition, robotPosition) < lookAhead) {
            currentPosition = nextPosition;
            currentCurve.increment(increment);
            nextPosition = currentCurve.getPose();

            if (currentCurve.getT() == 1) break;
        }

        this.currentPosition = currentPosition;
        return currentPosition;
    }

    public int getI() {
        return i;
    }

    public int getN() {
        return n;
    }

    public static class PathContainerBuilder {
        private final ArrayList<BezierCurve> curves;
        private double increment;

        public PathContainerBuilder() {
            this.curves = new ArrayList<>();
            this.increment = 0.01;
        }

        public PathContainerBuilder setIncrement(double increment) {
            assert increment > 0;

            this.increment = increment;
            return this;
        }

        public PathContainerBuilder addCurve(BezierCurve curve) {
            assert curve != null;
            this.curves.add(curve);
            return this;
        }

        public PathContainer build() {
            assert curves.size() > 0;
            return new PathContainer(this);
        }
    }
}
