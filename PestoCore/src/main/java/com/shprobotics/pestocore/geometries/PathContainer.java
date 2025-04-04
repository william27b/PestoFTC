package com.shprobotics.pestocore.geometries;

import java.util.ArrayList;

public class PathContainer {
    public final ArrayList<BezierCurve> curves;
    public final ArrayList<ParametricHeading> headings;
    private final Vector2D startPosition;
    private Vector2D currentPosition;
    private double heading;
    private final ArrayList<Runnable> actions;
    private boolean executed;
    private final int n;
    public int i;

    private final double increment;

    public PathContainer(PathContainerBuilder pathContainerBuilder) {
        this.curves = pathContainerBuilder.curves;
        this.headings = pathContainerBuilder.headings;
        this.startPosition = pathContainerBuilder.startPosition;
        this.currentPosition = this.startPosition.copy();
        this.heading = 0;
        this.actions = pathContainerBuilder.actions;
        this.executed = false;
        this.n = curves.size();
        this.i = 0;

        this.increment = pathContainerBuilder.increment;
    }

    public void reset() {
        for (BezierCurve curve: curves) {
            curve.reset();
        }
        this.currentPosition = this.startPosition.copy();
        executed = false;
        this.i = 0;
    }

    public Vector2D getEndpoint() {
        for (int i = n-1; i >= 0; i--) {
            if (curves.get(i) != null)
                return curves.get(i).getPoint(1.0);
        }
        return startPosition;
    }

    public boolean isFinished() {
        return curves.get(i).getT() == 1;
    }

    public boolean isExecuted() {
        return executed;
    }

    private static double distance(double alpha, double beta) {
        double phi = Math.abs(beta - alpha) % 360; // This is either the distance or 360 - distance
        return phi > 180 ? 360 - phi : phi;
    }

    public void updateHeading(double robotHeading) {
        // currentHeading
        ParametricHeading currentHeading = headings.get(i);

        if (currentHeading != null) {
            currentHeading.increment(increment);

            if (heading == currentHeading.getHeading()) {
                currentHeading.increment(increment);
            }

            double nextHeading = currentHeading.getHeading();

            while (distance(nextHeading, robotHeading) < distance(heading, robotHeading)) {
                heading = nextHeading;
                currentHeading.increment(increment);
                nextHeading = currentHeading.getHeading();
            }

            this.heading = nextHeading;
        }
    }

    public Vector2D getNextPosition(Vector2D robotPosition, double robotHeading) {
        // currentCurve
        BezierCurve currentCurve = curves.get(i);

        // currentHeading
        ParametricHeading currentHeading = headings.get(i);

        if ((currentCurve == null || currentCurve.getT() == 1) && (currentHeading == null || currentHeading.getT() == 1)) {
            if (this.actions.get(i) != null) {
                this.actions.get(i).run();
                if (i == n-1)
                    executed = true;
            }
            this.i = Math.min(n-1, i+1);
            currentCurve = curves.get(i);
        }

        if (currentCurve != null) {
            Vector2D currentPosition = currentCurve.getPoint();
            currentCurve.increment(increment);

            if (Vector2D.equals(currentPosition, robotPosition)) {
                currentCurve.increment(increment);
            }

            Vector2D nextPosition = currentCurve.getPoint();

            while (Vector2D.fastdist(nextPosition, robotPosition) < Vector2D.fastdist(currentPosition, robotPosition)) {
                currentPosition = nextPosition;
                currentCurve.increment(increment);
                nextPosition = currentCurve.getPoint();
            }

            this.currentPosition = currentPosition;
        }


        return currentPosition;
    }

    public double getHeading() {
        return heading;
    }

    public int getI() {
        return i;
    }

    public int getN() {
        return n;
    }

    public static class PathContainerBuilder {
        private final ArrayList<BezierCurve> curves;
        private final ArrayList<ParametricHeading> headings;
        private final ArrayList<Runnable> actions;
        private Vector2D startPosition;

        private double increment;

        public PathContainerBuilder() {
            this.curves = new ArrayList<>();
            this.headings = new ArrayList<>();
            this.actions = new ArrayList<>();
            this.startPosition = new Vector2D(0, 0);

            this.increment = 0.0005;
        }

        public PathContainerBuilder setStartPosition(Vector2D startPosition) {
            this.startPosition = startPosition;
            return this;
        }

        public PathContainerBuilder setIncrement(double increment) {
            this.increment = increment;
            return this;
        }

        public PathContainerBuilder addCurve(BezierCurve curve) {
            this.curves.add(curve);
            this.headings.add(null);
            this.actions.add(null);
            return this;
        }

        public PathContainerBuilder addCurve(ParametricHeading heading) {
            this.curves.add(null);
            this.headings.add(heading);
            this.actions.add(null);
            return this;
        }

        public PathContainerBuilder addCurve(BezierCurve curve, ParametricHeading heading) {
            this.curves.add(curve);
            this.headings.add(heading);
            this.actions.add(null);
            return this;
        }

        public PathContainerBuilder addCurve(BezierCurve curve, Runnable action) {
            this.curves.add(curve);
            this.headings.add(null);
            this.actions.add(action);
            return this;
        }

        public PathContainerBuilder addCurve(BezierCurve curve, ParametricHeading heading, Runnable action) {
            this.curves.add(curve);
            this.headings.add(heading);
            this.actions.add(action);
            return this;
        }

        public PathContainer build() {
            return new PathContainer(this);
        }
    }
}
