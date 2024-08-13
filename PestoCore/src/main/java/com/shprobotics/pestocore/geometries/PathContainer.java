package com.shprobotics.pestocore.geometries;

import java.util.ArrayList;

public class PathContainer {
    private ArrayList<BezierCurve> curves;
    private ArrayList<ParametricHeading> headings;
    private double heading;
    private ArrayList<Runnable> actions;
    private final int n;
    private int i;

    private final double increment;

    public PathContainer(PathContainerBuilder pathContainerBuilder) {
        this.curves = pathContainerBuilder.curves;
        this.headings = pathContainerBuilder.headings;
        this.heading = 0;
        this.actions = pathContainerBuilder.actions;
        this.n = curves.size();
        this.i = 0;

        this.increment = pathContainerBuilder.increment;
    }

    public void reset() {
        this.i = 0;
        for (BezierCurve curve: curves) {
            curve.reset();
        }
    }

    public Vector2D getEndpoint() {
        return curves.get(n-1).getPoint(1.0);
    }

    public Vector2D getNextPosition(Vector2D robotPosition) {
        BezierCurve current = curves.get(i);
        if (current.getT() == 1) {
            if (this.actions.get(i) != null) {
                this.actions.get(i).run();
            }
            this.i = Math.min(n-1, i+1);
            current = curves.get(i);
        }

        Vector2D currentPosition = current.getPoint();
        current.increment(increment);

        if (Vector2D.equals(currentPosition, robotPosition)) {
            current.increment(increment);
        }

        Vector2D nextPosition = current.getPoint();

        while (Vector2D.fastdist(nextPosition, robotPosition) < Vector2D.fastdist(currentPosition, robotPosition)) {
            currentPosition = nextPosition;
            current.increment(increment);
            nextPosition = current.getPoint();
        }

        this.heading = headings.get(i) != null ? headings.get(i).getHeading(current.getT()) : this.heading;

        return currentPosition;
    }

    public double getHeading() {
        return heading;
    }

    public static class PathContainerBuilder {
        private ArrayList<BezierCurve> curves;
        private ArrayList<ParametricHeading> headings;
        private ArrayList<Runnable> actions;
        private int i;

        private double increment;

        private Pose2D startPosition;

        public PathContainerBuilder() {
            this.curves = new ArrayList<>();
            this.headings = new ArrayList<>();
            this.actions = new ArrayList<>();

            this.startPosition = new Pose2D(0, 0, 0);
            this.increment = 0.0005;
        }

        public PathContainerBuilder(Pose2D startPosition) {
            this.curves = new ArrayList<>();
            this.headings = new ArrayList<>();
            this.actions = new ArrayList<>();

            this.startPosition = startPosition;
            this.increment = 0.0005;
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
