package com.shprobotics.pestocore.geometries;

public class Wall {
    @FunctionalInterface
    public interface IntersectionFunction {
        Vector2D getIntersection(double m, double b);
    }

    private final IntersectionFunction intersectionFunction;

    public Wall(IntersectionFunction intersectionFunction) {
        this.intersectionFunction = intersectionFunction;
    }

    public Vector2D getIntersection(double m, double b) {
        return intersectionFunction.getIntersection(m, b);
    }
}
