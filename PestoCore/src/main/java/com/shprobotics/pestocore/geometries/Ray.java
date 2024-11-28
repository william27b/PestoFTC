package com.shprobotics.pestocore.geometries;

import static java.lang.Math.abs;

import org.apache.commons.math3.util.MathUtils;

import java.util.function.Function;

public class Ray {
    public static double getDistance(Wall wall, Pose2D origin) {
        final double heading = MathUtils.normalizeAngle(origin.getHeadingRadians() + (Math.PI / 2), 0);

        double m = Math.tan(heading);
        double b = origin.getY() - (m * origin.getX());
        // y = mx + b

        Function<Vector2D, Boolean> reasonableIntersection = (Vector2D vector) -> {
            if (abs(heading) > Math.PI / 4 && abs(heading) < 3 * Math.PI / 4)
                return vector.getY() > 0 == heading > 0;
            return vector.getX() > 0 == abs(heading) < Math.PI/2;
        };

        Vector2D intersection = wall.getIntersection(m, b);

        if (intersection == Vector2D.NONE || !reasonableIntersection.apply(Vector2D.subtract(intersection, origin.asVector())))
            return Double.POSITIVE_INFINITY;

        return Vector2D.dist(intersection, origin.asVector());
    }
}
