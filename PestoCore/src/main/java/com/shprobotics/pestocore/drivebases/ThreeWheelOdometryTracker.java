package com.shprobotics.pestocore.drivebases;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.geometries.Circle;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

public class ThreeWheelOdometryTracker implements DeterministicTracker {
    private final double FORWARD_OFFSET;
    private final double ODOMETRY_WIDTH;

    public final Odometry leftOdometry;
    public final Odometry rightOdometry;
    public final Odometry centerOdometry;

    private Pose2D robotVelocity;
    private Pose2D positionMinus2;
    private Pose2D positionMinus1;
    private Pose2D currentPosition;

    private final ElapsedTime elapsedTime;
    private double lastTime;

    public ThreeWheelOdometryTracker(TrackerBuilder trackerBuilder) {
        this.FORWARD_OFFSET = trackerBuilder.FORWARD_OFFSET;
        this.ODOMETRY_WIDTH = trackerBuilder.ODOMETRY_WIDTH;

        this.leftOdometry = trackerBuilder.leftOdometry;
        this.rightOdometry = trackerBuilder.rightOdometry;
        this.centerOdometry = trackerBuilder.centerOdometry;

        this.robotVelocity = trackerBuilder.robotVelocity;
        this.positionMinus2 = trackerBuilder.positionMinus2;
        this.positionMinus1 = trackerBuilder.positionMinus1;
        this.currentPosition = trackerBuilder.currentPosition;

        this.elapsedTime = trackerBuilder.elapsedTime;
        this.lastTime = trackerBuilder.lastTime;
    }

    public void reset() {
        this.robotVelocity = new Pose2D(0, 0, 0);
        this.positionMinus2 = new Pose2D(0, 0, 0);
        this.positionMinus1 = new Pose2D(0, 0, 0);
        this.currentPosition = new Pose2D(0, 0, 0);
    }

    public void resetTime() {
        this.lastTime = this.elapsedTime.seconds();
    }

    public void update() {
        double dL = this.leftOdometry.getInchesTravelled();
        double dC = this.centerOdometry.getInchesTravelled();
        double dR = this.rightOdometry.getInchesTravelled();

        double distanceRotated = (dL - dR) / 2;
        double x = dC - (distanceRotated * this.FORWARD_OFFSET);
        double y = (dL + dR) / 2;
        double r = - (2 * distanceRotated) / this.ODOMETRY_WIDTH;

        double deltaTime = this.elapsedTime.seconds() - this.lastTime;
        this.lastTime = this.elapsedTime.seconds();
        this.robotVelocity = Pose2D.multiply(new Pose2D(x, y, r), 1/deltaTime);

        double headingRadians = currentPosition.getHeadingRadians();

        double xOriented = (Math.cos(headingRadians) * x) - (Math.sin(headingRadians) * y);
        double yOriented = (Math.cos(headingRadians) * y) + (Math.sin(headingRadians) * x);

        this.positionMinus2 = this.positionMinus1;
        this.positionMinus1 = this.currentPosition;

        this.currentPosition.add(
                new Pose2D(
                    xOriented,
                    yOriented,
                    r
                ),
                true
        );
    }


    public Pose2D getCurrentPosition() {
        return this.currentPosition;
    }

    public Pose2D getRobotVelocity() {
        return this.robotVelocity;
    }

    public Pose2D getDeltaPosition() {
        return Pose2D.subtract(this.currentPosition, this.positionMinus1, true);
    }

    public double getCentripetalRadius() {
        return Circle.getRadius(this.positionMinus2.asVector(), this.positionMinus1.asVector(), this.currentPosition.asVector());
    }

    public Vector2D getCentripetalForce() {
        double magnitude = this.robotVelocity.getMagnitude();
        double scalar = magnitude * magnitude / getCentripetalRadius();
        return Vector2D.scale(Vector2D.perpendicular(this.robotVelocity.asVector()), scalar);
    }

    public static class TrackerBuilder implements Tracker.TrackerBuilder {
        private final double FORWARD_OFFSET;
        private final double ODOMETRY_WIDTH;

        private final Odometry leftOdometry;
        private final Odometry centerOdometry;
        private final Odometry rightOdometry;

        private final Pose2D robotVelocity;
        private final Pose2D positionMinus2;
        private final Pose2D positionMinus1;
        private final Pose2D currentPosition;
        private final double currentHeading;
        private final ElapsedTime elapsedTime;
        private final double lastTime;

        public TrackerBuilder(
                HardwareMap hardwareMap,

                double ODOMETRY_TICKS_PER_INCH,
                double FORWARD_OFFSET,
                double ODOMETRY_WIDTH,

                String leftName,
                String centerName,
                String rightName,

                DcMotorSimple.Direction leftDirection,
                DcMotorSimple.Direction centerDirection,
                DcMotorSimple.Direction rightDirection
        ) {
            this.FORWARD_OFFSET = FORWARD_OFFSET;
            this.ODOMETRY_WIDTH = ODOMETRY_WIDTH;

            this.leftOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(leftName),
                    ODOMETRY_TICKS_PER_INCH);

            this.leftOdometry.setDirection(leftDirection);

            this.rightOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(rightName),
                    ODOMETRY_TICKS_PER_INCH);

            this.rightOdometry.setDirection(rightDirection);

            this.centerOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(centerName),
                    ODOMETRY_TICKS_PER_INCH);

            this.centerOdometry.setDirection(centerDirection);

            this.leftOdometry.reset();
            this.rightOdometry.reset();
            this.centerOdometry.reset();

            this.robotVelocity = new Pose2D(0, 0, 0);
            this.positionMinus2 = new Pose2D(0, 0, 0);
            this.positionMinus1 = new Pose2D(0, 0, 0);
            this.currentPosition = new Pose2D(0, 0, 0);
            this.currentHeading = 0;

            this.elapsedTime = new ElapsedTime();
            this.lastTime = elapsedTime.seconds();
        }

        public ThreeWheelOdometryTracker build() {
            return new ThreeWheelOdometryTracker(this);
        }
    }
}