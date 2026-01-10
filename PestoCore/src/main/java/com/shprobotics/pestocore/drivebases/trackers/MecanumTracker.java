package com.shprobotics.pestocore.drivebases.trackers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.geometries.Circle;
import com.shprobotics.pestocore.geometries.Pose;

public class MecanumTracker implements DeterministicTracker {
    private final double CIRCUMFERENCE;

    public final Odometry frontLeft;
    public final Odometry frontRight;
    public final Odometry backLeft;
    public final Odometry backRight;

    private Pose robotVelocity;
    private Pose positionMinus2;
    private Pose positionMinus1;
    private Pose currentPosition;

    private final ElapsedTime elapsedTime;
    private double lastTime;

    public MecanumTracker(TrackerBuilder trackerBuilder) {
        Pose SHAPE_VECTOR = trackerBuilder.SHAPE_VECTOR;
        this.CIRCUMFERENCE = SHAPE_VECTOR.getMagnitude() * Math.PI * 2;

        this.frontLeft = trackerBuilder.frontLeft;
        this.frontRight = trackerBuilder.frontRight;
        this.backLeft = trackerBuilder.backLeft;
        this.backRight = trackerBuilder.backRight;

        this.robotVelocity = trackerBuilder.robotVelocity;
        this.positionMinus2 = trackerBuilder.positionMinus2;
        this.positionMinus1 = trackerBuilder.positionMinus1;
        this.currentPosition = trackerBuilder.currentPosition;

        this.elapsedTime = trackerBuilder.elapsedTime;
        this.lastTime = trackerBuilder.lastTime;
    }

    public void reset() {
        reset(0.0);
    }

    public void reset(double heading) {
        this.robotVelocity = new Pose(0, 0, 0);
        this.positionMinus2 = new Pose(0, 0, 0);
        this.positionMinus1 = new Pose(0, 0, 0);
        this.currentPosition = new Pose(0, 0, heading);
    }

    public void reset(Pose position) {
        this.robotVelocity = new Pose(0, 0, 0);
        this.positionMinus2 = new Pose(0, 0, 0);
        this.positionMinus1 = new Pose(0, 0, 0);
        this.currentPosition = position;
    }

    public void resetTime() {
        this.lastTime = this.elapsedTime.seconds();
    }

    public void update() {
        double frontLeftDelta = this.frontLeft.getInchesTravelled();
        double frontRightDelta = this.frontRight.getInchesTravelled();
        double backLeftDelta = this.backLeft.getInchesTravelled();
        double backRightDelta = this.backRight.getInchesTravelled();

        double distanceRotated = (frontLeftDelta - backLeftDelta + frontRightDelta - backRightDelta) / 4;
        double x = (frontLeftDelta - backLeftDelta - frontRightDelta + backRightDelta) / 4 - distanceRotated;
        double y = (frontLeftDelta + frontRightDelta + backLeftDelta + backRightDelta) / 4;
        double r = distanceRotated / this.CIRCUMFERENCE;

        double deltaTime = this.elapsedTime.seconds() - this.lastTime;
        this.lastTime = this.elapsedTime.seconds();
        this.robotVelocity = Pose.multiply(new Pose(x, y, r), 1/deltaTime);

        double headingRadians = this.currentPosition.getHeadingRadians();

        double xOriented = (Math.cos(headingRadians) * x) - (Math.sin(headingRadians) * y);
        double yOriented = (Math.cos(headingRadians) * y) + (Math.sin(headingRadians) * x);

        this.positionMinus2 = this.positionMinus1;
        this.positionMinus1 = this.currentPosition;
        this.currentPosition.add(new Pose(
                xOriented,
                yOriented,
                r
        ));
    }


    public Pose getCurrentPosition() {
        return this.currentPosition;
    }

    public Pose getDeltaPosition() {
        Pose deltaPosition = Pose.subtract(currentPosition, positionMinus1);
        deltaPosition.normalizeHeading();

        return deltaPosition;
    }

    public Pose getRobotVelocity() {
        return this.robotVelocity;
    }

    public double getCentripetalRadius() {
        return Circle.getRadius(this.positionMinus2.asVector(), this.positionMinus1.asVector(), this.currentPosition.asVector());
    }

    public Pose getCentripetalForce() {
        double magnitude = this.robotVelocity.getMagnitude();
        double scalar = magnitude * magnitude / getCentripetalRadius();
        return Pose.scale(Pose.perpendicular(this.robotVelocity.asVector()), scalar);
    }

    public static class TrackerBuilder implements Tracker.TrackerBuilder {
        private final Pose SHAPE_VECTOR;

        private final Odometry frontLeft;
        private final Odometry frontRight;
        private final Odometry backLeft;
        private final Odometry backRight;

        private final Pose robotVelocity;
        private final Pose positionMinus2;
        private final Pose positionMinus1;
        private final Pose currentPosition;
        private final double currentHeading;
        private final ElapsedTime elapsedTime;
        private final double lastTime;

        public TrackerBuilder (
                HardwareMap hardwareMap,

                double MOTOR_TICKS_PER_INCH,
                Pose SHAPE_VECTOR,

                String frontLeftName,
                String frontRightName,
                String backLeftName,
                String backRightName,

                DcMotorSimple.Direction frontLeftDirection,
                DcMotorSimple.Direction frontRightDirection,
                DcMotorSimple.Direction backLeftDirection,
                DcMotorSimple.Direction backRightDirection
        ) {
            this.SHAPE_VECTOR = SHAPE_VECTOR;

            this.frontLeft = new Odometry(
                    (DcMotor)hardwareMap.get(frontLeftName),
                    MOTOR_TICKS_PER_INCH);
            this.frontLeft.setDirection(frontLeftDirection);

            this.frontRight = new Odometry(
                    (DcMotor)hardwareMap.get(frontRightName),
                    MOTOR_TICKS_PER_INCH);
            this.frontRight.setDirection(frontRightDirection);

            this.backLeft = new Odometry(
                    (DcMotor)hardwareMap.get(backLeftName),
                    MOTOR_TICKS_PER_INCH);
            this.backLeft.setDirection(backLeftDirection);

            this.backRight = new Odometry(
                    (DcMotor)hardwareMap.get(backRightName),
                    MOTOR_TICKS_PER_INCH);
            this.backRight.setDirection(backRightDirection);

            this.robotVelocity = new Pose(0, 0, 0);
            this.positionMinus2 = new Pose(0, 0, 0);
            this.positionMinus1 = new Pose(0, 0, 0);
            this.currentPosition = new Pose(0, 0, 0);
            this.currentHeading = 0;

            this.elapsedTime = new ElapsedTime();
            this.lastTime = elapsedTime.seconds();
        }

        public MecanumTracker build() {
            return new MecanumTracker(this);
        }
    }
}