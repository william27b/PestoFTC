package com.shprobotics.pestocore.drivebases.trackers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.geometries.Circle;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

public class MecanumTracker implements DeterministicTracker {
    private final double CIRCUMFERENCE;

    public final Odometry frontLeft;
    public final Odometry frontRight;
    public final Odometry backLeft;
    public final Odometry backRight;

    private Pose2D robotVelocity;
    private Pose2D positionMinus2;
    private Pose2D positionMinus1;
    private Pose2D currentPosition;

    private final ElapsedTime elapsedTime;
    private double lastTime;

    public MecanumTracker(TrackerBuilder trackerBuilder) {
        Vector2D SHAPE_VECTOR = trackerBuilder.SHAPE_VECTOR;
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
        this.robotVelocity = new Pose2D(0, 0, 0);
        this.positionMinus2 = new Pose2D(0, 0, 0);
        this.positionMinus1 = new Pose2D(0, 0, 0);
        this.currentPosition = new Pose2D(0, 0, heading);
    }

    public void reset(Pose2D position) {
        this.robotVelocity = new Pose2D(0, 0, 0);
        this.positionMinus2 = new Pose2D(0, 0, 0);
        this.positionMinus1 = new Pose2D(0, 0, 0);
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
        this.robotVelocity = Pose2D.multiply(new Pose2D(x, y, r), 1/deltaTime);

        double headingRadians = this.currentPosition.getHeadingRadians();

        double xOriented = (Math.cos(headingRadians) * x) - (Math.sin(headingRadians) * y);
        double yOriented = (Math.cos(headingRadians) * y) + (Math.sin(headingRadians) * x);

        this.positionMinus2 = this.positionMinus1;
        this.positionMinus1 = this.currentPosition;
        this.currentPosition.add(new Pose2D(
                xOriented,
                yOriented,
                r
        ));
    }


    public Pose2D getCurrentPosition() {
        return this.currentPosition;
    }

    public Pose2D getDeltaPosition() {
        return Pose2D.subtract(currentPosition, positionMinus1, true);
    }

    public Pose2D getRobotVelocity() {
        return this.robotVelocity;
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
        private final Vector2D SHAPE_VECTOR;

        private final Odometry frontLeft;
        private final Odometry frontRight;
        private final Odometry backLeft;
        private final Odometry backRight;

        private final Pose2D robotVelocity;
        private final Pose2D positionMinus2;
        private final Pose2D positionMinus1;
        private final Pose2D currentPosition;
        private final double currentHeading;
        private final ElapsedTime elapsedTime;
        private final double lastTime;

        public TrackerBuilder (
                HardwareMap hardwareMap,

                double MOTOR_TICKS_PER_INCH,
                Vector2D SHAPE_VECTOR,

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

            this.robotVelocity = new Pose2D(0, 0, 0);
            this.positionMinus2 = new Pose2D(0, 0, 0);
            this.positionMinus1 = new Pose2D(0, 0, 0);
            this.currentPosition = new Pose2D(0, 0, 0);
            this.currentHeading = 0;

            this.elapsedTime = new ElapsedTime();
            this.lastTime = elapsedTime.seconds();
        }

        public MecanumTracker build() {
            return new MecanumTracker(this);
        }
    }
}