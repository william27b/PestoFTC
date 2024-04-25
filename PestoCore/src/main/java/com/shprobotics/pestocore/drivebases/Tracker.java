package com.shprobotics.pestocore.drivebases;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.geometries.Pose2D;

public class Tracker {
    private final double FORWARD_OFFSET;
    private final double ODOMETRY_WIDTH;

    private final Odometry leftOdometry;
    private final Odometry rightOdometry;
    private final Odometry centerOdometry;

    private Pose2D robotVelocity;
    private Pose2D currentPosition;

    private final ElapsedTime elapsedTime;
    private double lastTime;

    public Tracker(TrackerBuilder trackerBuilder) {
        this.FORWARD_OFFSET = trackerBuilder.FORWARD_OFFSET;
        this.ODOMETRY_WIDTH = trackerBuilder.ODOMETRY_WIDTH;

        this.leftOdometry = trackerBuilder.leftOdometry;
        this.rightOdometry = trackerBuilder.rightOdometry;
        this.centerOdometry = trackerBuilder.centerOdometry;

        this.robotVelocity = trackerBuilder.robotVelocity;
        this.currentPosition = trackerBuilder.currentPosition;

        this.elapsedTime = trackerBuilder.elapsedTime;
        this.lastTime = trackerBuilder.lastTime;
    }

    public void reset() {
        this.currentPosition = new Pose2D(0, 0, 0);
    }

    public void resetTime() {
        this.lastTime = elapsedTime.seconds();
    }

    public void updateOdometry() {
        double lT = leftOdometry.getInchesTravelled();
        double cT = centerOdometry.getInchesTravelled();
        double rT = rightOdometry.getInchesTravelled();

        double distanceRotated = (lT - rT) / 2;
        double x = cT + (distanceRotated * FORWARD_OFFSET);
        double y = (lT + rT) / 2;
        double r = - (4 * distanceRotated) / ODOMETRY_WIDTH;

        double deltaTime = elapsedTime.seconds() - lastTime;
        lastTime = elapsedTime.seconds();
        this.robotVelocity = Pose2D.multiply(new Pose2D(x, y, r), 1/deltaTime);

        double headingRadians = getCurrentPosition().getHeadingRadians();

        double xOriented = (Math.cos(headingRadians) * x) - (Math.sin(headingRadians) * y);
        double yOriented = (Math.cos(headingRadians) * y) + (Math.sin(headingRadians) * x);

        currentPosition.add(new Pose2D(
                xOriented,
                yOriented,
                r
        ), true);
    }


    public Pose2D getCurrentPosition() {
        return this.currentPosition;
    }

    public Pose2D getRobotVelocity() {
        return robotVelocity;
    }

    public static class TrackerBuilder {
        private final double FORWARD_OFFSET;
        private final double ODOMETRY_WIDTH;

        private final Odometry leftOdometry;
        private final Odometry centerOdometry;
        private final Odometry rightOdometry;

        private final Pose2D currentPosition;
        private final Pose2D robotVelocity;
        private final ElapsedTime elapsedTime;
        private final double lastTime;

        public TrackerBuilder(
                HardwareMap hardwareMap,

                double ODOMETRY_TICKS_PER_INCH,
                double FORWARD_OFFSET,
                double ODOMETRY_WIDTH,

                DcMotorSimple.Direction leftEncoderDirection,
                DcMotorSimple.Direction rightEncoderDirection,
                DcMotorSimple.Direction centerEncoderDirection,

                String leftName,
                String centerName,
                String rightName
        ) {
            this.FORWARD_OFFSET = FORWARD_OFFSET;
            this.ODOMETRY_WIDTH = ODOMETRY_WIDTH;

            this.leftOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(leftName),
                    ODOMETRY_TICKS_PER_INCH);

            this.rightOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(rightName),
                    ODOMETRY_TICKS_PER_INCH);

            this.centerOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(centerName),
                    ODOMETRY_TICKS_PER_INCH);

            this.leftOdometry.setDirection(leftEncoderDirection);
            this.rightOdometry.setDirection(rightEncoderDirection);
            this.centerOdometry.setDirection(centerEncoderDirection);

            this.leftOdometry.reset();
            this.rightOdometry.reset();
            this.centerOdometry.reset();

            this.currentPosition = new Pose2D(0, 0, 0);
            this.robotVelocity = new Pose2D(0, 0, 0);

            this.elapsedTime = new ElapsedTime();
            this.lastTime = elapsedTime.seconds();
        }

        public Tracker build() {
            return new Tracker(this);
        }
    }
}