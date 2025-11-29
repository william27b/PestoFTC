package com.shprobotics.pestocore.drivebases.trackers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

public class GoBildaPinpointTracker implements DeterministicTracker {
    GoBildaPinpointDriver goBildaPinpointDriver;

    private Pose2D currentPosition;
    private Pose2D deltaPosition;
    private Pose2D robotVelocity;

    public final double FORWARD_OFFSET;
    public final double ODOMETRY_WIDTH;

    public GoBildaPinpointTracker(GoBildaPinpointTracker.TrackerBuilder trackerBuilder) {
        this.goBildaPinpointDriver = trackerBuilder.goBildaPinpointDriver;

        this.currentPosition = trackerBuilder.currentPosition;
        this.deltaPosition = trackerBuilder.deltaPosition;
        this.robotVelocity = trackerBuilder.robotVelocity;

        this.FORWARD_OFFSET = trackerBuilder.forwardOffset;
        this.ODOMETRY_WIDTH = trackerBuilder.odometryWidth;
    }

    public Pose2D getRobotVelocity() {
        return robotVelocity;
    }

    public Pose2D getDeltaPosition() {
        return deltaPosition;
    }

    public Vector2D getCentripetalForce() {
        return Vector2D.ZERO;
    }

    public void reset() {
        goBildaPinpointDriver.resetPosAndIMU();
        goBildaPinpointDriver.recalibrateIMU();

        this.currentPosition = new Pose2D(0, 0, 0);
        this.deltaPosition = new Pose2D(0, 0, 0);
        this.robotVelocity = new Pose2D(0, 0, 0);
    }

    public void reset(double heading) {
        reset();
        goBildaPinpointDriver.setYawScalar(heading);
    }

    public void reset(Pose2D position) {
        reset();
        goBildaPinpointDriver.setYawScalar(position.getHeadingRadians());
        this.currentPosition = position;
    }

    public void update() {
        goBildaPinpointDriver.update();

        deltaPosition = Pose2D.subtract(goBildaPinpointDriver.getPosition(), currentPosition, true);
        currentPosition = goBildaPinpointDriver.getPosition();

        robotVelocity = goBildaPinpointDriver.getVelocity();
    }

    public Pose2D getCurrentPosition() {
        return currentPosition;
    }

    public static class TrackerBuilder implements Tracker.TrackerBuilder {
        GoBildaPinpointDriver goBildaPinpointDriver;

        Pose2D currentPosition;
        Pose2D deltaPosition;
        Pose2D robotVelocity;

        GoBildaPinpointDriver.EncoderDirection xDirection;
        GoBildaPinpointDriver.EncoderDirection yDirection;
        double encoderResolution;

        double forwardOffset;
        double odometryWidth;

        public TrackerBuilder(
                HardwareMap hardwareMap,
                String deviceName
        ) {
            this.goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);

            this.currentPosition = new Pose2D(0, 0, 0);
            this.deltaPosition = new Pose2D(0, 0, 0);
            this.robotVelocity = new Pose2D(0, 0, 0);

            this.xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
            this.yDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
            this.encoderResolution = 1.0;

            this.forwardOffset = 0.0;
            this.odometryWidth = 0.0;
        }

        public GoBildaPinpointTracker.TrackerBuilder setXEncoderDirection(GoBildaPinpointDriver.EncoderDirection direction) {
            this.xDirection = direction;
            return this;
        }

        public GoBildaPinpointTracker.TrackerBuilder setYEncoderDirection(GoBildaPinpointDriver.EncoderDirection direction) {
            this.yDirection = direction;
            return this;
        }

        public GoBildaPinpointTracker.TrackerBuilder setForwardOffset(double forwardOffset) {
            this.forwardOffset = forwardOffset;

            this.goBildaPinpointDriver.setOffsets(this.forwardOffset, this.odometryWidth);
            return this;
        }

        public GoBildaPinpointTracker.TrackerBuilder setOdometryWidth(double odometryWidth) {
            this.odometryWidth = odometryWidth;

            this.goBildaPinpointDriver.setOffsets(this.forwardOffset, this.odometryWidth);
            return this;
        }

        public GoBildaPinpointTracker.TrackerBuilder setEncoderResolution(double encoderResolution) {
            this.encoderResolution = encoderResolution;
            return this;
        }

        public GoBildaPinpointTracker build() {
            this.goBildaPinpointDriver.setEncoderDirections(xDirection, yDirection);
            this.goBildaPinpointDriver.setEncoderResolution(encoderResolution);
            return new GoBildaPinpointTracker(this);
        }
    }
}
