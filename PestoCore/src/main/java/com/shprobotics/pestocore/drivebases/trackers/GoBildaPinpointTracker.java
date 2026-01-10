package com.shprobotics.pestocore.drivebases.trackers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.geometries.Pose;

public class GoBildaPinpointTracker implements DeterministicTracker {
    GoBildaPinpointDriver goBildaPinpointDriver;

    private Pose currentPosition;
    private Pose deltaPosition;
    private Pose robotVelocity;

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

    public Pose getRobotVelocity() {
        return robotVelocity;
    }

    public Pose getDeltaPosition() {
        return deltaPosition;
    }

    public Pose getCentripetalForce() {
        return Pose.ZERO;
    }

    public void reset() {
        goBildaPinpointDriver.resetPosAndIMU();
        goBildaPinpointDriver.recalibrateIMU();

        this.currentPosition = new Pose(0, 0, 0);
        this.deltaPosition = new Pose(0, 0, 0);
        this.robotVelocity = new Pose(0, 0, 0);
    }

    public void reset(double heading) {
        reset();
        goBildaPinpointDriver.setYawScalar(heading);
    }

    public void reset(Pose position) {
        reset();
        goBildaPinpointDriver.setYawScalar(position.getHeadingRadians());
        this.currentPosition = position;
    }

    public void update() {
        goBildaPinpointDriver.update();

        deltaPosition = Pose.subtract(goBildaPinpointDriver.getPosition(), currentPosition);
        deltaPosition.normalizeHeading();

        currentPosition = goBildaPinpointDriver.getPosition();

        robotVelocity = goBildaPinpointDriver.getVelocity();
    }

    public Pose getCurrentPosition() {
        return currentPosition;
    }

    public static class TrackerBuilder implements Tracker.TrackerBuilder {
        GoBildaPinpointDriver goBildaPinpointDriver;

        Pose currentPosition;
        Pose deltaPosition;
        Pose robotVelocity;

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

            this.currentPosition = new Pose(0, 0, 0);
            this.deltaPosition = new Pose(0, 0, 0);
            this.robotVelocity = new Pose(0, 0, 0);

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
