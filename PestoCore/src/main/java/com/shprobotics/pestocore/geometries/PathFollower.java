package com.shprobotics.pestocore.geometries;

import static org.apache.commons.math3.util.MathUtils.normalizeAngle;

import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;

public class PathFollower {
    @FunctionalInterface
    public interface DecelerationFunction {
        Pose decelerate(PathFollower pathFollower, double rotate);
    }

    public static final DecelerationFunction DEFAULT_DECELERATION = (PathFollower pathFollower, double heading) -> {
        Pose vectorToEndpoint = Pose.subtract(pathFollower.endpoint, pathFollower.predictBrakeStop(pathFollower.tracker.getRobotVelocity().asVector()));
        double forward = vectorToEndpoint.getY();
        double strafe = vectorToEndpoint.getX();

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

        // reconsider
        double drivePower = pathFollower.endpointPID.getOutput(pathFollower.tracker.getCurrentPosition().getMagnitude(), pathFollower.endpoint.getMagnitude());

        Pose drive = new Pose(
                forward,
                strafe
        );

        drive.scale(drivePower);

        return drive;
    };

    @FunctionalInterface
    public interface CheckFinishedFunction {
        boolean isFinished(PathFollower pathFollower, double toleranceXY, double toleranceR, double toleranceV);
    }

    public static final CheckFinishedFunction DEFAULT_CHECK_FINISHED = (PathFollower pathFollower, double toleranceXY, double toleranceR, double toleranceV) -> (
            Pose.dist(pathFollower.tracker.getCurrentPosition().asVector(), pathFollower.pathContainer.getEndpoint()) < toleranceXY
                    && normalizeAngle(pathFollower.tracker.getCurrentPosition().getHeadingRadians() - pathFollower.pathContainer.getEndpoint().getHeadingRadians(), Math.PI) < toleranceR
                    && pathFollower.tracker.getRobotVelocity().getMagnitude() < toleranceV
    );

    private final DriveController driveController;
    private final DeterministicTracker tracker;
    private final PathContainer pathContainer;
    private final Pose endpoint;
    private final CheckFinishedFunction checkFinishedFunction;
    private final DecelerationFunction decelerationFunction;

    private final double deceleration;
    private boolean decelerating;
    private final PID headingPID;
    private final PID endpointPID;

    private final double lookAhead;
    private final double endToleranceXY;
    private final double endToleranceR;
    private final double endVelocityTolerance;

    public PathFollower(PathFollowerBuilder pathFollowerBuilder) {
        this.driveController = pathFollowerBuilder.driveController;
        this.tracker = pathFollowerBuilder.tracker;
        this.pathContainer = pathFollowerBuilder.pathContainer;
        this.endpoint = pathFollowerBuilder.pathContainer.getEndpoint();
        this.checkFinishedFunction = pathFollowerBuilder.checkFinishedFunction;
        this.decelerationFunction = pathFollowerBuilder.decelerationFunction;

        this.deceleration = pathFollowerBuilder.deceleration;
        this.decelerating = pathFollowerBuilder.decelerating;
        this.headingPID = pathFollowerBuilder.headingPID;
        this.endpointPID = pathFollowerBuilder.endpointPID;

        this.lookAhead = pathFollowerBuilder.lookAhead;
        this.endToleranceXY = pathFollowerBuilder.endToleranceXY;
        this.endToleranceR = pathFollowerBuilder.endToleranceR;
        this.endVelocityTolerance = pathFollowerBuilder.endVelocityTolerance;
    }

    public boolean isFinished() {
        return checkFinishedFunction.isFinished(this, endToleranceXY, endToleranceR, endVelocityTolerance);
    }

    public boolean isDecelerating() {
        return decelerating;
    }

    public void reset() {
        this.decelerating = false;
        pathContainer.reset();
        headingPID.reset();
        endpointPID.reset();
    }

    private Pose predictBrakeStop(Pose velocity) {
        return Pose.scale(Pose.square(velocity), -1 / (2 * deceleration));
    }

    public void update() {
        // will hold position if it is no longer finished at any point in time
        if (isFinished())
            driveController.drive(0, 0, 0);


        Pose robotPosition = tracker.getCurrentPosition().asVector();
        Pose nextPosition = pathContainer.getNextPosition(robotPosition, lookAhead);

        double heading = tracker.getCurrentPosition().getHeadingRadians();
        double rotate = -headingPID.getOutput(heading, normalizeAngle(nextPosition.getHeadingRadians(), heading));

        if (decelerating || (pathContainer.getI() == pathContainer.getN() - 1 && Pose.dist(robotPosition, endpoint) < predictBrakeStop(tracker.getRobotVelocity().asVector()).getMagnitude())) {
            decelerating = true;

            Pose drive = decelerationFunction.decelerate(this, heading);
            driveController.drive(drive.getX(), drive.getY(), rotate);

            return;
        }

        Pose vectorToNextPosition = Pose.subtract(nextPosition, robotPosition);

        double forward = vectorToNextPosition.getY();
        double strafe = vectorToNextPosition.getX();

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

        driveController.overdrive(forward, strafe, rotate);
    }

    public static class PathFollowerBuilder {
        private final DriveController driveController;
        private final DeterministicTracker tracker;
        private final PathContainer pathContainer;
        private CheckFinishedFunction checkFinishedFunction;
        private DecelerationFunction decelerationFunction;

        private double deceleration;
        private final boolean decelerating;
        private PID headingPID;
        private PID endpointPID;

        private double lookAhead;
        private double endToleranceXY;
        private double endToleranceR;
        private double endVelocityTolerance;

        public PathFollowerBuilder(DriveController driveController, DeterministicTracker tracker, PathContainer pathContainer) {
            this.driveController = driveController;
            this.tracker = tracker;
            this.pathContainer = pathContainer;

            this.checkFinishedFunction = DEFAULT_CHECK_FINISHED;
            this.decelerationFunction = DEFAULT_DECELERATION;

            this.deceleration = 1;
            this.decelerating = false;
            this.headingPID = new PID(0,0,0);
            this.endpointPID = new PID(0,0,0);

            this.lookAhead = 0.5;
            this.endToleranceXY = 0;
            this.endToleranceR = 0;
            this.endVelocityTolerance = 0;

            if (deceleration == 0) {
                throw new IllegalArgumentException("Deceleration cannot be 0");
            }
        }

        public PathFollowerBuilder setCheckFinishedFunction(CheckFinishedFunction checkFinishedFunction) {
            this.checkFinishedFunction = checkFinishedFunction;
            return this;
        }

        public PathFollowerBuilder setDecelerationFunction(DecelerationFunction decelerationFunction) {
            this.decelerationFunction = decelerationFunction;
            return this;
        }

        public PathFollowerBuilder setLookAhead(double lookAhead) {
            this.lookAhead = lookAhead;
            return this;
        }

        public PathFollowerBuilder setEndTolerance(double endToleranceXY, double endToleranceR) {
            this.endToleranceXY = endToleranceXY;
            this.endToleranceR = endToleranceR;
            return this;
        }

        public PathFollowerBuilder setEndVelocityTolerance(double endVelocityTolerance) {
            this.endVelocityTolerance = endVelocityTolerance;
            return this;
        }

        public PathFollowerBuilder setDeceleration(double deceleration) {
            this.deceleration = deceleration;
            return this;
        }

        public PathFollowerBuilder setHeadingPID(PID headingPID) {
            this.headingPID = headingPID;
            return this;
        }

        public PathFollowerBuilder setEndpointPID(PID endpointPID) {
            this.endpointPID = endpointPID;
            return this;
        }

        public PathFollowerBuilder setSpeed(double speed) {
            this.driveController.setDriveSpeed(speed);
            return this;
        }

        public PathFollower build() {
            return new PathFollower(this);
        }
    }
}
