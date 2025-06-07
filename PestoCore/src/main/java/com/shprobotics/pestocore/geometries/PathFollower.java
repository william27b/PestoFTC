package com.shprobotics.pestocore.geometries;

import static org.apache.commons.math3.util.MathUtils.normalizeAngle;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;

public class PathFollower {
    @FunctionalInterface
    public interface DecelerationFunction {
        Vector2D decelerate(PathFollower pathFollower, double rotate);
    }

    public static final DecelerationFunction SQUID_DECELERATION = (PathFollower pathFollower, double heading) -> {
        Vector2D vectorToEndpoint = Vector2D.subtract(pathFollower.endpoint, pathFollower.predictBrakeStop(pathFollower.tracker.getRobotVelocity().asVector()));
        double forward = vectorToEndpoint.getY();
        double strafe = vectorToEndpoint.getX();

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

        // reconsider
        double drivePower = pathFollower.endpointPID.getOutput(pathFollower.tracker.getCurrentPosition().getMagnitude(), pathFollower.endpoint.getMagnitude());

        // SQUID
        drivePower = Math.sqrt(drivePower);

        Vector2D drive = new Vector2D(
                forward,
                strafe
        );

        drive.scale(drivePower);

        return drive;
    };

    public static final DecelerationFunction DEFAULT_DECELERATION = (PathFollower pathFollower, double heading) -> {
        Vector2D vectorToEndpoint = Vector2D.subtract(pathFollower.endpoint, pathFollower.predictBrakeStop(pathFollower.tracker.getRobotVelocity().asVector()));
        double forward = vectorToEndpoint.getY();
        double strafe = vectorToEndpoint.getX();

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

        // reconsider
        double drivePower = pathFollower.endpointPID.getOutput(pathFollower.tracker.getCurrentPosition().getMagnitude(), pathFollower.endpoint.getMagnitude());

        Vector2D drive = new Vector2D(
                forward,
                strafe
        );

        drive.scale(drivePower);

        return drive;
    };

    @FunctionalInterface
    public interface CheckFinishedFunction {
        boolean isFinished(PathFollower pathFollower, double toleranceXY, double toleranceR);
    }

    public static final CheckFinishedFunction DEFAULT_CHECK_FINISHED = (PathFollower pathFollower, double toleranceXY1, double toleranceR1) -> (
            Vector2D.dist(pathFollower.tracker.getCurrentPosition().asVector(), pathFollower.pathContainer.getEndpoint()) < toleranceXY1
                    && normalizeAngle(pathFollower.tracker.getCurrentPosition().getHeadingRadians() - pathFollower.pathContainer.getHeading(), Math.PI) < toleranceR1
    );

    private final DriveController driveController;
    private final DeterministicTracker tracker;
    private final PathContainer pathContainer;
    private final Vector2D endpoint;
    private final CheckFinishedFunction checkFinishedFunction;
    private final DecelerationFunction decelerationFunction;

    private final double deceleration;
    private boolean decelerating;
    private final PID headingPID;
    private final PID endpointPID;

    private final Runnable finalAction;
    private final double endToleranceXY;
    private final double endToleranceR;
    private final double endVelocityTolerance;
    private final double timeAfterDeceleration;
    private final double killTime;
    private ElapsedTime killTimer;
    private final ElapsedTime pathTime;
    private boolean completed;

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

        this.finalAction = pathFollowerBuilder.finalAction;
        this.endToleranceXY = pathFollowerBuilder.endToleranceXY;
        this.endToleranceR = pathFollowerBuilder.endToleranceR;
        this.endVelocityTolerance = pathFollowerBuilder.endVelocityTolerance;
        this.timeAfterDeceleration = pathFollowerBuilder.timeAfterDeceleration;
        this.killTime = pathFollowerBuilder.killTime;
        this.killTimer = null;
        this.pathTime = pathFollowerBuilder.pathTime;
        this.completed = pathFollowerBuilder.completed;
    }

    public boolean isFinished(double toleranceXY, double toleranceR) {
        return checkFinishedFunction.isFinished(this, toleranceXY, toleranceR);
    }

    public boolean isCompleted() {
        return completed;
    }

    public boolean isDecelerating() {
        return decelerating;
    }

    public void reset() {
        this.decelerating = false;
        this.killTimer = null;
        pathContainer.reset();
        headingPID.reset();
        endpointPID.reset();
    }

    private Vector2D predictBrakeStop(Vector2D velocity) {
        return Vector2D.scale(Vector2D.square(velocity), -1 / (2 * deceleration));
    }

    public void update() {
        if (completed)
            return;

        if (killTimer == null) {
            killTimer = new ElapsedTime();
            killTimer.reset();
        }

        if (
                (isFinished(endToleranceXY, endToleranceR)
                && (tracker.getRobotVelocity().getMagnitude() < endVelocityTolerance))
                || (decelerating
                && (pathTime.seconds() > timeAfterDeceleration))
                || (killTimer.seconds() > killTime)
        ) {
            completed = true;
            if (finalAction != null) {
                driveController.drive(0, 0, 0);
                finalAction.run();
            }
        }

        pathContainer.updateHeading(tracker.getCurrentPosition().getHeadingRadians());

        Vector2D robotPosition = tracker.getCurrentPosition().asVector();
        double heading = tracker.getCurrentPosition().getHeadingRadians();
        double rotate = -headingPID.getOutput(heading, normalizeAngle(pathContainer.getHeading(), heading));

        if (decelerating || (pathContainer.getI() == pathContainer.getN() - 1 && Vector2D.fastdist(robotPosition, endpoint) < Vector2D.fastdist(predictBrakeStop(tracker.getRobotVelocity().asVector()), Vector2D.ZERO))) {
            if (!decelerating)
                pathTime.reset();
            decelerating = true;

            Vector2D drive = decelerationFunction.decelerate(this, heading);
            driveController.drive(drive.getX(), drive.getY(), rotate);

            return;
        }

        Vector2D nextPosition = pathContainer.getNextPosition(robotPosition, heading);
        Vector2D vectorToNextPosition = Vector2D.subtract(nextPosition, robotPosition);

        double forward = vectorToNextPosition.getY();
        double strafe = vectorToNextPosition.getX();

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

        driveController.drive(forward, strafe, rotate);
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

        private Runnable finalAction;
        private double endToleranceXY;
        private double endToleranceR;
        private double endVelocityTolerance;
        private double timeAfterDeceleration;
        private double killTime;
        private final ElapsedTime pathTime;
        private final boolean completed;

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

            this.finalAction = null;
            this.endToleranceXY = 0;
            this.endToleranceR = 0;
            this.endVelocityTolerance = 0;
            this.timeAfterDeceleration = Double.POSITIVE_INFINITY;
            this.killTime = Double.POSITIVE_INFINITY;
            this.pathTime = new ElapsedTime();
            this.completed = false;

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

        public PathFollowerBuilder addFinalAction(Runnable action) {
            this.finalAction = action;
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

        public PathFollowerBuilder setTimeAfterDeceleration(double timeAfterDeceleration) {
            this.timeAfterDeceleration = timeAfterDeceleration;
            return this;
        }

        public PathFollowerBuilder setDeceleration(double deceleration) {
            this.deceleration = deceleration;
            return this;
        }

        public PathFollowerBuilder setKillTime(double killTime) {
            this.killTime = killTime;
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
