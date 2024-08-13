package com.shprobotics.pestocore.geometries;

import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.Tracker;

public class PathFollower {
    private final DriveController driveController;
    private final Tracker tracker;
    private final PathContainer pathContainer;
    private final Vector2D endpoint;
    private final double deceleration;
    private final PID headingPID;
    private final PID endpointPID;

    private boolean decelerate;

    public PathFollower(PathFollowerBuilder pathFollowerBuilder) {
        this.driveController = pathFollowerBuilder.driveController;
        this.tracker = pathFollowerBuilder.tracker;
        this.pathContainer = pathFollowerBuilder.pathContainer;
        this.endpoint = pathFollowerBuilder.pathContainer.getEndpoint();
        this.deceleration = pathFollowerBuilder.deceleration;
        this.headingPID = pathFollowerBuilder.headingPID;
        this.endpointPID = pathFollowerBuilder.endpointPID;

        this.decelerate = false;
    }

    public void reset() {
        pathContainer.reset();
        headingPID.reset();
        endpointPID.reset();
    }

    private Vector2D predictBrakeStop(Vector2D velocity) {
        return Vector2D.scale(Vector2D.square(velocity), -1 / (2 * deceleration));
    }

    public void update() {
        Vector2D robotPosition = tracker.getCurrentPosition();
        double heading = tracker.getCurrentHeading();
        double rotate = -headingPID.getOutput(heading, pathContainer.getHeading());
//        double rotate = 0;

        if (Vector2D.fastdist(robotPosition, endpoint) < Vector2D.fastdist(predictBrakeStop(tracker.getRobotVelocity().asVector()), Vector2D.ZERO)) {
            Vector2D vectorToEndpoint = Vector2D.subtract(endpoint, predictBrakeStop(tracker.getRobotVelocity().asVector()));
            double forward = vectorToEndpoint.getY();
            double strafe = vectorToEndpoint.getX();

            // reconsider
            double drivePower = endpointPID.getOutput(robotPosition.getMagnitude(), endpoint.getMagnitude());

            driveController.drive(forward * drivePower, strafe * drivePower, rotate);

//            throw new RuntimeException("here");
            return;
        }

        Vector2D nextPosition = pathContainer.getNextPosition(robotPosition);
        Vector2D vectorToNextPosition = Vector2D.subtract(nextPosition, robotPosition);

        double forward = vectorToNextPosition.getY();
        double strafe = vectorToNextPosition.getX();

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

//        driveController.overdrive(forward, strafe, rotate);
        driveController.drive(forward, strafe, rotate);
    }

    public static class PathFollowerBuilder {
        private final DriveController driveController;
        private final Tracker tracker;
        private final PathContainer pathContainer;
        private final double deceleration;
        private final PID headingPID;
        private final PID endpointPID;

        public PathFollowerBuilder(DriveController driveController, Tracker tracker, PathContainer pathContainer, double speed, double deceleration, PID headingPID, PID endpointPID) {
            this.driveController = driveController;
            this.tracker = tracker;
            this.pathContainer = pathContainer;
            this.deceleration = deceleration;
            this.driveController.setDriveSpeed(speed);
            this.headingPID = headingPID;
            this.endpointPID = endpointPID;

            if (deceleration == 0) {
                throw new IllegalArgumentException("Deceleration cannot be 0");
            }
        }

        public PathFollower build() {
            return new PathFollower(this);
        }
    }
}
