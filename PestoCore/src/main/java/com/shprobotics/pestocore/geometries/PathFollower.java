package com.shprobotics.pestocore.geometries;

import com.shprobotics.pestocore.algorithms.PID;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.Tracker;

public class PathFollower {
    private final DriveController driveController;
    private final Tracker tracker;
    private final PathContainer pathContainer;
    private final PID headingPID;
    private final double speed;

    public PathFollower(PathFollowerBuilder pathFollowerBuilder) {
        this.driveController = pathFollowerBuilder.driveController;
        this.tracker = pathFollowerBuilder.tracker;
        this.pathContainer = pathFollowerBuilder.pathContainer;
        this.headingPID = pathFollowerBuilder.headingPID;
        this.speed = pathFollowerBuilder.speed;
    }

    public void update() {
        Pose2D robotPosition = tracker.getCurrentPosition();
        Vector2D robotVector = robotPosition.asVector();
        double heading = robotPosition.getHeadingRadians();

        Vector2D nextPosition = pathContainer.getNextPosition(robotVector);
        Vector2D vectorToNextPosition = Vector2D.subtract(nextPosition, robotVector);

        double forward = vectorToNextPosition.getY();
        double strafe = vectorToNextPosition.getX();
        double rotate = headingPID.getOutput(heading, pathContainer.getHeading());

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

        driveController.drive(forward, strafe, rotate);
    }

    public static class PathFollowerBuilder {
        private final DriveController driveController;
        private final Tracker tracker;
        private final PathContainer pathContainer;
        private final double speed;
        private final PID headingPID;

        public PathFollowerBuilder(DriveController driveController, Tracker tracker, PathContainer pathContainer, double speed, PID headingPID) {
            this.driveController = driveController;
            this.tracker = tracker;
            this.pathContainer = pathContainer;
            this.speed = speed;
            this.headingPID = headingPID;
        }

        public PathFollower build() {
            return new PathFollower(this);
        }
    }
}
