package com.shprobotics.pestocore.drivebases.trackers;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

public class GoBildaPinpointTracker implements DeterministicTracker {
    GoBildaPinpointDriver goBildaPinpointDriver;

    private Pose2D currentPosition;
    private Pose2D deltaPosition;
    private Pose2D robotVelocity;

    public GoBildaPinpointTracker(HardwareMap hardwareMap, String deviceName) {
        this.goBildaPinpointDriver = hardwareMap.get(GoBildaPinpointDriver.class,deviceName);
    }

    public GoBildaPinpointTracker(GoBildaPinpointDriver goBildaPinpointDriver) {
        this.goBildaPinpointDriver = goBildaPinpointDriver;
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
    }

    public void reset(double heading) {
        goBildaPinpointDriver.resetPosAndIMU();
        goBildaPinpointDriver.setYawScalar(heading);
    }

    public void update() {
        deltaPosition = Pose2D.subtract(goBildaPinpointDriver.getPosition(), currentPosition, true);
        currentPosition = goBildaPinpointDriver.getPosition();

        robotVelocity = goBildaPinpointDriver.getVelocity();
    }

    public Pose2D getCurrentPosition() {
        return currentPosition;
    }
}
