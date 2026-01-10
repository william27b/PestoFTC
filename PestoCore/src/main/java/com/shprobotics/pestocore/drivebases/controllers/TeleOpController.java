package com.shprobotics.pestocore.drivebases.controllers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.geometries.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Function;

public class TeleOpController {
    private final DriveController driveController;
    private Function<Gamepad, Double> speedController;
    private IMU imu;

    private boolean useIMU = true;
    private DeterministicTracker imuTracker;
    private double angleOffset;

    private boolean counteractCentripetalForce = false;
    private DeterministicTracker tracker;
    private double MAX_FORCE;

    public TeleOpController(DriveController driveController, HardwareMap hardwareMap) {
        this.driveController = driveController;
        this.imu = (IMU) hardwareMap.get("imu");
        this.angleOffset = 0;
    }

    public TeleOpController(DriveController driveController, IMU imu) {
        this.driveController = driveController;
        this.imu = imu;
        this.angleOffset = 0;
    }

    public void counteractCentripetalForce(DeterministicTracker tracker, double MAX_FORCE) {
        this.counteractCentripetalForce = true;
        this.tracker = tracker;
        this.MAX_FORCE = MAX_FORCE;
    }

    public void deactivateCentripetalForce() {
        this.counteractCentripetalForce = false;
    }

    public void setSpeedController(Function<Gamepad, Double> speedController) {
        this.speedController = speedController;
    }

    public void configureIMU(RevHubOrientationOnRobot orientationOnRobot) {
        try {
            assert imu != null;
        } catch (AssertionError e) {
            throw new AssertionError("IMU cannot be null when configureIMU() is called");
        }

        this.imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    public void configureIMU(RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection, RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection) {
        this.configureIMU(new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection));
    }

    public void useTrackerIMU(DeterministicTracker tracker) {
        this.useIMU = false;
        this.imuTracker = tracker;
    }

    public void useIMU() {
        this.useIMU = true;
    }

    public double getHeading() {
        if (useIMU) {
            return Math.toRadians(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + this.angleOffset);
        } else {
            return this.imuTracker.getCurrentPosition().getHeadingRadians();
        }
    }

    public void resetIMU() {
        this.angleOffset = -this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetIMU(double offset) {
        this.angleOffset = offset - this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void updateSpeed(Gamepad gamepad) {
        if (speedController != null) {
            this.driveController.setDriveSpeed(speedController.apply(gamepad));
        }
    }

    public void driveRobotCentric(double forward, double strafe, double rotate) {
        if (counteractCentripetalForce) {
            Pose centripetalForce = tracker.getCentripetalForce();
            centripetalForce.scale(1 / MAX_FORCE);

            forward += centripetalForce.getY();
            strafe += centripetalForce.getX();
        }

        driveController.drive(forward, strafe, rotate);
    }

    public void driveFieldCentric(double forward, double strafe, double rotate) {
        try {
            assert imu != null;
        } catch (AssertionError e) {
            throw new AssertionError("IMU cannot be null when drivingFieldCentric() is called");
        }

        double heading = this.getHeading();

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

        this.driveRobotCentric(forward, strafe, rotate);
    }
}
