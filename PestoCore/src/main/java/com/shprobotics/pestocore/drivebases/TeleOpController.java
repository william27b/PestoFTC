package com.shprobotics.pestocore.drivebases;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.shprobotics.pestocore.geometries.Vector2D;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Function;

public class TeleOpController {
    private final DriveController driveController;
    private Function<Gamepad, Double> speedController;
    private IMU imu = null;

    private boolean useIMU;
    private Tracker imuTracker;
    private double angleOffset;

    private boolean counteractCentripetalForce = false;
    private Tracker tracker;
    private double MAX_VELOCITY;

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

    public void counteractCentripetalForce(Tracker tracker, double MAX_VELOCITY) {
        this.counteractCentripetalForce = true;
        this.tracker = tracker;
        this.MAX_VELOCITY = MAX_VELOCITY;
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

    public void useTrackerIMU(Tracker tracker) {
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
            return this.imuTracker.getCurrentHeading();
        }
    }

    public void resetIMU() {
        this.angleOffset = -this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void updateSpeed(Gamepad gamepad) {
        if (speedController != null) {
            this.driveController.setDriveSpeed(speedController.apply(gamepad));
        }
    }

    public void driveRobotCentric(double forward, double strafe, double rotate) {
        if (counteractCentripetalForce) {
            Vector2D centripetalForce = tracker.getCentripetalForce();
            centripetalForce.scale(1 / MAX_VELOCITY);

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
