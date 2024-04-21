package com.shprobotics.pestocore.drivebases;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.function.Function;

public class TeleOpController {
    private final DriveController driveController;
    private Function<Gamepad, Double> speedController;
    private IMU imu = null;

    public TeleOpController(DriveController driveController, HardwareMap hardwareMap) {
        this.driveController = driveController;
        this.imu = (IMU) hardwareMap.get("imu");
    }

    public TeleOpController(DriveController driveController, IMU imu) {
        this.driveController = driveController;
        this.imu = imu;
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

    public void updateSpeed(Gamepad gamepad) {
        if (speedController != null) {
            this.driveController.setDriveSpeed(speedController.apply(gamepad));
        }
    }

    public void driveRobotCentric(double forward, double strafe, double rotate) {
        driveController.drive(forward, strafe, rotate);
    }

    public void driveFieldCentric(double forward, double strafe, double rotate) {
        try {
            assert imu != null;
        } catch (AssertionError e) {
            throw new AssertionError("IMU cannot be null when drivingFieldCentric() is called");
        }

        double heading = Math.toRadians(this.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + 90);

        double temp = forward * Math.cos(heading) + strafe * Math.sin(-heading);
        strafe = forward * Math.sin(heading) + strafe * Math.cos(heading);
        forward = temp;

        driveController.drive(forward, strafe, rotate);
    }
}
