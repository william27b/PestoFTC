package com.shprobotics.pestocore.drivebases;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public interface DriveController {
    void configureMotorDirections(DcMotorSimple.Direction[] directions);
    void configureMotorDirections(DcMotorSimple.Direction frontLeftDirection, DcMotorSimple.Direction frontRightDirection, DcMotorSimple.Direction backLeftDirection, DcMotorSimple.Direction backRightDirection);
    void setRunMode(DcMotor.RunMode runMode);
    double getDriveSpeed();
    void setDriveSpeed(double speed);
    void drive(double forward, double strafe, double rotate);
}
