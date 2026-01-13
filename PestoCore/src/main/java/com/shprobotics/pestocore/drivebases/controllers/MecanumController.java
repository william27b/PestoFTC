package com.shprobotics.pestocore.drivebases.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.wheels.MecanumWheel;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;

public class MecanumController implements DriveController {
    //    protected CachingDcMotorEx frontLeft, frontRight, backLeft, backRight;
    public final MecanumWheel frontLeft, frontRight, backLeft, backRight;

    private Pose[] powerVectors = new Pose[]{new Pose(1, 1), new Pose(-1, 1), new Pose(-1, 1), new Pose(1, 1)};

    private double driveSpeed = 1;

    private double staticPower = 0.0;
    private boolean isStatic = false;

    public MecanumController(CortexLinkedMotor frontLeft, CortexLinkedMotor frontRight, CortexLinkedMotor backLeft, CortexLinkedMotor backRight) {
        this.frontLeft = new MecanumWheel(frontLeft);
        this.frontRight = new MecanumWheel(frontRight);
        this.backLeft = new MecanumWheel(backLeft);
        this.backRight = new MecanumWheel(backRight);
    }

    public MecanumController(HardwareMap hardwareMap, String[] motorNames) {
        this.frontLeft = new MecanumWheel(new CortexLinkedMotor(hardwareMap.get(DcMotorEx.class, motorNames[0])));
        this.frontRight = new MecanumWheel(new CortexLinkedMotor(hardwareMap.get(DcMotorEx.class, motorNames[1])));
        this.backLeft = new MecanumWheel(new CortexLinkedMotor(hardwareMap.get(DcMotorEx.class, motorNames[2])));
        this.backRight = new MecanumWheel(new CortexLinkedMotor(hardwareMap.get(DcMotorEx.class, motorNames[3])));
    }

    public MecanumController(HardwareMap hardwareMap) {
        this(hardwareMap, new String[]{"frontLeft", "frontRight", "backLeft", "backRight"});
    }

    public void setStaticPower(double staticPower) {
        this.staticPower = staticPower;
    }

    public void setIsStatic(boolean isStatic) {
        this.isStatic = isStatic;
    }

    public void setCachingTolerance(double cachingTolerance) {
        frontLeft.setCachingTolerance(cachingTolerance);
        frontRight.setCachingTolerance(cachingTolerance);
        backLeft.setCachingTolerance(cachingTolerance);
        backRight.setCachingTolerance(cachingTolerance);
    }

    @Override
    public void configureMotorDirections(DcMotorSimple.Direction[] directions) {
        frontLeft.setDirection(directions[0]);
        frontRight.setDirection(directions[1]);
        backLeft.setDirection(directions[2]);
        backRight.setDirection(directions[3]);
    }

    @Override
    public void configureMotorDirections(DcMotorSimple.Direction frontLeftDirection, DcMotorSimple.Direction frontRightDirection, DcMotorSimple.Direction backLeftDirection, DcMotorSimple.Direction backRightDirection) {
        frontLeft.setDirection(frontLeftDirection);
        frontRight.setDirection(frontRightDirection);
        backLeft.setDirection(backLeftDirection);
        backRight.setDirection(backRightDirection);
    }

    @Override
    public void setMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

    @Override
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPowerVectors(Pose[] powerVectors) {
        this.powerVectors = powerVectors;
    }

    public void setPowerVectors(Pose frontLeftVector, Pose frontRightVector, Pose backLeftVector, Pose backRightVector) {
        this.powerVectors = new Pose[]{frontLeftVector, frontRightVector, backLeftVector, backRightVector};
    }

    @Override
    public double getDriveSpeed() {
        return driveSpeed;
    }

    @Override
    public void setDriveSpeed(double driveSpeed) {
        try {
            assert driveSpeed >= 0 && driveSpeed <= 1;
        } catch (AssertionError e) {
            throw new AssertionError("DriveSpeed must be between 0 and 1");
        }
        this.driveSpeed = driveSpeed;
    }

    @Override
    public void overdrive(double forward, double strafe, double rotate) {
        double frontLeftPower = (powerVectors[0].getY() * forward) + (powerVectors[0].getX() * strafe) + rotate;
        double frontRightPower = (powerVectors[1].getY() * forward) + (powerVectors[1].getX() * strafe) - rotate;
        double backLeftPower = (powerVectors[2].getY() * forward) + (powerVectors[2].getX() * strafe) + rotate;
        double backRightPower = (powerVectors[3].getY() * forward) + (powerVectors[3].getX() * strafe) - rotate;

        frontLeftPower += staticPower * Math.signum(frontLeftPower) * (isStatic ? 1 : 0);
        frontRightPower += staticPower * Math.signum(frontRightPower) * (isStatic ? 1 : 0);
        backLeftPower += staticPower * Math.signum(backLeftPower) * (isStatic ? 1 : 0);
        backRightPower += staticPower * Math.signum(backRightPower) * (isStatic ? 1 : 0);

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        frontLeft.drive(frontLeftPower * this.driveSpeed / max);
        frontRight.drive(frontRightPower * this.driveSpeed / max);
        backLeft.drive(backLeftPower * this.driveSpeed / max);
        backRight.drive(backRightPower * this.driveSpeed / max);
    }

    @Override
    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = (powerVectors[0].getY() * forward) + (powerVectors[0].getX() * strafe) + rotate;
        double frontRightPower = (powerVectors[1].getY() * forward) + (powerVectors[1].getX() * strafe) - rotate;
        double backLeftPower = (powerVectors[2].getY() * forward) + (powerVectors[2].getX() * strafe) + rotate;
        double backRightPower = (powerVectors[3].getY() * forward) + (powerVectors[3].getX() * strafe) - rotate;

        frontLeftPower += staticPower * Math.signum(frontLeftPower) * (isStatic ? 1 : 0);
        frontRightPower += staticPower * Math.signum(frontRightPower) * (isStatic ? 1 : 0);
        backLeftPower += staticPower * Math.signum(backLeftPower) * (isStatic ? 1 : 0);
        backRightPower += staticPower * Math.signum(backRightPower) * (isStatic ? 1 : 0);

        double max = Math.max(1, Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        frontLeft.drive(frontLeftPower * this.driveSpeed / max);
        frontRight.drive(frontRightPower * this.driveSpeed / max);
        backLeft.drive(backLeftPower * this.driveSpeed / max);
        backRight.drive(backRightPower * this.driveSpeed / max);
    }
}
