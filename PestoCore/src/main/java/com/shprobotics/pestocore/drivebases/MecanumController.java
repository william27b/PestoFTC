package com.shprobotics.pestocore.drivebases;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.geometries.Vector2D;

public class MecanumController implements DriveController {
    protected DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private Vector2D[] powerVectors = new Vector2D[]{new Vector2D(1, 1), new Vector2D(-1, 1), new Vector2D(-1, 1), new Vector2D(1, 1)};

    private double driveSpeed = 1;

    public MecanumController(DcMotorEx frontLeft, DcMotorEx frontRight, DcMotorEx backLeft, DcMotorEx backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public MecanumController(HardwareMap hardwareMap, String[] motorNames) {
        this.frontLeft = hardwareMap.get(DcMotorEx.class, motorNames[0]);
        this.frontRight = hardwareMap.get(DcMotorEx.class, motorNames[1]);
        this.backLeft = hardwareMap.get(DcMotorEx.class, motorNames[2]);
        this.backRight = hardwareMap.get(DcMotorEx.class, motorNames[3]);
    }

    public MecanumController(HardwareMap hardwareMap) {
        this(hardwareMap, new String[]{"frontLeft", "frontRight", "backLeft", "backRight"});
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

    public void setPowerVectors(Vector2D[] powerVectors) {
        this.powerVectors = powerVectors;
    }

    public void setPowerVectors(Vector2D frontLeftVector, Vector2D frontRightVector, Vector2D backLeftVector, Vector2D backRightVector) {
        this.powerVectors = new Vector2D[]{frontLeftVector, frontRightVector, backLeftVector, backRightVector};
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

        double max = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        frontLeft.setPower(frontLeftPower * this.driveSpeed / max);
        frontRight.setPower(frontRightPower * this.driveSpeed / max);
        backLeft.setPower(backLeftPower * this.driveSpeed / max);
        backRight.setPower(backRightPower * this.driveSpeed / max);
    }

    @Override
    public void drive(double forward, double strafe, double rotate) {
        double frontLeftPower = (powerVectors[0].getY() * forward) + (powerVectors[0].getX() * strafe) + rotate;
        double frontRightPower = (powerVectors[1].getY() * forward) + (powerVectors[1].getX() * strafe) - rotate;
        double backLeftPower = (powerVectors[2].getY() * forward) + (powerVectors[2].getX() * strafe) + rotate;
        double backRightPower = (powerVectors[3].getY() * forward) + (powerVectors[3].getX() * strafe) - rotate;

        double max = Math.max(1, Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)), Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));

        frontLeft.setPower(frontLeftPower * this.driveSpeed / max);
        frontRight.setPower(frontRightPower * this.driveSpeed / max);
        backLeft.setPower(backLeftPower * this.driveSpeed / max);
        backRight.setPower(backRightPower * this.driveSpeed / max);
    }
}
