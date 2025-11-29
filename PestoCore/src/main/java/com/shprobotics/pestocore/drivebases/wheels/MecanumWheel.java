package com.shprobotics.pestocore.drivebases.wheels;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;

public class MecanumWheel implements Wheel {
    public final CortexLinkedMotor motor;

    private double accelerationMax = Double.POSITIVE_INFINITY;
    private double currentVelocity = 0.0;
    private double deltaTime = Double.POSITIVE_INFINITY;

    public MecanumWheel(CortexLinkedMotor motor) {
        this.motor = motor;
    }

    public void setCachingTolerance(double cachingTolerance) {
        motor.setCachingTolerance(cachingTolerance);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setMode(DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        motor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setAccelerationMax(double accelerationMax) {
        this.accelerationMax = accelerationMax;
    }

    public void updateStatus(double currentVelocity, double deltaTime) {
        this.currentVelocity = currentVelocity;
        this.deltaTime = deltaTime;
    }

    @Override
    public void drive(double power) {
        if (accelerationMax == Double.POSITIVE_INFINITY || deltaTime == 0)
            motor.setPowerResult(power);

        double minVelocity = currentVelocity - accelerationMax * deltaTime;
        double maxVelocity = currentVelocity + accelerationMax * deltaTime;

        motor.setPowerResult(min(max(power, minVelocity), maxVelocity));
    }
}
