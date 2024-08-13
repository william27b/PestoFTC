package com.shprobotics.pestocore.drivebases;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;

public class Odometry extends DcMotorImplEx {
    private double TICKS_PER_INCH = 1;
    private int lastCall = 0;
    private float direction = 1;

    public Odometry(DcMotor dcMotor, double ticksPerInch) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        this.TICKS_PER_INCH = ticksPerInch;
    }

    @Override
    public void setDirection(Direction direction) {
        if (direction == Direction.FORWARD) {
            this.direction = 1;
        } else {
            this.direction = -1;
        }
    }

    public void reset() {
        this.lastCall = this.getCurrentPosition();
    }

    public double getInchesTravelled() {
        int currentPosition = this.getCurrentPosition();
        int lastCall = this.lastCall;
        this.lastCall = currentPosition;
        return this.direction * (currentPosition - lastCall) / this.TICKS_PER_INCH;
    }
}
