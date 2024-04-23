package com.shprobotics.pestocore.drivebases;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

public class Odometry extends DcMotorImplEx {
    private double TICKS_PER_INCH = 1;
    private int lastCall = 0;

    public Odometry(DcMotorController controller, int portNumber) {
        super(controller, portNumber);
        this.setMode(RunMode.RUN_USING_ENCODER);
    }

    public Odometry(DcMotorController controller, int portNumber, Direction direction) {
        super(controller, portNumber, direction);
        this.setMode(RunMode.RUN_USING_ENCODER);
    }

    public Odometry(DcMotorController controller, int portNumber, Direction direction, @NonNull MotorConfigurationType motorType) {
        super(controller, portNumber, direction, motorType);
        this.setMode(RunMode.RUN_USING_ENCODER);
    }

    public Odometry(DcMotor dcMotor, double ticksPerInch) {
        super(dcMotor.getController(), dcMotor.getPortNumber());
        this.setDirection(dcMotor.getDirection());
        this.setMode(RunMode.RUN_USING_ENCODER);

        this.TICKS_PER_INCH = ticksPerInch;
    }

    public void reset() {
        this.lastCall = this.getCurrentPosition();
    }

    public double getInchesTravelled() {
        int currentPosition = this.getCurrentPosition();
        int lastCall = this.lastCall;
        // TODO: ADD THIS BACK!!!!
        this.lastCall = currentPosition;
        return (currentPosition - lastCall) / this.TICKS_PER_INCH;
    }
}
