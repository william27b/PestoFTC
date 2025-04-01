package com.shprobotics.pestocore.drivebases;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.shprobotics.pestocore.geometries.Vector2D;

public class SwerveModule {
    private DcMotorEx motor;
    private Servo servo;
    private AnalogInput servoPosition;

    public SwerveModule(DcMotorEx dcMotorEx, Servo servo) {
        this.motor = dcMotorEx;
        this.servo = servo;
    }

    public double getServoPosition() {
        return servoPosition.getVoltage() / 3.3 * 360;
    }

    public void setPowerVector(Vector2D powerVector) {

    }
}
