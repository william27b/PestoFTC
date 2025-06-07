package com.shprobotics.pestocore.drivebases.wheels;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;

public class SwerveModule implements Wheel {
    private CortexLinkedMotor motor;
    private CortexLinkedServo servo;
    private AnalogInput servoPosition;

    public SwerveModule(CortexLinkedMotor dcMotorEx, CortexLinkedServo servo) {
        this.motor = dcMotorEx;
        this.servo = servo;
    }

    public double getServoPosition() {
        return servoPosition.getVoltage() / 3.3 * 360;
    }

    public void setPowerVector(Vector2D powerVector) {

    }

    @Override
    public void drive(double power) {

    }
}
