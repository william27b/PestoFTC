package com.shprobotics.pestocore.commands;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;

public class MotorCommands {
    public static void disableMotor(DcMotor motor) {
        ((DcMotorControllerEx)motor.getController()).setMotorDisable(motor.getPortNumber());
    }

    public static void enableMotor(DcMotor motor) {
        ((DcMotorControllerEx)motor.getController()).setMotorEnable(motor.getPortNumber());
    }
}
