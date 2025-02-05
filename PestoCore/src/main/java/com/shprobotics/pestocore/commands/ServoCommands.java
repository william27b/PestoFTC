package com.shprobotics.pestocore.commands;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class ServoCommands {
    public static void disableServo(Servo servo) {
        ((ServoControllerEx)servo.getController()).setServoPwmDisable(servo.getPortNumber());
    }

    public static void enableServo(Servo servo) {
        ((ServoControllerEx)servo.getController()).setServoPwmEnable(servo.getPortNumber());
    }

    public static void disableCRServo(CRServo crServo) {
        ((ServoControllerEx)crServo.getController()).setServoPwmDisable(crServo.getPortNumber());
    }

    public static void enableCRServo(CRServo crServo) {
        ((ServoControllerEx)crServo.getController()).setServoPwmEnable(crServo.getPortNumber());
    }
}
