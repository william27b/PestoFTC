package com.shprobotics.pestocore.processing;

import com.acmerobotics.dashboard.message.MessageCache;
import com.acmerobotics.dashboard.message.redux.CachableMessage;
import com.acmerobotics.dashboard.message.redux.SetMotor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.shprobotics.pestocore.hardware.CortexLinkedCRServo;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;

import java.util.ArrayList;

public class MotorCortex {
    public static boolean motorsActivated = true;

    public static ArrayList<CortexLinkedMotor> motors;
    public static ArrayList<CortexLinkedServo> servos;
    public static ArrayList<CortexLinkedCRServo> crServos;

    public static HardwareMap hardwareMap;

    public static void initialize(HardwareMap hardwareMap) {
        motors = new ArrayList<>();

        MotorCortex.hardwareMap = hardwareMap;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public static CortexLinkedMotor getMotor(String name) {
        for (CortexLinkedMotor motor: motors) {
            if (motor.getDeviceName().equals(name))
                return motor;
        }

        if (hardwareMap == null)
            throw new HardwareException();

        CortexLinkedMotor motor = new CortexLinkedMotor((DcMotorEx) hardwareMap.get(name));
        motor.setName(name);

        motors.add(motor);

        return motor;
    }

    public static CortexLinkedServo getServo(String name) {
        for (CortexLinkedServo servo: servos) {
            if (servo.getDeviceName().equals(name))
                return servo;
        }

        if (hardwareMap == null)
            throw new HardwareException();

        CortexLinkedServo servo = new CortexLinkedServo((Servo) hardwareMap.get(name));
        servos.add(servo);

        return servo;
    }

    public static CortexLinkedCRServo getCRServo(String name) {
        for (CortexLinkedCRServo crServo: crServos) {
            if (crServo.getDeviceName().equals(name))
                return crServo;
        }

        if (hardwareMap == null)
            throw new HardwareException();

        CortexLinkedCRServo crServo = new CortexLinkedCRServo((CRServo) hardwareMap.get(name));
        crServos.add(crServo);

        return crServo;
    }

    public static void addMotor(CortexLinkedMotor motor) {
        for (CortexLinkedMotor storedMotor: motors) {
            if (storedMotor.getDeviceName().equals(motor.getDeviceName()))
                return;
        }

        motors.add(motor);
    }

    public static void addServo(CortexLinkedCRServo crServo) {
        crServos.add(crServo);
    }

    public static void addCRServo(CortexLinkedServo servo) {
        servos.add(servo);
    }

    public static void disableMotors() {
        for (CortexLinkedMotor motor: motors)
            motor.setPowerResult(0);
    }

    public static void setMotors() {
        int i = 0;

        while (i < MessageCache.getSize()) {
            CachableMessage command = MessageCache.getElement(i);
            i++;

            if (!(command instanceof SetMotor))
                continue;

            SetMotor setMotorCommand = (SetMotor) command;

            for (CortexLinkedMotor motor: motors) {
                // this getname function is jank
                if (setMotorCommand.getName().equals(motor.getName())) {
                    motor.setPowerResult(setMotorCommand.getPower());
                }
            }
        }
    }

    public static void update() {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.clearBulkCache();

        if (motorsActivated)
            return;

        disableMotors();
    }

    public static void reset() {}

    public static class MotorCommands {
        public static void disableMotor(DcMotor motor) {
            ((DcMotorControllerEx)motor.getController()).setMotorDisable(motor.getPortNumber());
        }

        public static void enableMotor(DcMotor motor) {
            ((DcMotorControllerEx)motor.getController()).setMotorEnable(motor.getPortNumber());
        }
    }

    public static class ServoCommands {
        public static void disableServo(Servo servo) {
            ((ServoControllerEx) servo.getController()).setServoPwmDisable(servo.getPortNumber());
        }

        public static void enableServo(Servo servo) {
            ((ServoControllerEx) servo.getController()).setServoPwmEnable(servo.getPortNumber());
        }

        public static void disableCRServo(CRServo crServo) {
            ((ServoControllerEx) crServo.getController()).setServoPwmDisable(crServo.getPortNumber());
        }

        public static void enableCRServo(CRServo crServo) {
            ((ServoControllerEx) crServo.getController()).setServoPwmEnable(crServo.getPortNumber());
        }
    }
}
