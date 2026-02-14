package com.shprobotics.pestocore.processing;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.HardwareDeviceManager;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorControllerEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DeviceManager;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.ServoConfigurationType;
import com.shprobotics.pestocore.hardware.CortexLinkedCRServo;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

import java.lang.annotation.Annotation;
import java.util.ArrayList;

public class MotorCortex {
    public static boolean motorsActivated = true;

    public static ArrayList<CortexLinkedMotor> motors;
    public static ArrayList<CortexLinkedServo> servos;
    public static ArrayList<CortexLinkedCRServo> crServos;

    public static HardwareMap hardwareMap;

    public static void initialize(HardwareMap hardwareMap) {
        motors = new ArrayList<>();
        servos = new ArrayList<>();
        crServos = new ArrayList<>();

        MotorCortex.hardwareMap = hardwareMap;

        for (LynxModule module : hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public static CortexLinkedMotor getMotor(int port, boolean isParent) {
        // From HardwareFactory.java
        LynxDcMotorController controller = null;
        try {
            for (LynxModule module: hardwareMap.getAll(LynxModule.class)) {
                if (module.isParent() != isParent)
                    continue;

                controller = new LynxDcMotorController(hardwareMap.appContext, module);
            }
        }  catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }

        assert controller != null;

        // USB Scan Manager.java uses null manager?
        DeviceManager deviceMgr = new HardwareDeviceManager(hardwareMap.appContext, null);

        // Using GoBILDA 5203 Motor Configuration
        MotorConfigurationType motorConfigurationType = new MotorConfigurationType();
        motorConfigurationType.setTicksPerRev(505.3169);
        motorConfigurationType.setGearing(99.5);
        motorConfigurationType.setMaxRPM(60);
        motorConfigurationType.setOrientation(Rotation.CCW);

        DcMotor m = deviceMgr.createDcMotorEx(controller, port, motorConfigurationType, motorConfigurationType.getName());

        // Since it is not automatically enabled, we manually enable it
        MotorCommands.enableMotor(m);

        // Cast to CortexLinkedMotor
        CortexLinkedMotor motor = new CortexLinkedMotor((DcMotorEx) m);

        return motor;
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

    public static CortexLinkedServo getServo(int port, boolean isParent) {
        // From HardwareFactory.java
        LynxServoController controller = null;
        try {
            for (LynxModule module: hardwareMap.getAll(LynxModule.class)) {
                if (module.isParent() != isParent)
                    continue;

                controller = new LynxServoController(hardwareMap.appContext, module);
            }
        }  catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }

        assert controller != null;

        // USB Scan Manager.java uses null manager?
        DeviceManager deviceMgr = new HardwareDeviceManager(hardwareMap.appContext, null);

        // Using default servo configuration
        ServoConfigurationType servoConfigurationType = new ServoConfigurationType();

        ServoType servoAnnotation = new ServoType() {
            @Override
            public Class<? extends Annotation> annotationType() {
                return ServoType.class;
            }

            @NonNull
            @Override
            public ServoFlavor flavor() {
                return ServoFlavor.STANDARD;
            }

            @Override
            public double usPulseLower() {
                return PwmControl.PwmRange.usPulseLowerDefault;
            }

            @Override
            public double usPulseUpper() {
                return PwmControl.PwmRange.usPulseUpperDefault;
            }

            @Override
            public double usPulseFrameRate() {
                return PwmControl.PwmRange.usFrameDefault;
            }
        };

        servoConfigurationType.processAnnotation(servoAnnotation);

        Servo s = deviceMgr.createServoEx(controller, port, servoConfigurationType.getName(), servoConfigurationType);

        // Since it is not automatically enabled, we manually enable it
        ServoCommands.enableServo(s);

        // Cast to CortexLinkedServo
        CortexLinkedServo servo = new CortexLinkedServo(s);
        servos.add(servo);

        return servo;
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

    public static CortexLinkedCRServo getCRServo(int port, boolean isParent) {
        // From HardwareFactory.java
        LynxServoController controller = null;
        try {
            for (LynxModule module: hardwareMap.getAll(LynxModule.class)) {
                if (module.isParent() != isParent)
                    continue;

                controller = new LynxServoController(hardwareMap.appContext, module);
            }
        }  catch (RobotCoreException | InterruptedException e) {
            throw new RuntimeException(e);
        }

        assert controller != null;

        // USB Scan Manager.java uses null manager?
        DeviceManager deviceMgr = new HardwareDeviceManager(hardwareMap.appContext, null);

        // Using GoBILDA 5203 Motor Configuration
        ServoConfigurationType servoConfigurationType = new ServoConfigurationType();

        CRServo s = deviceMgr.createCRServoEx(controller, port, servoConfigurationType.getName(), servoConfigurationType);

        // Since it is not automatically enabled, we manually enable it
        ServoCommands.enableCRServo(s);

        // Cast to CortexLinkedServo
        CortexLinkedCRServo servo = new CortexLinkedCRServo(s);
        crServos.add(servo);

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
