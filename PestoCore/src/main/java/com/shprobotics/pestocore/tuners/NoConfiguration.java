package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.devices.GamepadInterface;
import com.shprobotics.pestocore.devices.GamepadKey;
import com.shprobotics.pestocore.hardware.CortexLinkedMotor;
import com.shprobotics.pestocore.hardware.CortexLinkedServo;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "No Configuration", group = "Pesto Tuners")
public class NoConfiguration extends LinearOpMode {
    public void controlMotor(int port, boolean isParent) {
        CortexLinkedMotor motor = MotorCortex.getMotor(port, isParent);
        GamepadInterface gamepad = new GamepadInterface(gamepad1);
        int zeroedPosition = 0;

        while (opModeIsActive() && !isStopRequested() && !gamepad.isKeyUp(GamepadKey.B)) {
            MotorCortex.update();
            gamepad.update();

            if (gamepad.isKeyUp(GamepadKey.X)) {
                zeroedPosition = motor.getCurrentPosition();
            }

            motor.setPowerResult(-gamepad1.left_stick_y);

            telemetry.addLine("Press B to escape");
            telemetry.addLine("Press X to zero position");
            telemetry.addLine();
            telemetry.addLine("motor port" + port);
            telemetry.addData("power", -gamepad1.left_stick_y);
            telemetry.addData("position", motor.getCurrentPosition() - zeroedPosition);
            telemetry.update();
        }
    }

    public void controlServo(int port, boolean isParent) {
        CortexLinkedServo servo = MotorCortex.getServo(port, isParent);
        GamepadInterface gamepad = new GamepadInterface(gamepad1);

        while (opModeIsActive() && !isStopRequested() && !gamepad.isKeyUp(GamepadKey.B)) {
            MotorCortex.update();
            gamepad.update();

            servo.setPositionResult(-gamepad1.left_stick_y);

            telemetry.addLine("Press B to escape");
            telemetry.addLine();
            telemetry.addLine("servo port" + port);
            telemetry.addData("position / power", -gamepad1.left_stick_y);
            telemetry.update();
        }
    }

    public void mainScreen() {
        int mainScreen = 0;
        GamepadInterface gamepad = new GamepadInterface(gamepad1);

        while (opModeIsActive() && !isStopRequested()) {
            gamepad.update();

            if (gamepad.isKeyUp(GamepadKey.DPAD_DOWN) || gamepad.isKeyUp(GamepadKey.DPAD_UP)) {
                mainScreen += 1;
                mainScreen %= 2;
            }

            if (gamepad.isKeyUp(GamepadKey.X)) {
                if (mainScreen == 0) {
                    controlHub();
                } else {
                    expansionHub();
                }
            }

            telemetry.addLine("Use DPAD to select");
            telemetry.addLine("Press X to select");
            telemetry.addLine();
            telemetry.addLine((mainScreen == 0 ? "> " : "  ") + "Control Hub");
            telemetry.addLine((mainScreen == 1 ? "> " : "  ") + "Expansion Hub");
            telemetry.update();
        }
    }

    public void controlHub() {
        int controlHub = 0;
        GamepadInterface gamepad = new GamepadInterface(gamepad1);

        while (opModeIsActive() && !isStopRequested() && !gamepad.isKeyUp(GamepadKey.B)) {
            gamepad.update();

            if (gamepad.isKeyUp(GamepadKey.DPAD_DOWN)) {
                controlHub += 1;
                controlHub %= 10;
            }

            if (gamepad.isKeyUp(GamepadKey.DPAD_UP)) {
                controlHub += 9;
                controlHub %= 10;
            }

            if (gamepad.isKeyUp(GamepadKey.X)) {
                if (controlHub < 4) {
                    controlMotor(controlHub, true);
                } else {
                    controlServo(controlHub - 4, true);
                }
            }

            telemetry.addLine("Use DPAD to select");
            telemetry.addLine("Press X to select");
            telemetry.addLine("Press B to escape");
            telemetry.addLine();
            telemetry.addLine((controlHub == 0 ? "> " : "  ") + "Motor Port 0");
            telemetry.addLine((controlHub == 1 ? "> " : "  ") + "Motor Port 1");
            telemetry.addLine((controlHub == 2 ? "> " : "  ") + "Motor Port 2");
            telemetry.addLine((controlHub == 3 ? "> " : "  ") + "Motor Port 3");
            telemetry.addLine();
            telemetry.addLine((controlHub == 4 ? "> " : "  ") + "Servo Port 0");
            telemetry.addLine((controlHub == 5 ? "> " : "  ") + "Servo Port 1");
            telemetry.addLine((controlHub == 6 ? "> " : "  ") + "Servo Port 2");
            telemetry.addLine((controlHub == 7 ? "> " : "  ") + "Servo Port 3");
            telemetry.addLine((controlHub == 8 ? "> " : "  ") + "Servo Port 4");
            telemetry.addLine((controlHub == 9 ? "> " : "  ") + "Servo Port 5");
            telemetry.update();
        }
    }

    public void expansionHub() {
        int expansionHub = 0;
        GamepadInterface gamepad = new GamepadInterface(gamepad1);

        while (opModeIsActive() && !isStopRequested() && !gamepad.isKeyUp(GamepadKey.B)) {
            gamepad.update();

            if (gamepad.isKeyUp(GamepadKey.DPAD_DOWN)) {
                expansionHub += 1;
                expansionHub %= 10;
            }

            if (gamepad.isKeyUp(GamepadKey.DPAD_UP)) {
                expansionHub += 9;
                expansionHub %= 10;
            }

            if (gamepad.isKeyUp(GamepadKey.X)) {
                if (expansionHub < 4) {
                    controlMotor(expansionHub, false);
                } else {
                    controlServo(expansionHub - 4, false);
                }
            }

            telemetry.addLine("Use DPAD to select");
            telemetry.addLine("Press X to select");
            telemetry.addLine("Press B to escape");
            telemetry.addLine();
            telemetry.addLine((expansionHub == 0 ? "> " : "  ") + "Motor Port 0");
            telemetry.addLine((expansionHub == 1 ? "> " : "  ") + "Motor Port 1");
            telemetry.addLine((expansionHub == 2 ? "> " : "  ") + "Motor Port 2");
            telemetry.addLine((expansionHub == 3 ? "> " : "  ") + "Motor Port 3");
            telemetry.addLine();
            telemetry.addLine((expansionHub == 4 ? "> " : "  ") + "Servo Port 0");
            telemetry.addLine((expansionHub == 5 ? "> " : "  ") + "Servo Port 1");
            telemetry.addLine((expansionHub == 6 ? "> " : "  ") + "Servo Port 2");
            telemetry.addLine((expansionHub == 7 ? "> " : "  ") + "Servo Port 3");
            telemetry.addLine((expansionHub == 8 ? "> " : "  ") + "Servo Port 4");
            telemetry.addLine((expansionHub == 9 ? "> " : "  ") + "Servo Port 5");
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() {
        MotorCortex.initialize(hardwareMap);

        waitForStart();

        mainScreen();
    }
}
