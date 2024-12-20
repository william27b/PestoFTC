package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.geometries.Vector2D;

public abstract class DecelerationTuner extends LinearOpMode {
    private Vector2D before;
    public MecanumController mecanumController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;

    public abstract void setMecanumController(HardwareMap hardwareMap);
    public abstract void setTracker(HardwareMap hardwareMap);
    public abstract void setTeleOpController(MecanumController mecanumController, DeterministicTracker tracker, HardwareMap hardwareMap);

    @Override
    public void runOpMode() {
        waitForStart();

        setMecanumController(hardwareMap);
        setTracker(hardwareMap);
        setTeleOpController(mecanumController, tracker, hardwareMap);

        telemetry.addLine("drive at max speed, then press b to get deceleration value");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();
            teleOpController.updateSpeed(gamepad1);
            before = tracker.getCurrentPosition().asVector().copy();
        }

        mecanumController.drive(0, 0, 0);

        while (opModeIsActive() && !isStopRequested()) {
            tracker.update();
            telemetry.addData("deceleration", Vector2D.dist(before, tracker.getCurrentPosition().asVector()));
            telemetry.update();
        }
    }
};