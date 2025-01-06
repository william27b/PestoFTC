package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;

public abstract class ForwardOffsetTuner extends LinearOpMode {
    public MecanumController mecanumController;
    public ThreeWheelOdometryTracker tracker;
    public TeleOpController teleOpController;

    public abstract void setMecanumController(HardwareMap hardwareMap);
    public abstract void setTracker(HardwareMap hardwareMap);
    public abstract void setTeleOpController(DriveController driveController, DeterministicTracker tracker, HardwareMap hardwareMap);

    @Override
    public void runOpMode() {
        setMecanumController(hardwareMap);
        setTracker(hardwareMap);
        setTeleOpController(mecanumController, tracker, hardwareMap);

        double distanceRotated;

        telemetry.addLine("rotate 10x times counter-clockwise; then press b");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();

            telemetry.addLine("rotate 10x times counter-clockwise; then press b");
            telemetry.addData("integral (left odometry)", tracker.leftOdometry.getTotalInchesTravelled());
            telemetry.addData("integral (center odometry)", tracker.centerOdometry.getTotalInchesTravelled());
            telemetry.addData("integral (right odometry)", tracker.rightOdometry.getTotalInchesTravelled());
            telemetry.update();
        }

        while (opModeIsActive() && !isStopRequested()) {
            tracker.update();

            distanceRotated = (tracker.leftOdometry.getTotalInchesTravelled() - tracker.rightOdometry.getTotalInchesTravelled()) / 2;

            telemetry.addData("Forward Offset", tracker.centerOdometry.getTotalInchesTravelled() / distanceRotated);
            telemetry.update();
        }
    }
};