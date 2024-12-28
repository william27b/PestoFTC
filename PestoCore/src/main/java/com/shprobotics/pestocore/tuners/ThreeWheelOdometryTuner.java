package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Pose2D;

public abstract class ThreeWheelOdometryTuner extends LinearOpMode {
    public MecanumController mecanumController;
    public ThreeWheelOdometryTracker tracker;
    public TeleOpController teleOpController;

    public abstract void setMecanumController(HardwareMap hardwareMap);
    public abstract void setTracker(HardwareMap hardwareMap);
    public abstract void setTeleOpController(MecanumController mecanumController, DeterministicTracker tracker, HardwareMap hardwareMap);

    public double heading;

    @Override
    public void runOpMode() {
        heading = 0.0;

        setMecanumController(hardwareMap);
        setTracker(hardwareMap);
        setTeleOpController(mecanumController, tracker, hardwareMap);

        telemetry.addLine("rotate 10x times counter-clockwise, then press b");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();

            Pose2D deltaPosition = tracker.getDeltaPosition();
            heading += deltaPosition.getHeadingRadians();

            teleOpController.updateSpeed(gamepad1);
        }

        mecanumController.drive(0, 0, 0);

        while (opModeIsActive() && !isStopRequested()) {
            tracker.update();

            Pose2D deltaPosition = tracker.getDeltaPosition();
            heading += deltaPosition.getHeadingRadians();

            telemetry.addData("heading total", heading);
            telemetry.addData("x position", tracker.getCurrentPosition().getX());
            telemetry.addLine();
            telemetry.addData("forward offset", tracker.centerOdometry.getTotalInchesTravelled() / heading);
            telemetry.addData("odometry width", tracker.ODOMETRY_WIDTH * Math.PI * 20 / heading);
            telemetry.update();
        }
    }
};