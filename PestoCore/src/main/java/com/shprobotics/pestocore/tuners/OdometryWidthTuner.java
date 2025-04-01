package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Pose2D;

public abstract class OdometryWidthTuner extends LinearOpMode {
    public DriveController driveController;
    public ThreeWheelOdometryTracker tracker;
    public TeleOpController teleOpController;

    public abstract void setDriveController(HardwareMap hardwareMap);
    public abstract void setTracker(HardwareMap hardwareMap);
    public abstract void setTeleOpController(DriveController driveController, DeterministicTracker tracker, HardwareMap hardwareMap);

    public double heading;

    @Override
    public void runOpMode() {
        heading = 0.0;

        setDriveController(hardwareMap);
        setTracker(hardwareMap);
        setTeleOpController(driveController, tracker, hardwareMap);

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
            telemetry.addLine();

            telemetry.addData("odometry width", tracker.ODOMETRY_WIDTH * heading / (Math.PI * 20));
            telemetry.update();
        }
    }
};