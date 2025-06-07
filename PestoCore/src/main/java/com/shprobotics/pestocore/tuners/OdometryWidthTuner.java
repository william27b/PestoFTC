package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.processing.FrontalLobe;

@TeleOp(name = "Odometry Width Tuner", group = "Pesto Tuners")
public class OdometryWidthTuner extends LinearOpMode {
    public DriveController driveController;
    public ThreeWheelOdometryTracker tracker;
    public TeleOpController teleOpController;

    public double heading;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        heading = 0.0;

        driveController = FrontalLobe.driveController;

        assert FrontalLobe.tracker instanceof ThreeWheelOdometryTracker;
        tracker = (ThreeWheelOdometryTracker) FrontalLobe.tracker;

        teleOpController = FrontalLobe.teleOpController;

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

        driveController.drive(0, 0, 0);

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