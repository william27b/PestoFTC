package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.trackers.GoBildaPinpointTracker;
import com.shprobotics.pestocore.drivebases.trackers.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.drivebases.trackers.TwoWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Odometry Width Tuner", group = "Pesto Tuners")
public class OdometryWidthTuner extends LinearOpMode {
    public DriveController driveController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;

    public double heading;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        heading = 0.0;

        driveController = FrontalLobe.driveController;
        tracker = FrontalLobe.tracker;
        teleOpController = FrontalLobe.teleOpController;

        assert (tracker instanceof GoBildaPinpointTracker) || (tracker instanceof TwoWheelOdometryTracker) || (tracker instanceof ThreeWheelOdometryTracker);
        tracker = FrontalLobe.tracker;

        teleOpController = FrontalLobe.teleOpController;

        telemetry.addLine("rotate 10x times counter-clockwise, then press b");
        telemetry.update();

        waitForStart();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            MotorCortex.update();
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();

            Pose deltaPosition = tracker.getDeltaPosition();
            heading += deltaPosition.getHeadingRadians();

            teleOpController.updateSpeed(gamepad1);
        }

        driveController.drive(0, 0, 0);

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();

            Pose deltaPosition = tracker.getDeltaPosition();
            heading += deltaPosition.getHeadingRadians();

            telemetry.addData("heading total", heading);
            telemetry.addLine();

            if (tracker instanceof ThreeWheelOdometryTracker)
                telemetry.addData("odometry width", ((ThreeWheelOdometryTracker) tracker).ODOMETRY_WIDTH * heading / (Math.PI * 20));
            if (tracker instanceof TwoWheelOdometryTracker)
                telemetry.addData("odometry width", ((TwoWheelOdometryTracker) tracker).ODOMETRY_WIDTH * heading / (Math.PI * 20));
            if (tracker instanceof GoBildaPinpointTracker)
                telemetry.addData("odometry width", ((GoBildaPinpointTracker) tracker).ODOMETRY_WIDTH * heading / (Math.PI * 20));

            telemetry.update();
        }
    }
};