package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.geometries.Pose;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Localization Tuner", group = "Pesto Tuners")
public class LocalizationTuner extends LinearOpMode {
    public DriveController driveController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        driveController = FrontalLobe.driveController;
        tracker = FrontalLobe.tracker;
        teleOpController = FrontalLobe.teleOpController;

        tracker.reset();

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();

            Pose currentPosition = tracker.getCurrentPosition().asVector();
            Pose currentVelocity = tracker.getRobotVelocity();
            telemetry.addData("X", currentPosition.getX());
            telemetry.addData("Y", currentPosition.getY());
            telemetry.addData("Rotation", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.addLine();
            telemetry.addData("X velocity", currentVelocity.getX());
            telemetry.addData("Y velocity", currentVelocity.getY());
            telemetry.addData("Rotational velocity", currentVelocity.getHeadingRadians());
            telemetry.update();

            if (gamepad1.b) {
                tracker.reset();
            }
        }
    }
}
