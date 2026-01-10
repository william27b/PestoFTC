package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.algorithms.Constants;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Force Tuner", group = "Pesto Tuners")
public class ForceTuner extends LinearOpMode {
    private double distance = 100;

    public DriveController driveController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        driveController = FrontalLobe.driveController;
        tracker = FrontalLobe.tracker;
        teleOpController = FrontalLobe.teleOpController;

        waitForStart();

        telemetry.addLine("press b to drive forward at 100% speed for " + distance + " inches");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            MotorCortex.update();
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();
            teleOpController.updateSpeed(gamepad1);
        }

        double startTime = System.nanoTime() / 1E9;

        tracker.reset();

        driveController.drive(1, 0, 0);

        while (opModeIsActive() && !isStopRequested() && tracker.getCurrentPosition().getY() < distance) {
            if (gamepad1.b)
                return;

            MotorCortex.update();
            tracker.update();
        }

        double endTime = System.nanoTime() / 1E9;
        double deltaTime = endTime - startTime;

        double acceleration = 2 * distance / (deltaTime * deltaTime);

        driveController.drive(0, 0, 0);

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();
            telemetry.addData("acceleration", acceleration);
            telemetry.addData("mass is configured as ", Constants.mass);
            telemetry.addData("distance travelled", tracker.getCurrentPosition().asVector().getMagnitude());
            telemetry.addData("F = ma = ", Constants.mass * acceleration);
            telemetry.update();
        }
    }
};