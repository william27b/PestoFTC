package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.controllers.TeleOpController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.geometries.Vector2D;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Deceleration Tuner", group = "Pesto Tuners")
public class DecelerationTuner extends LinearOpMode {
    private Vector2D before;
    private Vector2D velocity;

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

        telemetry.addLine("drive at max speed, then press b to get deceleration value");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            MotorCortex.update();
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();
            teleOpController.updateSpeed(gamepad1);
            before = tracker.getCurrentPosition().asVector().copy();
            velocity = tracker.getRobotVelocity().asVector().copy();
        }

        driveController.drive(0, 0, 0);

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();
            telemetry.addData("deceleration", velocity.getMagnitude() / (2 * Vector2D.dist(before, tracker.getCurrentPosition().asVector())));
            telemetry.addData("max velocity", velocity.getMagnitude());
            telemetry.addData("distance travelled", 2 * Vector2D.dist(before, tracker.getCurrentPosition().asVector()));
            telemetry.update();
        }
    }
};