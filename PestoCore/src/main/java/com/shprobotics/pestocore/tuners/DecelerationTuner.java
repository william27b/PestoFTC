package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.geometries.Vector2D;

public abstract class DecelerationTuner extends LinearOpMode {
    private Vector2D before;
    private Vector2D velocity;

    public DriveController driveController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;

    public abstract void setDriveController(HardwareMap hardwareMap);
    public abstract void setTracker(HardwareMap hardwareMap);
    public abstract void setTeleOpController(DriveController driveController, DeterministicTracker tracker, HardwareMap hardwareMap);

    @Override
    public void runOpMode() {
        waitForStart();

        setDriveController(hardwareMap);
        setTracker(hardwareMap);
        setTeleOpController(driveController, tracker, hardwareMap);

        telemetry.addLine("drive at max speed, then press b to get deceleration value");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();
            teleOpController.updateSpeed(gamepad1);
            before = tracker.getCurrentPosition().asVector().copy();
            velocity = tracker.getRobotVelocity().asVector().copy();
        }

        driveController.drive(0, 0, 0);

        while (opModeIsActive() && !isStopRequested()) {
            tracker.update();
            telemetry.addData("deceleration", velocity.getMagnitude() / (2 * Vector2D.dist(before, tracker.getCurrentPosition().asVector())));
            telemetry.addData("max velocity", velocity.getMagnitude());
            telemetry.addData("distance travelled", 2 * Vector2D.dist(before, tracker.getCurrentPosition().asVector()));
            telemetry.update();
        }
    }
};