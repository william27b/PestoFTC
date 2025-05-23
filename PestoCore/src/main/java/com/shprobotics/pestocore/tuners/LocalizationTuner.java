package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.TeleOpController;
import com.shprobotics.pestocore.geometries.Pose2D;
import com.shprobotics.pestocore.geometries.Vector2D;

public abstract class LocalizationTuner extends LinearOpMode {
    public DriveController driveController;
    public DeterministicTracker tracker;
    public TeleOpController teleOpController;

    public abstract void setDriveController(HardwareMap hardwareMap);
    public abstract void setTracker(HardwareMap hardwareMap);
    public abstract void setTeleOpController(DriveController driveController, DeterministicTracker tracker, HardwareMap hardwareMap);

    @Override
    public void runOpMode() {
        setDriveController(hardwareMap);
        setTracker(hardwareMap);
        setTeleOpController(driveController, tracker, hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            teleOpController.driveRobotCentric(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            tracker.update();

            Vector2D currentPosition = tracker.getCurrentPosition().asVector();
            Pose2D currentVelocity = tracker.getRobotVelocity();
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
