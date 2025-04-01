package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.DriveController;

import java.util.ArrayList;

public abstract class StrafeDecelerationTuner extends LinearOpMode {
    private ArrayList<Double> accelerations = new ArrayList<>();

    protected DeterministicTracker tracker;
    protected DriveController driveController;

    protected double targetVelocity;
    private boolean stopping;

    public abstract void setTracker();
    public abstract void setDriveController();
    public abstract void setTargetVelocity();

    @Override
    public void runOpMode() {
        setTracker();
        setDriveController();
        setTargetVelocity();

        driveController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("The robot will run to the right until it reaches " + targetVelocity + " inches per second.");
        telemetry.addLine("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.addLine("After stopping, the strafe zero power acceleration (natural deceleration) will be displayed.");
        telemetry.addLine("Press CROSS or A on game pad 1 to stop.");
        telemetry.update();

        waitForStart();

        double previousVelocity = 0.0;
        double previousTimeNano = System.nanoTime();

        driveController.drive(0, 1, 0);

        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                driveController.drive(0, 0, 0);
                driveController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                requestOpModeStop();
            }

            tracker.update();

            double currentVelocity = tracker.getRobotVelocity().getMagnitude();

            if (!stopping) {
                if (currentVelocity > targetVelocity) {
                    previousVelocity = currentVelocity;
                    previousTimeNano = System.nanoTime();
                    stopping = true;

                    driveController.drive(0, 0, 0);
                    driveController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                }
            } else {
                accelerations.add((currentVelocity - previousVelocity) / ((System.nanoTime() - previousTimeNano) / Math.pow(10.0, 9)));
                previousVelocity = currentVelocity;
                previousTimeNano = System.nanoTime();

                double average = 0;
                for (Double acceleration : accelerations)
                    average += acceleration;

                average /= accelerations.size();

                telemetry.addData("strafe zero power acceleration (deceleration):", average);
                telemetry.update();
            }
        }
    }
}
