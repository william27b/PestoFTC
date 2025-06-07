package com.shprobotics.pestocore.tuners;

import com.acmerobotics.dashboard.config.variable.DataItem;
import com.acmerobotics.dashboard.config.variable.NumericalSelector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.shprobotics.pestocore.drivebases.controllers.DriveController;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.PestoTelemetry;

import java.util.ArrayList;

@TeleOp(name = "Forward Deceleration Tuner", group = "Pesto Tuners")
public class ForwardDecelerationTuner extends LinearOpMode {
    private ArrayList<Double> accelerations = new ArrayList<>();

    protected DeterministicTracker tracker;
    protected DriveController driveController;

    public static double targetVelocity = 30.0;
    private boolean stopping;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);
        PestoTelemetry pestoTelemetry = FrontalLobe.pestoTelemetry;

        NumericalSelector targetVelocitySelector = new NumericalSelector("targetVelocity", 30.0, 0.0, 60.0);
        targetVelocitySelector.setColor(DataItem.MessageColor.INFO);
        targetVelocitySelector.setUnit("inches / s");

        pestoTelemetry.addToDash(targetVelocitySelector);
        pestoTelemetry.update();

        driveController = FrontalLobe.driveController;
        tracker = FrontalLobe.tracker;

        driveController.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("The robot will run forward until it reaches " + targetVelocity + " inches per second.");
        telemetry.addLine("Then, it will cut power from the drivetrain and roll to a stop.");
        telemetry.addLine("Make sure you have enough room.");
        telemetry.addLine("After stopping, the forward zero power acceleration (natural deceleration) will be displayed.");
        telemetry.addLine("Press CROSS or A on game pad 1 to stop.");
        telemetry.update();

        waitForStart();

        double previousVelocity = 0.0;
        double previousTimeNano = System.nanoTime();

        driveController.drive(1, 0, 0);

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

                telemetry.addData("forward zero power acceleration (deceleration):", average);
                telemetry.update();
            }
        }
    }
}
