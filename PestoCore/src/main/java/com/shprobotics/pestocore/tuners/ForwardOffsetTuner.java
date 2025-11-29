package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Forward Offset Tuner", group = "Pesto Tuners")
public class ForwardOffsetTuner extends LinearOpMode {
    public DeterministicTracker tracker;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        tracker = FrontalLobe.tracker;

        telemetry.addLine("1. Rotate the robot 180 degrees");
        telemetry.addLine("2. Subtract axis movement from actual movement");
        telemetry.addLine("3. Divide by PI");
        telemetry.addLine("4. Add to FORWARD_OFFSET");
        telemetry.update();

        waitForStart();

        tracker.reset();
        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();

            telemetry.addLine("1. Rotate the robot 180 degrees");
            telemetry.addLine("2. Subtract axis movement from actual movement");
            telemetry.addLine("3. Divide by PI");
            telemetry.addLine("4. Add to FORWARD_OFFSET");

            telemetry.addData("X", tracker.getCurrentPosition().getX());
            telemetry.addData("Y", tracker.getCurrentPosition().getY());
            telemetry.addData("R", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
};