package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;

public abstract class ForwardOffsetTuner extends LinearOpMode {
    public DeterministicTracker tracker;

    public abstract void setTracker(HardwareMap hardwareMap);

    @Override
    public void runOpMode() {
        setTracker(hardwareMap);

        telemetry.addLine("1. Rotate the robot 180 degrees");
        telemetry.addLine("2. Subtract axis movement from actual movement");
        telemetry.addLine("3. Divide by PI");
        telemetry.addLine("4. Add to FORWARD_OFFSET");
        telemetry.update();

        waitForStart();

        tracker.reset();
        while (opModeIsActive() && !isStopRequested()) {
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