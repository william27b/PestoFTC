package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.trackers.Odometry;
import com.shprobotics.pestocore.drivebases.trackers.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Forward Offset Tuner", group = "Pesto Tuners")
public class ForwardOffsetTuner extends LinearOpMode {
    public DeterministicTracker tracker;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        tracker = FrontalLobe.tracker;
        assert tracker instanceof ThreeWheelOdometryTracker;

        Odometry left = ((ThreeWheelOdometryTracker) tracker).leftOdometry;
        Odometry right = ((ThreeWheelOdometryTracker) tracker).rightOdometry;
        Odometry center = ((ThreeWheelOdometryTracker) tracker).centerOdometry;

        telemetry.addLine("1. Rotate the robot 360 degrees");
        telemetry.addLine("2. Press b");
        telemetry.update();

        waitForStart();

        double measuredRotationInner = 0.0;
        double measuredRotationOuter = 0.0;

        tracker.reset();
        while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
            MotorCortex.update();

            double dL = left.getInchesTravelled();
            double dC = right.getInchesTravelled();
            double dR = center.getInchesTravelled();

            double distanceRotated = (dL - dR) / 2;
            double x = dC;
            double r = - (2 * distanceRotated) / ((ThreeWheelOdometryTracker) tracker).ODOMETRY_WIDTH;

            measuredRotationInner += r;
            measuredRotationOuter += x;

            telemetry.addLine("1. Rotate the robot 360 degrees");
            telemetry.addLine("2. Press b");
            telemetry.addData("measured rotation inner = ", measuredRotationInner);
            telemetry.addData("measured rotation outer = ", measuredRotationOuter);
            telemetry.update();
        }

        double forward_offset = (((ThreeWheelOdometryTracker) tracker).ODOMETRY_WIDTH / 2) * (measuredRotationOuter / measuredRotationInner);

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.update();
            tracker.update();

            telemetry.addLine("1. Rotate the robot 360 degrees");
            telemetry.addLine("2. Press b");
            telemetry.addData("measured rotation = ", measuredRotationInner);
            telemetry.addData("measured rotation outer = ", measuredRotationOuter);
            telemetry.addData("forward offset = ", forward_offset);
            telemetry.update();
        }
    }
};