package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.trackers.NotEnoughOdometryTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

@TeleOp(name = "NEO Rotation Tuner")
public class NEORotationTuner extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        DeterministicTracker tracker = FrontalLobe.tracker;
        assert tracker instanceof NotEnoughOdometryTracker : "Tracker must be a NEO Tracker to run a NEO Tuner";

        int n = ((NotEnoughOdometryTracker) tracker).odometryPods.length;

        telemetry.addLine("1. Press a to start recording");
        telemetry.addLine("2. Rotate the robot 180 degrees");
        telemetry.addLine("3. Press b to stop recording");
        telemetry.addLine("4. Repeat to get R parameters");
        telemetry.update();

        ArrayList<double[]> T_arr = new ArrayList<>();

        double[] column3 = new double[n];

        for (int i = 0; i < n; i++) {
            column3[i] = 0.0;
        }

        waitForStart();

        tracker.reset();
        while (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive() && !isStopRequested() && !gamepad1.a) {
                telemetry.addLine("1. Press a to start recording");
                telemetry.addLine("2. Rotate the robot 180 degrees");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get R parameters");
                telemetry.addLine();
                telemetry.addLine("Waiting for A");
                telemetry.addLine();
                telemetry.addLine("R Parameters");
                for (int i = 0; i < n; i++) {
                    telemetry.addLine(String.valueOf(column3[i]));
                }
                telemetry.update();
            }

            double[] Td = new double[n];

            for (int i = 0; i < n; i++) {
                Td[i] = 0.0;
            }

            double R = 0.0;

            while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
                MotorCortex.update();

                for (int i = 0; i < n; i++) {
                    double deltaPosition = ((NotEnoughOdometryTracker) tracker).odometryPods[i].getInchesTravelled();
                    Td[i] += deltaPosition;
                    R += (deltaPosition * column3[i]);
                }

                telemetry.addLine("1. Recording!");
                telemetry.addData("R", R);
                telemetry.addLine();
                telemetry.addLine("2. Rotate the robot 180 degrees");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get R parameters");
                telemetry.update();
            }

            T_arr.add(Td);
            SimpleMatrix T_matrix = new SimpleMatrix(T_arr.size(), 3);

            SimpleMatrix rotations = new SimpleMatrix(T_arr.size(), 1);
            rotations.fill(Math.PI);

            for (int i = 0; i < T_arr.size(); i++) {
                T_matrix.setRow(i, new SimpleMatrix(T_arr.get(i)));
            }

            SimpleMatrix rotation_parameters = T_matrix.pseudoInverse().mult(rotations);

            for (int i = 0; i < n; i++) {
                column3[i] = rotation_parameters.get(i, 0);
            }
        }
    }
};