package com.shprobotics.pestocore.tuners;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.drivebases.trackers.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.trackers.NotEnoughOdometryTracker;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

import org.ejml.simple.SimpleMatrix;

import java.util.ArrayList;

@Config
@TeleOp(name = "NEO Translation Tuner")
public class NEOTranslationTuner extends LinearOpMode {
    public static double DISTANCE = 40.0;

    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        DeterministicTracker tracker = FrontalLobe.tracker;
        assert tracker instanceof NotEnoughOdometryTracker : "Tracker must be a NEO Tracker to run a NEO Tuner";

        int n = ((NotEnoughOdometryTracker) tracker).odometryPods.length;

        telemetry.addLine("1. Press a to start recording");
        telemetry.addLine("2. Push the robot " + DISTANCE + " inches forward");
        telemetry.addLine("3. Press b to stop recording");
        telemetry.addLine("4. Repeat to get X,Y parameters");
        telemetry.update();

        ArrayList<double[]> T_arr = new ArrayList<>();

        double[] column1 = new double[n];
        double[] column2 = new double[n];
        double[] column3 = new double[n];

        for (int i = 0; i < n; i++) {
            column1[i] = 0.0;
            column2[i] = 0.0;

            column3[i] = ((NotEnoughOdometryTracker) tracker).ODOMETRY_PARAMETERS_R.get(0, i);
        }

        waitForStart();

        tracker.reset();
        while (opModeIsActive() && !isStopRequested()) {
            while (opModeIsActive() && !isStopRequested() && !gamepad1.a) {
                telemetry.addLine("1. Press a to start recording");
                telemetry.addLine("2. Push the robot " + DISTANCE + " inches forward");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get X,Y parameters");
                telemetry.addLine();
                telemetry.addLine("Waiting for A");
                telemetry.addLine();
                telemetry.addLine("X Parameters");
                for (int i = 0; i < n; i++) {
                    telemetry.addLine(String.valueOf(column1[i]));
                }
                telemetry.addLine();
                telemetry.addLine("Y Parameters");
                for (int i = 0; i < n; i++) {
                    telemetry.addLine(String.valueOf(column2[i]));
                }
                telemetry.update();
            }

            double[] X_constants = new double[n*2];
            double[] Y_constants = new double[n*2];

            for (int i = 0; i < n*2; i++) {
                X_constants[i] = 0.0;
                Y_constants[i] = 0.0;
            }

            double X = 0.0;
            double Y = 0.0;
            double R = 0.0;

            while (opModeIsActive() && !isStopRequested() && !gamepad1.b) {
                MotorCortex.update();

                double dx = 0.0;
                double dy = 0.0;

                for (int i = 0; i < n; i++) {
                    double deltaPosition = ((NotEnoughOdometryTracker) tracker).odometryPods[i].getInchesTravelled();

                    X_constants[i*2] += Math.cos(R) * deltaPosition;
                    X_constants[i*2 + 1] -= Math.sin(R) * deltaPosition;

                    Y_constants[i*2] += Math.sin(R) * deltaPosition;
                    Y_constants[i*2 + 1] += Math.cos(R) * deltaPosition;

                    dx += (deltaPosition * column1[i]);
                    dy += (deltaPosition * column2[i]);
                    R += (deltaPosition * column3[i]);
                }

                // Just for tracking
                X += (Math.cos(R) * dx) - (Math.sin(R) * dy);
                Y += (Math.cos(R) * dy) + (Math.sin(R) * dx);

                telemetry.addLine("1. Recording!");
                telemetry.addData("X", X);
                telemetry.addData("Y", Y);
                telemetry.addData("R", R);
                telemetry.addLine();
                telemetry.addLine("2. Push the robot " + DISTANCE + " inches forward");
                telemetry.addLine("3. Press b to stop recording");
                telemetry.addLine("4. Repeat to get X,Y parameters");
                telemetry.update();
            }

            T_arr.add(X_constants);
            T_arr.add(Y_constants);
            SimpleMatrix T_matrix = new SimpleMatrix(T_arr.size(), n*2);

            double[] translation_arr = new double[T_arr.size()];

            for (int i = 0; i < T_arr.size(); i++) {
                // X
                if (i % 2 == 0)
                    translation_arr[i] = 0.0;
                // Y
                else
                    translation_arr[i] = DISTANCE;
            }

            SimpleMatrix translations = new SimpleMatrix(T_arr.size(), 1);
            translations.setColumn(0, new SimpleMatrix(translation_arr));

            for (int i = 0; i < T_arr.size(); i++) {
                T_matrix.setRow(i, new SimpleMatrix(T_arr.get(i)));
            }

            SimpleMatrix translational_parameters = T_matrix.pseudoInverse().mult(translations);

            for (int i = 0; i < n*2; i++) {
                if (i % 2 == 0)
                    column1[i / 2] = translational_parameters.get(i, 0);
                else
                    column2[(i-1) / 2] = translational_parameters.get(i, 0);
            }
        }
    }
};