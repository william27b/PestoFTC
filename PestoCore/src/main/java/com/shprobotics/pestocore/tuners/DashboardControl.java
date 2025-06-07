package com.shprobotics.pestocore.tuners;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.shprobotics.pestocore.processing.FrontalLobe;
import com.shprobotics.pestocore.processing.MotorCortex;

@TeleOp(name = "Dashboard Control", group = "Pesto Tuners")
public class DashboardControl extends LinearOpMode {
    @Override
    public void runOpMode() {
        FrontalLobe.initialize(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            MotorCortex.setMotors();
        }
    }
};