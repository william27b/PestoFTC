package com.shprobotics.pestocore.tuners;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.drivebases.ThreeWheelOdometryTracker;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;

public abstract class ForwardOffsetTest extends LinearOpMode {
    public MecanumController mecanumController;
    public ThreeWheelOdometryTracker tracker;
    public PathFollower pathFollower;

    public abstract void setMecanumController(HardwareMap hardwareMap);
    public abstract void setTracker(HardwareMap hardwareMap);
    public abstract void setPathFollower(DriveController driveController, DeterministicTracker tracker, PathContainer pathContainer);

    @Override
    public void runOpMode() {
        setMecanumController(hardwareMap);
        setTracker(hardwareMap);

        PathContainer pathContainer = new PathContainer.PathContainerBuilder()
                .addCurve(new ParametricHeading(new double[]{
                        0,
                        PI / 2,
                        PI,
                        3 * PI / 2,
                        2 * PI
                }))
                .setIncrement(0.01)
                .build();

        setPathFollower(mecanumController, tracker, pathContainer);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            pathFollower.update();
            tracker.update();

            telemetry.addData("x", tracker.getCurrentPosition().getX());
            telemetry.addData("y", tracker.getCurrentPosition().getY());
            telemetry.addData("r", tracker.getCurrentPosition().getHeadingRadians());
            telemetry.update();
        }
    }
};