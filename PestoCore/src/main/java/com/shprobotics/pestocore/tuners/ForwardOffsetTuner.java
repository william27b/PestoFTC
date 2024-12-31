package com.shprobotics.pestocore.tuners;

import static java.lang.Math.PI;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.shprobotics.pestocore.drivebases.DeterministicTracker;
import com.shprobotics.pestocore.drivebases.DriveController;
import com.shprobotics.pestocore.drivebases.MecanumController;
import com.shprobotics.pestocore.geometries.BezierCurve;
import com.shprobotics.pestocore.geometries.ParametricHeading;
import com.shprobotics.pestocore.geometries.PathContainer;
import com.shprobotics.pestocore.geometries.PathFollower;
import com.shprobotics.pestocore.geometries.Vector2D;

public abstract class ForwardOffsetTuner extends LinearOpMode {
    public MecanumController mecanumController;
    public DeterministicTracker tracker;
    public PathFollower pathFollower;

    public abstract void setMecanumController(HardwareMap hardwareMap);
    public abstract void setTracker(HardwareMap hardwareMap);
    public abstract void setPathFollower(DriveController driveController, DeterministicTracker tracker, PathContainer pathContainer);

    @Override
    public void runOpMode() {
        setMecanumController(hardwareMap);
        setTracker(hardwareMap);

        PathContainer pathContainer = new PathContainer.PathContainerBuilder()
                .addCurve(new BezierCurve(new Vector2D[]{
                        new Vector2D(0, 0),
                        new Vector2D(0, 0)
                }),
                        new ParametricHeading(new double[]{
                                0,
                                1 * PI / 2,
                                1 * PI,
                                3 * PI / 2,
                                2 * PI
                        }))
                .build();

        setPathFollower(mecanumController, tracker, pathContainer);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            tracker.update();
            pathFollower.update();
        }
    }
};