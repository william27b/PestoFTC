package com.shprobotics.pestocore.drivebases.trackers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.geometries.Circle;
import com.shprobotics.pestocore.geometries.Pose;

import org.ejml.simple.SimpleMatrix;

public class TWOT implements DeterministicTracker {
    public final SimpleMatrix ODOMETRY_PARAMETERS_X;
    public final SimpleMatrix ODOMETRY_PARAMETERS_Y;
    public final SimpleMatrix ODOMETRY_PARAMETERS_R;

    public final Odometry leftOdometry;
    public final Odometry rightOdometry;
    public final Odometry centerOdometry;

    private Pose robotVelocity;
    private Pose positionMinus2;
    private Pose positionMinus1;
    private Pose deltaPosition;
    private Pose currentPosition;

    private final ElapsedTime elapsedTime;
    private double lastTime;

    public TWOT(TrackerBuilder trackerBuilder) {
        this.ODOMETRY_PARAMETERS_X = trackerBuilder.ODOMETRY_PARAMETERS_X;
        this.ODOMETRY_PARAMETERS_Y = trackerBuilder.ODOMETRY_PARAMETERS_Y;
        this.ODOMETRY_PARAMETERS_R = trackerBuilder.ODOMETRY_PARAMETERS_R;

        this.leftOdometry = trackerBuilder.leftOdometry;
        this.rightOdometry = trackerBuilder.rightOdometry;
        this.centerOdometry = trackerBuilder.centerOdometry;

        this.robotVelocity = trackerBuilder.robotVelocity;
        this.positionMinus2 = trackerBuilder.positionMinus2;
        this.positionMinus1 = trackerBuilder.positionMinus1;
        this.currentPosition = trackerBuilder.currentPosition;

        this.elapsedTime = trackerBuilder.elapsedTime;
        this.lastTime = trackerBuilder.lastTime;
    }

    public void reset() {
        reset(0.0);
    }

    public void reset(double heading) {
        this.robotVelocity = new Pose(0, 0, 0);
        this.positionMinus2 = new Pose(0, 0, 0);
        this.positionMinus1 = new Pose(0, 0, 0);
        this.deltaPosition = new Pose(0, 0, 0);
        this.currentPosition = new Pose(0, 0, heading);
    }

    public void reset(Pose position) {
        this.robotVelocity = new Pose(0, 0, 0);
        this.positionMinus2 = new Pose(0, 0, 0);
        this.positionMinus1 = new Pose(0, 0, 0);
        this.deltaPosition = new Pose(0, 0, 0);
        this.currentPosition = position;
    }

    public void resetHeading(double heading) {
        this.currentPosition = new Pose(
                this.currentPosition.getX(),
                this.currentPosition.getY(),
                heading
        );
    }

    public void resetTime() {
        this.lastTime = this.elapsedTime.seconds();
    }

    public void update() {
        double dL = this.leftOdometry.getInchesTravelled();
        double dC = this.centerOdometry.getInchesTravelled();
        double dR = this.rightOdometry.getInchesTravelled();

        SimpleMatrix r_inputs = new SimpleMatrix(new double[][]{
                new double[]{dL, dC, dR}
        });

        double r = r_inputs.mult(ODOMETRY_PARAMETERS_R).toArray2()[0][0];

        double heading = currentPosition.getHeadingRadians();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        SimpleMatrix xy_inputs = new SimpleMatrix(new double[][]{
                new double[]{
                        dL * cos,
                        dL * sin,
                        dC * cos,
                        dC * sin,
                        dR * cos,
                        dR * sin
                }
        });

        double x = xy_inputs.mult(ODOMETRY_PARAMETERS_X).toArray2()[0][0];
        double y = xy_inputs.mult(ODOMETRY_PARAMETERS_Y).toArray2()[0][0];

        double deltaTime = this.elapsedTime.seconds() - this.lastTime;
        this.lastTime = this.elapsedTime.seconds();
        this.robotVelocity = Pose.multiply(new Pose(x, y, r), 1/deltaTime);

        this.positionMinus2 = this.positionMinus1;
        this.positionMinus1 = this.currentPosition;

        this.deltaPosition = new Pose(
                x,
                y,
                r
        );

        this.currentPosition.add(
                this.deltaPosition
        );
    }


    public Pose getCurrentPosition() {
        return this.currentPosition;
    }

    public Pose getRobotVelocity() {
        return this.robotVelocity;
    }

    public Pose getDeltaPosition() {
        return deltaPosition;
    }

    public double getCentripetalRadius() {
        return Circle.getRadius(this.positionMinus2.asVector(), this.positionMinus1.asVector(), this.currentPosition.asVector());
    }

    public Pose getCentripetalForce() {
        double magnitude = this.robotVelocity.getMagnitude();
        double scalar = magnitude / getCentripetalRadius(); // we don't need magnitude * magnitude; we multiply by robotVelocity after
        return Pose.scale(Pose.perpendicular(this.robotVelocity.asVector()), scalar);
    }

    public static class TrackerBuilder implements Tracker.TrackerBuilder {
        private final SimpleMatrix ODOMETRY_PARAMETERS_X;
        private final SimpleMatrix ODOMETRY_PARAMETERS_Y;
        private final SimpleMatrix ODOMETRY_PARAMETERS_R;

        private final Odometry leftOdometry;
        private final Odometry centerOdometry;
        private final Odometry rightOdometry;

        private final Pose robotVelocity;
        private final Pose positionMinus2;
        private final Pose positionMinus1;
        private final Pose currentPosition;
        private final ElapsedTime elapsedTime;
        private final double lastTime;

        public TrackerBuilder(
                HardwareMap hardwareMap,

                double ODOMETRY_TICKS_PER_INCH,
                SimpleMatrix ODOMETRY_PARAMETERS_X,
                SimpleMatrix ODOMETRY_PARAMETERS_Y,
                SimpleMatrix ODOMETRY_PARAMETERS_R,

                String leftName,
                String centerName,
                String rightName,

                DcMotorSimple.Direction leftDirection,
                DcMotorSimple.Direction centerDirection,
                DcMotorSimple.Direction rightDirection
        ) {
            this.ODOMETRY_PARAMETERS_X = ODOMETRY_PARAMETERS_X;
            this.ODOMETRY_PARAMETERS_Y = ODOMETRY_PARAMETERS_Y;
            this.ODOMETRY_PARAMETERS_R = ODOMETRY_PARAMETERS_R;

            this.leftOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(leftName),
                    ODOMETRY_TICKS_PER_INCH);

            this.leftOdometry.setDirection(leftDirection);

            this.rightOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(rightName),
                    ODOMETRY_TICKS_PER_INCH);

            this.rightOdometry.setDirection(rightDirection);

            this.centerOdometry = new Odometry(
                    (DcMotor)hardwareMap.get(centerName),
                    ODOMETRY_TICKS_PER_INCH);

            this.centerOdometry.setDirection(centerDirection);

            this.leftOdometry.reset();
            this.rightOdometry.reset();
            this.centerOdometry.reset();

            this.robotVelocity = new Pose(0, 0, 0);
            this.positionMinus2 = new Pose(0, 0, 0);
            this.positionMinus1 = new Pose(0, 0, 0);
            this.currentPosition = new Pose(0, 0, 0);

            this.elapsedTime = new ElapsedTime();
            this.lastTime = elapsedTime.seconds();
        }

        public TWOT build() {
            return new TWOT(this);
        }
    }
}