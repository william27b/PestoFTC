package com.shprobotics.pestocore.drivebases.trackers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.shprobotics.pestocore.geometries.Circle;
import com.shprobotics.pestocore.geometries.Pose;

import org.ejml.simple.SimpleMatrix;

public class NotEnoughOdometryTracker implements DeterministicTracker {
    public final SimpleMatrix ODOMETRY_PARAMETERS_X;
    public final SimpleMatrix ODOMETRY_PARAMETERS_Y;
    public final SimpleMatrix ODOMETRY_PARAMETERS_R;

    public final Odometry[] odometryPods;
    private final int n;

    private Pose robotVelocity;
    private Pose positionMinus2;
    private Pose positionMinus1;
    private Pose deltaPosition;
    private Pose currentPosition;

    private final ElapsedTime elapsedTime;
    private double lastTime;

    public NotEnoughOdometryTracker(TrackerBuilder trackerBuilder) {
        this.ODOMETRY_PARAMETERS_X = trackerBuilder.ODOMETRY_PARAMETERS_X;
        this.ODOMETRY_PARAMETERS_Y = trackerBuilder.ODOMETRY_PARAMETERS_Y;
        this.ODOMETRY_PARAMETERS_R = trackerBuilder.ODOMETRY_PARAMETERS_R;

        this.odometryPods = trackerBuilder.odometryPods;
        this.n = this.odometryPods.length;

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
        double[] arr_inputs = new double[n];

        for (int i = 0; i < n; i++) {
            arr_inputs[i] = this.odometryPods[i].getInchesTravelled();
        }

        SimpleMatrix inputs = new SimpleMatrix(new double[][]{
                arr_inputs
        });

        double x = inputs.mult(ODOMETRY_PARAMETERS_X).toArray2()[0][0];
        double y = inputs.mult(ODOMETRY_PARAMETERS_Y).toArray2()[0][0];
        double r = inputs.mult(ODOMETRY_PARAMETERS_R).toArray2()[0][0];

        double deltaTime = this.elapsedTime.seconds() - this.lastTime;
        this.lastTime = this.elapsedTime.seconds();
        this.robotVelocity = Pose.multiply(new Pose(x, y, r), 1/deltaTime);

        double headingRadians = currentPosition.getHeadingRadians();

        double xOriented = (Math.cos(headingRadians) * x) - (Math.sin(headingRadians) * y);
        double yOriented = (Math.cos(headingRadians) * y) + (Math.sin(headingRadians) * x);

        this.positionMinus2 = this.positionMinus1;
        this.positionMinus1 = this.currentPosition;

        this.deltaPosition = new Pose(
                xOriented,
                yOriented,
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

        private final Odometry[] odometryPods;

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

                String[] odometryNames,
                DcMotorSimple.Direction[] odometryDirections
        ) {
            this.ODOMETRY_PARAMETERS_X = ODOMETRY_PARAMETERS_X;
            this.ODOMETRY_PARAMETERS_Y = ODOMETRY_PARAMETERS_Y;
            this.ODOMETRY_PARAMETERS_R = ODOMETRY_PARAMETERS_R;

            int n = odometryNames.length;

            assert odometryDirections.length == n : "# of Odometry Names must == # of Odometry Directions";

            assert ODOMETRY_PARAMETERS_X.getNumRows() == 1 : "Odometry Parameters X must have 1 row";
            assert ODOMETRY_PARAMETERS_X.getNumCols() == n : "Odometry Parameters X # of columns must match # of odometry pods";

            assert ODOMETRY_PARAMETERS_Y.getNumRows() == 1 : "Odometry Parameters Y must have 1 row";
            assert ODOMETRY_PARAMETERS_Y.getNumCols() == n : "Odometry Parameters Y # of columns must match # of odometry pods";

            assert ODOMETRY_PARAMETERS_R.getNumRows() == 1 : "Odometry Parameters R must have 1 row";
            assert ODOMETRY_PARAMETERS_R.getNumCols() == n : "Odometry Parameters R # of columns must match # of odometry pods";

            odometryPods = new Odometry[n];

            for (int i = 0; i < odometryNames.length; i++) {
                String name = odometryNames[i];
                DcMotorSimple.Direction direction = odometryDirections[i];

                Odometry odometry = new Odometry(
                        (DcMotor)hardwareMap.get(name),
                        ODOMETRY_TICKS_PER_INCH
                );

                odometry.setDirection(direction);
                odometry.reset();

                odometryPods[i] = odometry;
            }

            this.robotVelocity = new Pose(0, 0, 0);
            this.positionMinus2 = new Pose(0, 0, 0);
            this.positionMinus1 = new Pose(0, 0, 0);
            this.currentPosition = new Pose(0, 0, 0);

            this.elapsedTime = new ElapsedTime();
            this.lastTime = elapsedTime.seconds();
        }

        public NotEnoughOdometryTracker build() {
            return new NotEnoughOdometryTracker(this);
        }
    }
}