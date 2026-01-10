package com.shprobotics.pestocore.vision;

import com.shprobotics.pestocore.geometries.Pose;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

public class ComputerVision {
    public static Mat convertColor(Mat input, int code) {
        Mat out = new Mat();
        Imgproc.cvtColor(input, out, code);
        return out;
    }

    public static Mat filterColor(Mat input, Scalar low, Scalar high) {
        Mat out = new Mat();
        Core.inRange(input, low, high, out);
        return out;
    }

    public static List<MatOfPoint> findContours(Mat input) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(input, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        hierarchy.release();
        return contours;
    }

    public static List<MatOfPoint> findContours(Mat input, int mode, int method) {
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(input, contours, hierarchy, mode, method);
        hierarchy.release();
        return contours;
    }

    public static Mat blur(Mat input, Size size) {
        Mat output = new Mat();
        Imgproc.blur(input, output, size);
        return output;
    }

    public static Pose getPose(Mat input) {
        Moments moments = Imgproc.moments(input);

        double x = moments.get_m10() / moments.get_m00();
        double y = moments.get_m01() / moments.get_m00();

        double a = moments.get_m20() / moments.get_m00() - x * x;
        double b = 2 * (moments.get_m11() / moments.get_m00() - x * y);
        double c = moments.get_m02() / moments.get_m00() - y * y;

        double theta = 0.5 * Math.atan(b / (a - c)) + (a < c ? Math.PI / 2 : 0);

//        % Barycenter
//        E.x = E.m10/E.m00;
//        E.y = E.m01/E.m00;
//
//        % Central moments (intermediary step)
//        a = E.m20/E.m00 - E.x^2;
//        b = 2*(E.m11/E.m00 - E.x*E.y);
//        c = E.m02/E.m00 - E.y^2;
//
//        % Orientation (radians)
//        E.theta = 1/2*atan(b/(a-c)) + (a<c)*pi/2;

        return new Pose(x, y, theta);
    }
}