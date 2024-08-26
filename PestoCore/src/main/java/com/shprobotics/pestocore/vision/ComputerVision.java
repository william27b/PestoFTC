package com.shprobotics.pestocore.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class ComputerVision {
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
}