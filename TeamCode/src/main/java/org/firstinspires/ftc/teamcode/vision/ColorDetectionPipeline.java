package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorDetectionPipeline extends OpenCvPipeline {
    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private List<Rect> detectedRects = new ArrayList<>();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar blueLower = new Scalar(80, 50, 30);
        Scalar blueUpper = new Scalar(140, 255, 255);
        Scalar yellowLower = new Scalar(50, 255, 255);
        Scalar yellowUpper = new Scalar(60, 100, 100);
        Scalar redLower = new Scalar(0, 50, 50);
        Scalar redUpper = new Scalar(10, 100, 100);

        Core.inRange(hsv, blueLower, blueUpper, thresholded);
        Core.inRange(hsv, yellowLower, yellowUpper, thresholded);
        Core.inRange(hsv, redLower, redUpper, thresholded);

        /*List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(thresholded, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        detectedRects.clear();
        for (MatOfPoint contour : contours) {
            Rect rect = Imgproc.boundingRect(contour);
            Imgproc.rectangle(input, rect, new Scalar(0, 255, 0), 2);
            detectedRects.add(rect);
        }*/

        return thresholded;
    }

    public List<Rect> getDetectedRects() {
        return detectedRects;
    }
}
