package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class TestPipeline extends OpenCvPipeline {
    public static double H_MIN = 0, S_MIN = 50, V_MIN = 50;
    public static double H_MAX = 10, S_MAX = 100, V_MAX = 100;

    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBound = new Scalar(H_MIN, S_MIN, V_MIN);
        Scalar upperBound = new Scalar(H_MAX, S_MAX, V_MAX);

        Core.inRange(hsv, lowerBound, upperBound, thresholded);

        return thresholded;
    }
}
