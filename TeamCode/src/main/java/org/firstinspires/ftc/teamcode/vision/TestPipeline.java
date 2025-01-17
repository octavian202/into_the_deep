package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class TestPipeline extends OpenCvPipeline {
    public static Scalar lowerBound = new Scalar(80, 50, 30);
    public static Scalar upperBound = new Scalar(140, 255, 255);
    public static Scalar strictLowerBound = new Scalar(100, 60, 40);
    public static Scalar strictUpperBound = new Scalar(130, 255, 255);

    @Override
    public Mat processFrame(Mat input) {

        Mat hsv = new Mat();
        Mat mask = new Mat();
        Mat stricterMask = new Mat();
        Mat thresholded = new Mat();
        Mat output = new Mat();
        Mat stricterThresholded = new Mat();
        Mat edges = new Mat();

        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Core.inRange(hsv, lowerBound, upperBound, mask);
        Core.bitwise_and(hsv, hsv, thresholded, mask);

        Core.inRange(thresholded, strictLowerBound, strictUpperBound, stricterMask);
        Core.bitwise_and(thresholded, thresholded, stricterThresholded, stricterMask);

        Imgproc.Canny(stricterThresholded, edges, 100, 200);

//        Imgproc.cvtColor(edges, output, Imgproc.COLOR_HSV2RGB);
        edges.copyTo(input);

        hsv.release();
        mask.release();
        stricterMask.release();
        thresholded.release();
        output.release();
        stricterThresholded.release();
        edges.release();

        return input;
    }
}
