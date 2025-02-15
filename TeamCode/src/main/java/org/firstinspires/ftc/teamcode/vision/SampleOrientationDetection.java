package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

@Config
public class SampleOrientationDetection implements VisionProcessor {

    public static Scalar lowerHSV = new Scalar(80, 50, 30); // Adjust these values
    public static Scalar upperHSV = new Scalar(140, 255, 255);
    private double detectedAngle = -1;
    private Mat processedMat = new Mat();
    Mat finalMat = new Mat();
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat kernel = new Mat();
    private Mat hierarchy = new Mat();


    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat input, long captureTimeNanos) {
        try {
            // Convert to HSV color space
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Threshold HSV image
            Core.inRange(hsv, lowerHSV, upperHSV, mask);

            // Morphological operations to reduce noise
            kernel = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            // Find contours
            List<MatOfPoint> contours = new ArrayList<>();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // Find largest contour
            double maxArea = 0;
            MatOfPoint largestContour = null;
            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea && area > 100) { // Filter small contours
                    maxArea = area;
                    largestContour = contour;
                }
            }

            if (largestContour != null) {
                // Fit ellipse to contour
                RotatedRect ellipse = Imgproc.fitEllipse(new MatOfPoint2f(largestContour.toArray()));

                // Draw visualization
                Imgproc.ellipse(input, ellipse, new Scalar(0, 255, 0), 2);
                Point[] vertices = new Point[4];
                ellipse.points(vertices);
                for (int i = 0; i < 4; i++) {
                    Imgproc.line(input, vertices[i], vertices[(i+1)%4], new Scalar(255, 0, 0), 2);
                }

                // Calculate orientation angle (adjusted for OpenCV's coordinate system)
                detectedAngle = (90 + ellipse.angle) % 180;
                Imgproc.putText(input, "Angle: " + String.format("%.2f", detectedAngle),
                        new Point(10, 30), Imgproc.FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(255, 255, 255), 2);
            } else {
                detectedAngle = -1;
            }

            // Return processed frame for viewing
            mask.copyTo(processedMat);
            processedMat.copyTo(input);
            return null;
        } catch (Exception e) {
            return input;
        }
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

    }

    public double getDetectedAngle() {
        return detectedAngle - 90;
    }
}
