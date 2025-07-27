package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Point;

@Config
public class SampleCamera extends SubsystemBase {
    SampleDetection sampleDetection;
    VisionPortal visionPortal;

    private Point center = new Point();
    private double orientation = 0d;

    Timer timer = new Timer();

    public SampleCamera(HardwareMap hardwareMap) {

        timer.resetTimer();

        sampleDetection = new SampleDetection();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(sampleDetection)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    @Override
    public void periodic() {

        orientation = sampleDetection.getDetectedAngle();

        Point newCenter = sampleDetection.getSampleCenter();
        newCenter.y = Math.max(newCenter.y, -1.0);
        newCenter.y = Math.min(newCenter.y, 1.0);
        newCenter.x = Math.max(newCenter.x, -1.0);
        newCenter.x = Math.min(newCenter.x, 1.0);

        center = new Point(newCenter.x, newCenter.y);
    }

    public void clearData() {
        orientation = 0d;
        center = new Point(0d, 0d);
    }

    public double getOrientation() {
        return orientation;
    }

    public Point getPosition() {
        return center;
    }

}
