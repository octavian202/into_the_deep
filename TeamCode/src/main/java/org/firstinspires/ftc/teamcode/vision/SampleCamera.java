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
    public static double COOLDOWN = 0.05;

    SampleDetection sampleDetection;
    VisionPortal visionPortal;

    private Point center = new Point();
    private double orientation = 0d;

    Timer timer = new Timer();
    public boolean reading = false;

    public SampleCamera(HardwareMap hardwareMap) {

        timer.resetTimer();

        sampleDetection = new SampleDetection();
        visionPortal = new VisionPortal.Builder()
                .addProcessor(sampleDetection)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        resumeReading();
//        reading = true;
    }

    @Override
    public void periodic() {
        if (reading) {
            timer.resetTimer();
            orientation = sampleDetection.getDetectedAngle();
            center = sampleDetection.getSampleCenter();
        }
    }

    public void clearData() {
        orientation = 0d;
        center = new Point(0d, 0d);
    }

    public void resumeReading() {
        reading = true;
    }

    public void stopReading() {
        reading = false;
    }

    public double getOrientation() {
        return orientation;
    }

    public Point getPosition() {
        return center;
    }

}
