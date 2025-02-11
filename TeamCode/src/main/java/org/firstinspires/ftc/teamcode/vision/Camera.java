package org.firstinspires.ftc.teamcode.vision;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;

public class Camera extends SubsystemBase {

    ColorBlobLocatorProcessor colorBlobLocatorProcessor;
    VisionPortal visionPortal;

    List<ColorBlobLocatorProcessor.Blob> blobs;

    public Camera(HardwareMap hardwareMap, String color) {
        ColorRange colorRange;

        switch (color) {
            case "red":
                colorRange = ColorRange.RED;
                break;
            case "blue":
                colorRange = ColorRange.BLUE;
                break;
            default:
                colorRange = ColorRange.YELLOW;
                break;
        }

        colorBlobLocatorProcessor = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(colorRange)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBlurSize(10)
                .build();

        visionPortal = new VisionPortal.Builder()
                .addProcessor(colorBlobLocatorProcessor)
                .setCameraResolution(new Size(1280, 720))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

    }

    @Override
    public void periodic() {
        blobs = colorBlobLocatorProcessor.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 900000, blobs);
    }

    public List<ColorBlobLocatorProcessor.Blob> getBlobs() {
        return blobs;
    }

}
