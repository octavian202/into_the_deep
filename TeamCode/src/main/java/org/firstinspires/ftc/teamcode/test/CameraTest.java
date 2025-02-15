package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.SampleOrientationDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.core.RotatedRect;

import java.util.List;

@TeleOp(name = "camera test", group = "test")
public class CameraTest extends LinearOpMode {

//    List<ColorBlobLocatorProcessor.Blob> blobs;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleOrientationDetection sampleOrientationDetection = new SampleOrientationDetection();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(sampleOrientationDetection)
                .build();


        CommandScheduler.getInstance().reset();

        waitForStart();

        visionPortal.resumeLiveView();
        visionPortal.resumeStreaming();

//        FtcDashboard.getInstance().startCameraStream(hardwareMap.get(WebcamName.class, "Webcam 1"));

        while (opModeIsActive()) {

            CommandScheduler.getInstance().run();

            telemetry.addData("angle", sampleOrientationDetection.getDetectedAngle());
            telemetry.update();


//            blobs = camera.getBlobs();
//
//            for (ColorBlobLocatorProcessor.Blob b : blobs) {
//                RotatedRect boxFit = b.getBoxFit();
//
//                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
//                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
//            }
//
//            telemetry.update();
        }
    }
}
