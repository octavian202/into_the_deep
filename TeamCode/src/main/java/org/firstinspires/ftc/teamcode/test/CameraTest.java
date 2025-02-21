package org.firstinspires.ftc.teamcode.test;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.vision.SampleCamera;
import org.firstinspires.ftc.teamcode.vision.SampleDetection;
import org.firstinspires.ftc.vision.VisionPortal;

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

        CommandScheduler.getInstance().reset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Arm arm = new Arm(hardwareMap);
        Gripper gripper = new Gripper(hardwareMap);

        arm.intakeSubmersibleAuto();
        gripper.turn(0);

        SampleCamera sampleCamera = new SampleCamera(hardwareMap);
        sampleCamera.resumeReading();

        while (opModeInInit()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("angle", sampleCamera.getOrientation());
            telemetry.addData("pos", sampleCamera.getPosition().toString());
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            CommandScheduler.getInstance().run();

            telemetry.addData("angle", sampleCamera.getOrientation());
            telemetry.addData("pos", sampleCamera.getPosition().toString());
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
