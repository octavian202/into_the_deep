package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.TestPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@TeleOp(name = "pipeline test", group = "vision")
public class PipelineTest extends LinearOpMode {
    private OpenCvCamera camera;
    private TestPipeline pipeline;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()
        );
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "pula"), cameraMonitorViewId);

        pipeline = new TestPipeline();
        camera.setPipeline(pipeline);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("Cam error:", "Error code: " + errorCode);
                telemetry.update();
            }
        });

        FtcDashboard.getInstance().startCameraStream(camera, 30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("H_MIN", TestPipeline.H_MIN);
            telemetry.addData("S_MIN", TestPipeline.S_MIN);
            telemetry.addData("V_MIN", TestPipeline.V_MIN);
            telemetry.addData("H_MAX", TestPipeline.H_MAX);
            telemetry.addData("S_MAX", TestPipeline.S_MAX);
            telemetry.addData("V_MAX", TestPipeline.V_MAX);
            telemetry.update();
        }

        camera.stopStreaming();
    }
}
