package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.vision.TestPipeline;

@TeleOp(name = "pipeline test", group = "vision")
public class PipelineTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        TestPipeline pipeline = new TestPipeline();

        Camera camera = new Camera(hardwareMap);
        camera.setPipeline(pipeline);

        waitForStart();

        while (opModeIsActive()) {
        }

        camera.stopStreaming();
    }
}
