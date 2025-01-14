package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;

@TeleOp(name = "pivot test", group = "grup")
public class PivotTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().reset();

        Arm arm = new Arm(hardwareMap);
        arm.intakeOverSubmersible();

        Pivot pivot = new Pivot(hardwareMap);


        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            if (gamepad1.triangle) {
                pivot.resetAngleVertical();
            }
            if (gamepad1.circle) {
                pivot.resetAngleHorizontal();

            }

            telemetry.addData("angle", pivot.getAngle());
            telemetry.addData("target", Pivot.targetAngle);
            telemetry.update();
        }
    }
}