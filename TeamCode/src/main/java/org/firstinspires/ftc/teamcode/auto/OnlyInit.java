package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.GoToDefaultPosition;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;

@Autonomous(name = "auto", group = ".")
public class OnlyInit extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }


        CommandScheduler.getInstance().reset();

        Pivot pivot = new Pivot(hardwareMap);
        pivot.setAngledStart();

        Arm arm = new Arm(hardwareMap);
        arm.initInDimensions();

        GoToDefaultPosition defaultCommand = new GoToDefaultPosition(pivot);

        while (opModeInInit() && !isStopRequested()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("target", Pivot.targetAngle);
            telemetry.addData("angle", Pivot.angle);
            telemetry.update();
        }

        waitForStart();

        defaultCommand.schedule();

        while (opModeIsActive()) {

            CommandScheduler.getInstance().run();

        }

    }
}
