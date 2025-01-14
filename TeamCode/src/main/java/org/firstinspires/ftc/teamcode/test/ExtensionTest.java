package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.ManualPivot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;

@Config
@TeleOp(name = "test extensie", group = "test")
public class ExtensionTest extends LinearOpMode {
    public static int target = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        GamepadEx gp2 = new GamepadEx(gamepad2);

        Pivot pivot = new Pivot(hardwareMap);
        pivot.setDefaultCommand(new ManualPivot(pivot, gp2::getLeftY));

        Arm arm = new Arm(hardwareMap);
        arm.intakeSample();

        Extension extension = new Extension(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            extension.setTarget(target);
            CommandScheduler.getInstance().run();

            telemetry.addData("position", extension.getPosition());
            telemetry.addData("target", extension.getTarget());
            telemetry.update();

        }
    }
}
