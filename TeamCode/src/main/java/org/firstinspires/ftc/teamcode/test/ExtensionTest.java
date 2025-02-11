package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.commands.ManualPivot;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;

@Config
@TeleOp(name = "test extensie", group = "test")
public class ExtensionTest extends LinearOpMode {

    public static double KP = 0.00006, KI = 0d, KD = 0d, KG = 0d;
    public static int TARGET = 0;
    Motor left, right, ascend;
    Motor.Encoder encoder;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        left = new Motor(hardwareMap, "est");
        right = new Motor(hardwareMap, "edr");
        ascend = new Motor(hardwareMap, "ascend");

        left.setRunMode(Motor.RunMode.RawPower);
        right.setRunMode(Motor.RunMode.RawPower);
        ascend.setRunMode(Motor.RunMode.RawPower);

        left.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        ascend.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

        left.set(0d);
        right.set(0d);
        ascend.set(0d);

        left.setInverted(true);

        encoder = right.encoder;
        encoder.reset();

        PIDController pidController = new PIDController(KP, KI, KD);

        waitForStart();

        while (opModeIsActive()) {
            pidController.setPID(KP, KI, KD);

            double output = pidController.calculate(encoder.getPosition(), TARGET) + KG * Math.sin(Math.toRadians(95));
            left.set(output);
            right.set(output);

            telemetry.addData("position", encoder.getPosition());
            telemetry.addData("target", TARGET);
            telemetry.addData("power", output);
            telemetry.update();
        }
    }
}
