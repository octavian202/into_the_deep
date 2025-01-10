package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;

@Config
@TeleOp(name = "test extensie", group = "test")
public class ExtensionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        CommandScheduler.getInstance().reset();

        Motor left = new Motor(hardwareMap, "est", Motor.GoBILDA.RPM_1150);
        Motor right = new Motor(hardwareMap, "edr", Motor.GoBILDA.RPM_1150);

        left.setRunMode(Motor.RunMode.RawPower);
        right.setRunMode(Motor.RunMode.RawPower);

        left.setInverted(true);

        Motor.Encoder extensionEncoder = right.encoder;
        extensionEncoder.reset();

        Pivot pivot = new Pivot(hardwareMap);


        Arm arm = new Arm(hardwareMap);
        //arm.intakeSpecimen();

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            double power = gamepad2.right_trigger - gamepad2.left_trigger;
            left.set(power);
            right.set(power);

            pivot.set(-gamepad2.right_stick_y);

            telemetry.addData("extension", extensionEncoder.getPosition());
            telemetry.addData("pivot angle", pivot.getAngle());
            telemetry.addData("target", Pivot.targetAngle);
            telemetry.addData("power", -gamepad2.right_stick_y);
            telemetry.update();
        }
    }
}
