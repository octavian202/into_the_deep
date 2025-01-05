package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;

import java.util.List;

@Config
@TeleOp(name = "arm test", group = "grup")
public class ArmTest extends LinearOpMode {

    public static double ARM = 0d, WRIST = 0d;
    public static double GRIPPER = 0, ROLL = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Arm arm = new Arm(hardwareMap);
        Gripper gripper = new Gripper(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            arm.set(ARM, WRIST);
            gripper.set(GRIPPER, ROLL);
        }
    }
}
