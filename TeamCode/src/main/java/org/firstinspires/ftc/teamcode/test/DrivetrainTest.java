package org.firstinspires.ftc.teamcode.test;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@TeleOp(name = "drivetrain test", group = "test")
public class DrivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        CommandScheduler.getInstance().reset();

        GamepadEx gp1 = new GamepadEx(gamepad1);
        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setDefaultCommand(new DriveRobotCentric(drivetrain, gp1::getLeftX, gp1::getLeftY, gp1::getRightX));

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
        }
    }
}
