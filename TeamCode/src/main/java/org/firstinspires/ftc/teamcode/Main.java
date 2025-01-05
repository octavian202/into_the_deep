package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.Supplier;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.commands.ManualExtension;
import org.firstinspires.ftc.teamcode.commands.PickUp;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Extension;
import org.firstinspires.ftc.teamcode.subsystems.Gripper;
import org.firstinspires.ftc.teamcode.subsystems.Pivot;

import java.util.List;

@TeleOp(name = "main", group = ".")
public class Main extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        CommandScheduler.getInstance().reset();

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setDefaultCommand(new DriveRobotCentric(drivetrain, gp1::getLeftX, gp1::getLeftY, gp1::getRightX));

        Pivot pivot = new Pivot(hardwareMap);
        gp1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(pivot::resetAngle);

        Extension extension = new Extension(hardwareMap);
        extension.setDefaultCommand(new ManualExtension(extension, () -> gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER), pivot::getAngle));

        Gripper gripper = new Gripper(hardwareMap);
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                new InstantCommand(gripper::close, gripper),
                new InstantCommand(gripper::open, gripper),
                () -> gripper.isOpen
        ));

        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new InstantCommand(gripper::turnLeft, gripper));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new InstantCommand(gripper::turnRight, gripper));
        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new InstantCommand(gripper::turnDefault, gripper));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new InstantCommand(gripper::turnDefault, gripper));


        Arm arm = new Arm(hardwareMap);
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(arm::outtakeSample, arm));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(arm::outtakeSpecimen, arm));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(arm::intakeSpecimen, arm));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(arm::intakeOverSubmersible));
        gp2.getGamepadButton(GamepadKeys.Button.X).whenPressed(new PickUp(arm, gripper));


        pivot.set(0);

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            pivot.set(gp2.getLeftY());

            telemetry.addData("pos", extension.getPosition());
            telemetry.addData("angle", pivot.getAngle());
            telemetry.update();
        }
    }
}
