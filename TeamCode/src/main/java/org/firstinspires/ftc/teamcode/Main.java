package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.AutoArmControl;
import org.firstinspires.ftc.teamcode.commands.DriveRobotCentric;
import org.firstinspires.ftc.teamcode.commands.GoToDefaultPosition;
import org.firstinspires.ftc.teamcode.commands.GripperRoll;
import org.firstinspires.ftc.teamcode.commands.ManualExtension;
import org.firstinspires.ftc.teamcode.commands.ManualPivot;
import org.firstinspires.ftc.teamcode.commands.PickUpSample;
import org.firstinspires.ftc.teamcode.commands.PickUpSpecimen;
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
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(pivot::resetAngleVertical);
        gp1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(pivot::resetAngleHorizontal);
        pivot.resetAngleVertical();

        gp2.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                new InstantCommand(pivot::goDown, pivot),
                new InstantCommand(pivot::goUp, pivot),
                () -> pivot.getAngle() >= 45
        ));
        pivot.setDefaultCommand(new ManualPivot(pivot, gp2::getRightY));

        Extension extension = new Extension(hardwareMap);
        extension.setDefaultCommand(new ManualExtension(extension, () -> gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gp2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new InstantCommand(extension::goHighBasket));
        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new InstantCommand(extension::goHighRung, extension));


        Gripper gripper = new Gripper(hardwareMap);
        gp2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                new InstantCommand(gripper::close),
                new InstantCommand(gripper::open),
                () -> gripper.isOpen
        ));
//        gp1.getGamepadButton(GamepadKeys.Button.X).whenHeld(new InstantCommand(gripper::aliniat, gripper));
//        gp1.getGamepadButton(GamepadKeys.Button.X).whenReleased(new InstantCommand(gripper::turnDefault, gripper));

//        gripper.setDefaultCommand(new GripperRoll(gripper, gp2::getLeftX));


        Arm arm = new Arm(hardwareMap);
        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);

        pivotIsDown.whenActive(new GripperRoll(gripper, gp2::getLeftX));

        pivotIsUp.whenActive(new AutoArmControl(arm, gripper, extension));

        pivotIsDown.whenActive(new InstantCommand(() -> gripper.turn(0), gripper));
        pivotIsDown.whenActive(new InstantCommand(arm::intakeSpecimen, arm));

        gp2.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(pivotIsDown).whenActive(new InstantCommand(arm::intakeSpecimen, arm));
        gp2.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(pivotIsDown).whenActive(new InstantCommand(arm::intakeOverSubmersible));
        gp2.getGamepadButton(GamepadKeys.Button.X).and(pivotIsDown).whenActive(new PickUpSample(arm, gripper));

//        gp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenReleased(new InstantCommand(gripper::aliniat, gripper));
//        gp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(gripper::turnDefault, gripper));

        (new AutoArmControl(arm, gripper, extension)).schedule();

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("pos", extension.getPosition());
            telemetry.addData("angle", pivot.getAngle());
            telemetry.addData("angle target", Pivot.targetAngle);
            telemetry.addData("y", gp2.getRightY());
            telemetry.update();
        }
    }
}
