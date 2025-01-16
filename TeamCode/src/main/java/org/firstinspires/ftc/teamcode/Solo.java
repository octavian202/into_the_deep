package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.DriveRobotCentric;
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


@TeleOp(name = "solo")
public class Solo extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        CommandScheduler.getInstance().reset();

        GamepadEx gp = new GamepadEx(gamepad1);

        Drivetrain drivetrain = new Drivetrain(hardwareMap);
        drivetrain.setDefaultCommand(new DriveRobotCentric(drivetrain, gp::getLeftX, gp::getLeftY, gp::getRightX));

        Pivot pivot = new Pivot(hardwareMap);
        gp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(pivot::resetAngleVertical);
        gp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(pivot::resetAngleHorizontal);
        pivot.resetAngleVertical();

        gp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                new InstantCommand(pivot::goDown, pivot),
                new InstantCommand(pivot::goUp, pivot),
                () -> pivot.getAngle() >= 45
        ));
        pivot.setDefaultCommand(new ManualPivot(pivot, () -> gp.getRightY() < 0.3 || gp.getRightY() > 0.3 ? gp.getRightY() : 0d));

        Extension extension = new Extension(hardwareMap);
        extension.setDefaultCommand(new ManualExtension(extension, () -> gp.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - gp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        Gripper gripper = new Gripper(hardwareMap);
        gp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                new InstantCommand(gripper::close, gripper),
                new InstantCommand(gripper::open, gripper),
                () -> gripper.isOpen
        ));
//        gp1.getGamepadButton(GamepadKeys.Button.X).whenHeld(new InstantCommand(gripper::aliniat, gripper));
//        gp1.getGamepadButton(GamepadKeys.Button.X).whenReleased(new InstantCommand(gripper::turnDefault, gripper));
//
//        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenHeld(new InstantCommand(gripper::turnLeft, gripper));
//        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenHeld(new InstantCommand(gripper::turnRight, gripper));
//        gp2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenReleased(new InstantCommand(gripper::turnDefault, gripper));
//        gp2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenReleased(new InstantCommand(gripper::turnDefault, gripper));


        Arm arm = new Arm(hardwareMap);
        gp.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(new InstantCommand(arm::outtake, arm));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new InstantCommand(arm::outtake, arm));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(new InstantCommand(arm::intakeOverSubmersible));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(new InstantCommand(arm::outtakeSpecimen));
        gp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new PickUpSample(arm, gripper));
        gp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new PickUpSpecimen(arm, gripper));

        arm.outtake();

        waitForStart();

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            telemetry.addData("pos", extension.getPosition());
            telemetry.addData("angle", pivot.getAngle());
            telemetry.addData("angle target", Pivot.targetAngle);
            telemetry.update();
        }
    }
}
