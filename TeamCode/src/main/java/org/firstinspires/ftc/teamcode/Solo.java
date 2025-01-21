package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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

        Arm arm = new Arm(hardwareMap);
        Trigger pivotIsUp = new Trigger(() -> pivot.getAngle() >= 45);
        Trigger pivotIsDown = new Trigger(() -> pivot.getAngle() < 45);

        pivotIsUp.whenActive(new AutoArmControl(arm, extension));

        pivotIsDown.whenActive(new InstantCommand(() -> gripper.turn(0), gripper));
        pivotIsDown.whenActive(new InstantCommand(arm::intakeSpecimen, arm));

        gp.getGamepadButton(GamepadKeys.Button.DPAD_UP).and(pivotIsDown).whenActive(new InstantCommand(arm::intakeSpecimen, arm));
        gp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).and(pivotIsDown).whenActive(new InstantCommand(arm::intakeOverSubmersible));
        gp.getGamepadButton(GamepadKeys.Button.X).and(pivotIsDown).whenActive(new PickUpSample(arm, gripper));

//        gp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenReleased(new InstantCommand(gripper::aliniat, gripper));
//        gp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON).whenPressed(new InstantCommand(gripper::turnDefault, gripper));


//        arm.outtake();

//        (new AutoArmControl(arm, extension)).schedule();

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
